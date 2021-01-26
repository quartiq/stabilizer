#![deny(warnings)]
#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

use stm32h7xx_hal as hal;

#[macro_use]
extern crate log;

use rtic::cyccnt::{Instant, U32Ext};

use heapless::{consts::*, String};

// A constant sinusoid to send on the DAC output.
const DAC_SEQUENCE: [f32; 8] =
    [0.0, 0.707, 1.0, 0.707, 0.0, -0.707, -1.0, -0.707];

use dsp::{iir, iir_int, lockin::Lockin, reciprocal_pll::TimestampHandler};
use hardware::{Adc1Input, Dac0Output, Dac1Output, AFE0, AFE1};
use stabilizer::{
    hardware, server, ADC_SAMPLE_TICKS_LOG2, SAMPLE_BUFFER_SIZE_LOG2,
};

const SCALE: f32 = ((1 << 15) - 1) as f32;

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adc1: Adc1Input,
        dacs: (Dac0Output, Dac1Output),
        net_interface: hardware::Ethernet,

        #[init([0.; 5])]
        iir_state: iir::IIRState,

        #[init(iir::IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE })]
        iir: iir::IIR,

        pll: TimestampHandler,
        lockin: Lockin,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let pll = TimestampHandler::new(
            4, // relative PLL frequency bandwidth: 2**-4, TODO: expose
            3, // relative PLL phase bandwidth: 2**-3, TODO: expose
            ADC_SAMPLE_TICKS_LOG2 as usize,
            SAMPLE_BUFFER_SIZE_LOG2,
        );

        let lockin = Lockin::new(
            &iir_int::IIRState::default(), // TODO: lowpass, expose
        );

        // Enable ADC/DAC events
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            lockin,
            pll,
            afes: stabilizer.afes,
            adc1: stabilizer.adcs.1,
            dacs: stabilizer.dacs,
            net_interface: stabilizer.net.interface,
        }
    }

    /// Main DSP processing routine for Stabilizer.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    ///
    /// TODO: Document
    #[task(binds=DMA1_STR4, resources=[adc1, dacs, iir_state, iir, lockin, pll], priority=2)]
    fn process(mut c: process::Context) {
        let adc_samples = c.resources.adc1.acquire_buffer();
        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        // DAC0 always generates a fixed sinusoidal output.
        for (i, value) in DAC_SEQUENCE.iter().enumerate() {
            let y = value * i16::MAX as f32;
            // Note(unsafe): The DAC_SEQUENCE values are guaranteed to be normalized.
            let y = unsafe { y.to_int_unchecked::<i16>() };

            // Convert to DAC code
            dac_samples[0][i] = y as u16 ^ 0x8000;
        }

        // TODO: Verify that the DAC code is always generated at T=0
        let (pll_phase, pll_frequency) = c.resources.pll.update(Some(0));

        // Harmonic index of the LO: -1 to _de_modulate the fundamental
        let harmonic: i32 = -1;

        // Demodulation LO phase offset
        let phase_offset: i32 = 0;
        let sample_frequency = (pll_frequency as i32).wrapping_mul(harmonic);
        let mut sample_phase = phase_offset
            .wrapping_add((pll_phase as i32).wrapping_mul(harmonic));

        let mut phase = 0f32;

        for sample in adc_samples.iter() {
            // Convert to signed, MSB align the ADC sample.
            let input = (*sample as i16 as i32) << 16;

            // Obtain demodulated, filtered IQ sample.
            let output = c.resources.lockin.update(input, sample_phase);

            // Advance the sample phase.
            sample_phase = sample_phase.wrapping_add(sample_frequency);

            // Convert from IQ to phase.
            phase = output.phase() as _;
        }

        // Filter phase through an IIR.
        phase = c.resources.iir.update(&mut c.resources.iir_state, phase);

        for value in dac_samples[1].iter_mut() {
            *value = phase as u16 ^ 0x8000
        }
    }

    #[idle(resources=[net_interface, iir_state, iir, afes])]
    fn idle(mut c: idle::Context) -> ! {
        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets =
            smoltcp::socket::SocketSet::new(&mut socket_set_entries[..]);

        let mut rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_handle = {
            let tcp_rx_buffer =
                smoltcp::socket::TcpSocketBuffer::new(&mut rx_storage[..]);
            let tcp_tx_buffer =
                smoltcp::socket::TcpSocketBuffer::new(&mut tx_storage[..]);
            let tcp_socket =
                smoltcp::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
            sockets.add(tcp_socket)
        };

        let mut server = server::Server::new();

        let mut time = 0u32;
        let mut next_ms = Instant::now();

        // TODO: Replace with reference to CPU clock from CCDR.
        next_ms += 400_000.cycles();

        loop {
            let tick = Instant::now() > next_ms;

            if tick {
                next_ms += 400_000.cycles();
                time += 1;
            }

            {
                let socket =
                    &mut *sockets.get::<smoltcp::socket::TcpSocket>(tcp_handle);
                if socket.state() == smoltcp::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket
                        .listen(1235)
                        .unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else {
                    server.poll(socket, |req| {
                        info!("Got request: {:?}", req);
                        stabilizer::route_request!(req,
                            readable_attributes: [
                                "stabilizer/iir/state": (|| {
                                    let state = c.resources.iir_state.lock(|iir_state|
                                        server::Status {
                                            t: time,
                                            x0: iir_state[0],
                                            y0: iir_state[2],
                                            x1: iir_state[0],
                                            y1: iir_state[2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                "stabilizer/afe0/gain": (|| c.resources.afes.0.get_gain()),
                                "stabilizer/afe1/gain": (|| c.resources.afes.1.get_gain())
                            ],

                            modifiable_attributes: [
                                "stabilizer/iir/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir.lock(|iir| {
                                        if req.channel >= 1 {
                                            return Err(());
                                        }

                                        *iir = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/afe0/gain": hardware::AfeGain, (|gain| {
                                    c.resources.afes.0.set_gain(gain);
                                    Ok::<(), ()>(())
                                }),
                                "stabilizer/afe1/gain": hardware::AfeGain, (|gain| {
                                    c.resources.afes.1.set_gain(gain);
                                    Ok::<(), ()>(())
                                })
                            ]
                        )
                    });
                }
            }

            let sleep = match c.resources.net_interface.poll(
                &mut sockets,
                smoltcp::time::Instant::from_millis(time as i64),
            ) {
                Ok(changed) => !changed,
                Err(smoltcp::Error::Unrecognized) => true,
                Err(e) => {
                    info!("iface poll error: {:?}", e);
                    true
                }
            };

            if sleep {
                cortex_m::asm::wfi();
            }
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 input overrun");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 output error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 output error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
