#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

#[macro_use]
extern crate log;

use rtic::cyccnt::{Instant, U32Ext};

use heapless::{consts::*, String};

use stabilizer::{
    hardware, server, ADC_SAMPLE_TICKS_LOG2, SAMPLE_BUFFER_SIZE_LOG2,
};

use dsp::{iir, iir_int, lockin::Lockin, rpll::RPLL, Accu};
use hardware::{
    Adc0Input, Adc1Input, Dac0Output, Dac1Output, InputStamper, AFE0, AFE1,
};

const SCALE: f32 = i16::MAX as _;

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        net_interface: hardware::Ethernet,

        // Format: iir_state[ch][cascade-no][coeff]
        #[init([[iir::Vec5([0.; 5]); IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR::new(1./(1 << 16) as f32, -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],

        timestamper: InputStamper,
        pll: RPLL,
        lockin: Lockin,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let pll = RPLL::new(ADC_SAMPLE_TICKS_LOG2 + SAMPLE_BUFFER_SIZE_LOG2);

        let lockin = Lockin::new(
            iir_int::Vec5::lowpass(1e-3, 0.707, 2.), // TODO: expose
        );

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start recording digital input timestamps.
        stabilizer.timestamp_timer.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        // Enable the timestamper.
        stabilizer.timestamper.start();

        init::LateResources {
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            net_interface: stabilizer.net.interface,
            timestamper: stabilizer.timestamper,

            pll,
            lockin,
        }
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of a externally (DI0) referenced PLL lockin on the ADC0 signal.
    /// It outputs either I/Q or power/phase on DAC0/DAC1. Data is normalized to full scale.
    /// PLL bandwidth, filter bandwidth, slope, and x/y or power/phase post-filters are available.
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, iir_state, iir_ch, lockin, timestamper, pll], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        let iir_ch = c.resources.iir_ch;
        let iir_state = c.resources.iir_state;
        let lockin = c.resources.lockin;

        let timestamp = c
            .resources
            .timestamper
            .latest_timestamp()
            .unwrap_or_else(|t| t) // Ignore timer capture overflows.
            .map(|t| t as i32);
        let (pll_phase, pll_frequency) = c.resources.pll.update(
            timestamp,
            22, // frequency settling time (log2 counter cycles), TODO: expose
            22, // phase settling time, TODO: expose
        );

        // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
        let harmonic: i32 = -1; // TODO: expose
                                // Demodulation LO phase offset
        let phase_offset: i32 = 0; // TODO: expose

        let sample_frequency = ((pll_frequency
            // .wrapping_add(1 << SAMPLE_BUFFER_SIZE_LOG2 - 1)  // half-up rounding bias
            >> SAMPLE_BUFFER_SIZE_LOG2) as i32)
            .wrapping_mul(harmonic);
        let sample_phase =
            phase_offset.wrapping_add(pll_phase.wrapping_mul(harmonic));

        let output = adc_samples[0]
            .iter()
            .zip(Accu::new(sample_phase, sample_frequency))
            // Convert to signed, MSB align the ADC sample.
            .map(|(&sample, phase)| {
                lockin.update((sample as i16 as i32) << 16, phase)
            })
            .last()
            .unwrap();

        // convert i/q to power/phase,
        let power_phase = true; // TODO: expose

        let mut output = if power_phase {
            // Convert from IQ to power and phase.
            [output.abs_sqr() as _, output.arg() as _]
        } else {
            [output.0 as _, output.1 as _]
        };

        // Filter power and phase through IIR filters.
        // Note: Normalization to be done in filters. Phase will wrap happily.
        for j in 0..iir_state[0].len() {
            for k in 0..output.len() {
                output[k] =
                    iir_ch[k][j].update(&mut iir_state[k][j], output[k]);
            }
        }

        // Note(unsafe): range clipping to i16 is ensured by IIR filters above.
        // Convert to DAC data.
        for i in 0..dac_samples[0].len() {
            unsafe {
                dac_samples[0][i] =
                    output[0].to_int_unchecked::<i16>() as u16 ^ 0x8000;
                dac_samples[1][i] =
                    output[1].to_int_unchecked::<i16>() as u16 ^ 0x8000;
            }
        }
    }

    #[idle(resources=[net_interface, iir_state, iir_ch, afes])]
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
                                            x0: iir_state[0][0].0[0],
                                            y0: iir_state[0][0].0[2],
                                            x1: iir_state[1][0].0[0],
                                            y1: iir_state[1][0].0[2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                // "_b" means cascades 2nd IIR
                                "stabilizer/iir_b/state": (|| { let state = c.resources.iir_state.lock(|iir_state|
                                        server::Status {
                                            t: time,
                                            x0: iir_state[0][IIR_CASCADE_LENGTH-1].0[0],
                                            y0: iir_state[0][IIR_CASCADE_LENGTH-1].0[2],
                                            x1: iir_state[1][IIR_CASCADE_LENGTH-1].0[0],
                                            y1: iir_state[1][IIR_CASCADE_LENGTH-1].0[2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                "stabilizer/afe0/gain": (|| c.resources.afes.0.get_gain()),
                                "stabilizer/afe1/gain": (|| c.resources.afes.1.get_gain())
                            ],

                            modifiable_attributes: [
                                "stabilizer/iir0/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][0] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir1/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][0] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir_b0/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][IIR_CASCADE_LENGTH-1] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir_b1/state": server::IirRequest,(|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][IIR_CASCADE_LENGTH-1] = req.iir;

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
        panic!("ADC0 input overrun");
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
