#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

#[macro_use]
extern crate log;

use rtic::cyccnt::{Instant, U32Ext};

use heapless::{consts::*, String};

use stabilizer::{hardware, server};

use dsp::iir;
use hardware::{Adc0Input, Adc1Input, Dac0Output, Dac1Output, AFE0, AFE1};

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
        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
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
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, iir_state, iir_ch], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        for channel in 0..adc_samples.len() {
            for sample in 0..adc_samples[0].len() {
                let x = f32::from(adc_samples[channel][sample] as i16);
                let mut y = x;
                for i in 0..c.resources.iir_state[channel].len() {
                    y = c.resources.iir_ch[channel][i]
                        .update(&mut c.resources.iir_state[channel][i], y);
                }
                // Note(unsafe): The filter limits ensure that the value is in range.
                // The truncation introduces 1/2 LSB distortion.
                let y = unsafe { y.to_int_unchecked::<i16>() };
                // Convert to DAC code
                dac_samples[channel][sample] = y as u16 ^ 0x8000;
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
                                            x0: iir_state[0][0][0],
                                            y0: iir_state[0][0][2],
                                            x1: iir_state[1][0][0],
                                            y1: iir_state[1][0][2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                // "_b" means cascades 2nd IIR
                                "stabilizer/iir_b/state": (|| { let state = c.resources.iir_state.lock(|iir_state|
                                        server::Status {
                                            t: time,
                                            x0: iir_state[0][IIR_CASCADE_LENGTH-1][0],
                                            y0: iir_state[0][IIR_CASCADE_LENGTH-1][2],
                                            x1: iir_state[1][IIR_CASCADE_LENGTH-1][0],
                                            y1: iir_state[1][IIR_CASCADE_LENGTH-1][2],
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
