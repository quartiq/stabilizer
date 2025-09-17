//! # Dual IIR
//!
//! The Dual IIR application exposes two configurable channels. Stabilizer samples input at a fixed
//! rate, digitally filters the data, and then generates filtered output signals on the respective
//! channel outputs.
//!
//! ## Features
//! * Two indpenendent channels
//! * up to 800 kHz rate, timed sampling
//! * Run-time filter configuration
//! * Input/Output data streaming
//! * Down to 2 µs latency
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//!
//! ## Settings
//! Refer to the [DualIir] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [stabilizer::net::telemetry::Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stabilizer::net::data_stream] for more information.
#![no_std]
#![no_main]

use core::sync::atomic::{fence, Ordering};
use miniconf::{Leaf, Tree};

use rtic_monotonics::Monotonic;

use fugit::ExtU32 as _;

use idsp::iir;

use serde::{Deserialize, Serialize};
use signal_generator::{self, Source};
use stream::{FrameGenerator, StreamFormat, StreamTarget};
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, Pgia, SerialTerminal, SystemTimer,
        Systick, UsbDevice,
    },
    net::{
        telemetry::TelemetryBuffer,
        NetworkState, NetworkUsers,
    },
    settings::NetSettings,
};

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

#[derive(Clone, Debug, Tree)]
pub struct Settings {
    dual_iir: DualIir,
    net: NetSettings,
}

impl stabilizer::settings::AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            dual_iir: DualIir::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            dual_iir: DualIir::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

#[derive(Clone, Debug, Tree)]
pub struct BiquadRepr {
    /// Biquad parameters
    #[tree(rename="typ", typ="&str", with=miniconf::str_leaf, defer=self.repr)]
    _typ: (),
    repr: iir::BiquadRepr<f32, f32>,
}

impl Default for BiquadRepr {
    fn default() -> Self {
        let mut i = iir::Biquad::IDENTITY;
        i.set_min(-i16::MAX as _);
        i.set_max(i16::MAX as _);
        Self {
            _typ: (),
            repr: iir::BiquadRepr::Raw(Leaf(i)),
        }
    }
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Default)]
pub enum Run {
    #[default]
    /// Run
    Run,
    /// Hold
    Hold,
    /// Hold controlled by corresponding digital input
    External,
}

impl Run {
    fn run(&self, di: bool) -> bool {
        match self {
            Self::Run => true,
            Self::Hold => false,
            Self::External => di,
        }
    }
}

#[derive(Clone, Debug, Tree, Default)]
pub struct Channel {
    /// Analog Front End (AFE) gain.
    #[tree(with=miniconf::leaf)]
    gain: Gain,
    /// Biquad
    biquad: [BiquadRepr; IIR_CASCADE_LENGTH],
    /// Run/Hold behavior
    #[tree(with=miniconf::leaf)]
    run: Run,
    /// Signal generator configuration to add to the DAC0/DAC1 outputs
    source: signal_generator::Config,
}

impl Channel {
    fn build(&self) -> Result<Active, signal_generator::Error> {
        Ok(Active {
            source: self
                .source
                .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip())
                .unwrap(),
            state: Default::default(),
            run: self.run,
            biquad: self.biquad.each_ref().map(|biquad| {
                biquad.repr.build::<f32>(
                    SAMPLE_PERIOD,
                    1.0,
                    DacCode::LSB_PER_VOLT,
                )
            }),
        })
    }
}

#[derive(Clone, Debug, Tree)]
pub struct DualIir {
    /// Channel configuration
    ch: [Channel; 2],
    /// Trigger both signal sources
    #[tree(with=miniconf::leaf)]
    trigger: bool,
    /// Telemetry output period in seconds.
    #[tree(with=miniconf::leaf)]
    telemetry_period: f32,
    /// Target IP and port for UDP streaming.
    ///
    /// Can be multicast.
    #[tree(with=miniconf::leaf)]
    stream: StreamTarget,
}

impl Default for DualIir {
    fn default() -> Self {
        Self {
            telemetry_period: 10.0,
            trigger: false,
            stream: Default::default(),
            ch: Default::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Active {
    run: Run,
    biquad: [iir::Biquad<f32>; IIR_CASCADE_LENGTH],
    state: [[f32; 4]; IIR_CASCADE_LENGTH],
    source: Source,
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<DualIir, 8>,
        settings: Settings,
        active: [Active; 2],
        telemetry: TelemetryBuffer,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings, 9>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (stabilizer, _pounder) = hardware::setup::setup::<Settings, 9>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &stabilizer.settings.net,
            stabilizer.metadata,
        );

        let generator = network.configure_streaming(StreamFormat::AdcDacData);

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            active: stabilizer
                .settings
                .dual_iir
                .ch
                .each_ref()
                .map(|a| a.build().unwrap()),
            telemetry: TelemetryBuffer::default(),
            settings: stabilizer.settings,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
        start::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local=[sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(100.millis()).await;
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
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
    #[task(
        binds=DMA1_STR4,
        local=[digital_inputs, adcs, dacs, generator, source: [[i16; BATCH_SIZE]; 2] = [[0; BATCH_SIZE]; 2]],
        shared=[active, telemetry],
        priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            active, telemetry, ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            generator,
            source,
            ..
        } = c.local;

        (active, telemetry).lock(|active, telemetry| {
            (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                // Preserve instruction and data ordering w.r.t. DMA flag access before and after.
                fence(Ordering::SeqCst);
                let adc: [&[u16; BATCH_SIZE]; 2] = [
                    (**adc0).try_into().unwrap(),
                    (**adc1).try_into().unwrap(),
                ];
                let mut dac: [&mut [u16; BATCH_SIZE]; 2] =
                    [(*dac0).try_into().unwrap(), (*dac1).try_into().unwrap()];

                for ((((adc, dac), active), di), source) in adc
                    .into_iter()
                    .zip(dac.iter_mut())
                    .zip(active.iter_mut())
                    .zip(telemetry.digital_inputs)
                    .zip(source.iter())
                {
                    for ((adc, dac), source) in
                        adc.iter().zip(dac.iter_mut()).zip(source)
                    {
                        let x = f32::from(*adc as i16);
                        let y = active
                            .biquad
                            .iter()
                            .zip(active.state.iter_mut())
                            .fold(x, |y, (ch, state)| {
                                let filter = if active.run.run(di) {
                                    ch
                                } else {
                                    &iir::Biquad::HOLD
                                };
                                filter.update(state, y)
                            });

                        // Note(unsafe): The filter limits must ensure that the value is in range.
                        // The truncation introduces 1/2 LSB distortion.
                        let y: i16 = unsafe { y.to_int_unchecked() };
                        *dac = DacCode::from(y.saturating_add(*source)).0;
                    }
                }
                telemetry.adcs = [AdcCode(adc[0][0]), AdcCode(adc[1][0])];
                telemetry.dacs = [DacCode(dac[0][0]), DacCode(dac[1][0])];

                const N: usize = BATCH_SIZE * size_of::<i16>();
                generator.add(|buf| {
                    [adc[0], adc[1], dac[0], dac[1]]
                        .into_iter()
                        .zip(buf.chunks_exact_mut(N))
                        .map(|(data, buf)| {
                            buf.copy_from_slice(bytemuck::cast_slice(data))
                        })
                        .count()
                        * N
                });

                fence(Ordering::SeqCst);
            });
            *source = active.each_mut().map(|ch| {
                core::array::from_fn(|_| (ch.source.next().unwrap() >> 16) as _)
            });
            telemetry.digital_inputs =
                [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
        });
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.dual_iir))
            {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap();
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared.usb.lock(|usb| {
                        usb.state()
                            == usb_device::device::UsbDeviceState::Suspend
                    }) {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }

    #[task(priority = 1, local=[afes], shared=[network, settings, active])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes[0].set_gain(settings.dual_iir.ch[0].gain);
            c.local.afes[1].set_gain(settings.dual_iir.ch[1].gain);

            if settings.dual_iir.trigger {
                settings.dual_iir.trigger = false;
                let s = settings.dual_iir.ch.each_ref().map(|ch| {
                    let s = ch
                        .source
                        .build(SAMPLE_PERIOD, DacCode::FULL_SCALE.recip());
                    if let Err(err) = &s {
                        log::error!("Failed to update source: {:?}", err);
                    }
                    s
                });
                c.shared.active.lock(|ch| {
                    for (ch, s) in ch.iter_mut().zip(s) {
                        if let Ok(s) = s {
                            ch.source = s;
                        }
                    }
                });
            }
            let b = settings.dual_iir.ch.each_ref().map(|ch| {
                (
                    ch.run,
                    ch.biquad.each_ref().map(|b| {
                        b.repr.build::<f32>(
                            SAMPLE_PERIOD,
                            1.0,
                            DacCode::LSB_PER_VOLT,
                        )
                    }),
                )
            });
            c.shared.active.lock(|active| {
                for (a, b) in active.iter_mut().zip(b) {
                    (a.run, a.biquad) = b;
                }
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.dual_iir.stream));
        });
    }

    #[task(priority = 1, shared=[network, settings, telemetry], local=[cpu_temp_sensor])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let telemetry =
                c.shared.telemetry.lock(|telemetry| telemetry.clone());

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (
                        settings.dual_iir.ch.each_ref().map(|ch| ch.gain),
                        settings.dual_iir.telemetry_period,
                    )
                });

            c.shared.network.lock(|net| {
                net.telemetry.publish(&telemetry.finalize(
                    gains[0],
                    gains[1],
                    c.local.cpu_temp_sensor.get_temperature().unwrap(),
                ))
            });

            Systick::delay(((telemetry_period * 1000.0) as u32).millis()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()]);
            });

            c.shared.settings.lock(|settings| {
                if c.local.usb_terminal.poll(settings).unwrap() {
                    settings_update::spawn().unwrap()
                }
            });

            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) {
        loop {
            c.shared.network.lock(|net| net.processor.handle_link());
            Systick::delay(1.secs()).await;
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 4)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }

    #[task(binds = SPI4, priority = 4)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 4)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }
}
