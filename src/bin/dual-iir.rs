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
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Livestreaming
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stabilizer::net::data_stream](../stabilizer/net/data_stream/index.html) for more information.
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{fence, Ordering};
use serde::{Deserialize, Serialize};

use rtic_monotonics::Monotonic;

use fugit::ExtU32;
use mutex_trait::prelude::*;

use idsp::iir;

use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        signal_generator::{self, SignalGenerator},
        timers::SamplingTimer,
        CpuDacOutput1, DigitalInput0, DigitalInput1, GpioDacSpi,
        SerialTerminal, SystemTimer, Systick, UsbDevice, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        telemetry::TelemetryBuffer,
        NetworkState, NetworkUsers,
    },
    settings::NetSettings,
};

use hal::traits::DacOut;

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 2;

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
    #[tree(depth = 3)]
    pub dual_iir: DualIir,

    #[tree(depth = 1)]
    pub net: NetSettings,
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

impl serial_settings::Settings<4> for Settings {
    fn reset(&mut self) {
        *self = Self {
            dual_iir: DualIir::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

#[derive(Clone, Debug, Tree, Serialize, Deserialize)]
pub struct DualIir {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    #[tree(depth = 1)]
    afe: [Gain; 2],

    ///Configure the internal DAC output.
    ///
    /// # Path
    /// `cpu_dac1`
    ///
    /// # Value
    /// Any value between 0 and 4095.
    cpu_dac1: u16,

    ///Configure the current_sense frontend offset dac when DI0 is low.
    ///
    /// # Path
    /// `frontend_offset_low`
    ///
    /// # Value
    /// Any value between 0 and 65535.
    frontend_offset_low: u16,

    ///Configure the current_sense frontend offset dac when DI0 is high.
    ///
    /// # Path
    /// `frontend_offset_high`
    ///
    /// # Value
    /// Any value between 0 and 65535.
    frontend_offset_high: u16,

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    /// * `<m>` specifies which cascade to configure. `<m>` := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// See [iir::Biquad]
    #[tree(depth = 2)]
    iir_ch: [[iir::Biquad<f32>; IIR_CASCADE_LENGTH]; 2],

    /// Specified true if DI1 should be used as a "hold" input.
    ///
    /// # Path
    /// `allow_hold`
    ///
    /// # Value
    /// "true" or "false"
    allow_hold: bool,

    /// Specified true if "hold" should be forced regardless of DI1 state and hold allowance.
    ///
    /// # Path
    /// `force_hold`
    ///
    /// # Value
    /// "true" or "false"
    force_hold: bool,

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any non-zero value less than 65536.
    telemetry_period: u16,

    /// Specifies the target for data livestreaming.
    ///
    /// # Path
    /// `stream_target`
    ///
    /// # Value
    /// See [StreamTarget#miniconf]
    stream_target: StreamTarget,

    /// Specifies the config for signal generators to add on to DAC0/DAC1 outputs.
    ///
    /// # Path
    /// `signal_generator/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// See [signal_generator::BasicConfig#miniconf]
    #[tree(depth = 2)]
    signal_generator: [signal_generator::BasicConfig; 2],
}

impl Default for DualIir {
    fn default() -> Self {
        let mut i = iir::Biquad::IDENTITY;
        i.set_min(-SCALE);
        i.set_max(SCALE);
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // CPU DAC1 output
            cpu_dac1: 0,
            // Frontend offset DACs
            frontend_offset_low: 0,
            frontend_offset_high: 0,
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[i; IIR_CASCADE_LENGTH]; 2],

            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 10,

            signal_generator: [signal_generator::BasicConfig::default(); 2],

            stream_target: StreamTarget::default(),
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use cortex_m::prelude::_embedded_hal_blocking_spi_Write;

    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<DualIir, 3>,
        settings: Settings,
        active_settings: DualIir,
        telemetry: TelemetryBuffer,
        signal_generator: [SignalGenerator; 2],
        di0_state: bool,
        current_offset: u16,
        gpio_dac_spi: GpioDacSpi,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings, 4>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        iir_state: [[[f32; 4]; IIR_CASCADE_LENGTH]; 2],
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        cpu_dac1: CpuDacOutput1,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let stabilizer = hardware::setup::setup::<Settings, 4>(
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
            active_settings: stabilizer.settings.dual_iir.clone(),
            telemetry: TelemetryBuffer::default(),
            signal_generator: [
                SignalGenerator::new(
                    stabilizer.settings.dual_iir.signal_generator[0]
                        .try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE)
                        .unwrap(),
                ),
                SignalGenerator::new(
                    stabilizer.settings.dual_iir.signal_generator[1]
                        .try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE)
                        .unwrap(),
                ),
            ],
            settings: stabilizer.settings,
            di0_state: false, // Initialize DI0 state as low (with pull-down)
            current_offset: 0, // Will be set by initial settings update
            gpio_dac_spi: stabilizer.gpio_dac_spi,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[[0.; 4]; IIR_CASCADE_LENGTH]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            cpu_dac1: stabilizer.cpu_dac1,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // // Test the SPI DAC
        // if let Err(err) = local.gpio_dac_spi.write(&[0x0000]) {
        //     log::error!("Failed to write to the DAC: {:?}", err);
        // }

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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[active_settings, signal_generator, telemetry, di0_state], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            active_settings,
            telemetry,
            signal_generator,
            di0_state,
            ..
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
            ..
        } = c.local;

        (active_settings, telemetry, signal_generator, di0_state).lock(
            |settings, telemetry, signal_generator, di0_state| {
                let digital_inputs =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                telemetry.digital_inputs = digital_inputs;

                // Check if DI0 state changed and trigger offset DAC update if needed
                let current_di0_state = digital_inputs[0];
                if *di0_state != current_di0_state {
                    *di0_state = current_di0_state;
                    // Trigger the offset DAC update task (non-blocking)
                    update_offset_dac::spawn().ok(); // Ignore spawn errors to maintain real-time constraints
                }

                let hold = settings.force_hold
                    || (digital_inputs[1] && settings.allow_hold);

                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    for channel in 0..adc_samples.len() {
                        adc_samples[channel]
                            .iter()
                            .zip(dac_samples[channel].iter_mut())
                            .zip(&mut signal_generator[channel])
                            .map(|((ai, di), signal)| {
                                let x = f32::from(*ai as i16);
                                let y = settings.iir_ch[channel]
                                    .iter()
                                    .zip(iir_state[channel].iter_mut())
                                    .fold(x, |yi, (ch, state)| {
                                        let filter = if hold {
                                            &iir::Biquad::HOLD
                                        } else {
                                            ch
                                        };

                                        filter.update(state, yi)
                                    });

                                // Note(unsafe): The filter limits must ensure that the value is in range.
                                // The truncation introduces 1/2 LSB distortion.
                                let y: i16 = unsafe { y.to_int_unchecked() };

                                let y = y.saturating_add(signal);

                                // Convert to DAC code
                                *di = DacCode::from(y).0;
                            })
                            .last();
                    }

                    // Stream the data.
                    const N: usize = BATCH_SIZE * core::mem::size_of::<i16>();
                    generator.add(|buf| {
                        for (data, buf) in adc_samples
                            .iter()
                            .chain(dac_samples.iter())
                            .zip(buf.chunks_exact_mut(N))
                        {
                            let data = unsafe {
                                core::slice::from_raw_parts(
                                    data.as_ptr() as *const MaybeUninit<u8>,
                                    N,
                                )
                            };
                            buf.copy_from_slice(data)
                        }
                        N * 4
                    });
                    // Update telemetry measurements.
                    telemetry.adcs = [
                        AdcCode(adc_samples[0][0]),
                        AdcCode(adc_samples[1][0]),
                    ];

                    telemetry.dacs = [
                        DacCode(dac_samples[0][0]),
                        DacCode(dac_samples[1][0]),
                    ];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.dual_iir))
            {
                NetworkState::SettingsChanged => {
                    settings_update::spawn().unwrap()
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

    #[task(priority = 1, local=[afes, cpu_dac1], shared=[network, settings, active_settings, signal_generator])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes.0.set_gain(settings.dual_iir.afe[0]);
            c.local.afes.1.set_gain(settings.dual_iir.afe[1]);
            c.local.cpu_dac1.set_value(settings.dual_iir.cpu_dac1);

            // Update the signal generators
            for (i, &config) in
                settings.dual_iir.signal_generator.iter().enumerate()
            {
                match config.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE)
                {
                    Ok(config) => {
                        c.shared.signal_generator.lock(|generator| {
                            generator[i].update_waveform(config)
                        });
                    }
                    Err(err) => log::error!(
                        "Failed to update signal generation on DAC{}: {:?}",
                        i,
                        err
                    ),
                }
            }

            c.shared
                .network
                .lock(|net| net.direct_stream(settings.dual_iir.stream_target));

            c.shared
                .active_settings
                .lock(|current| *current = settings.dual_iir.clone());
        });

        // Trigger initial offset DAC update
        update_offset_dac::spawn().unwrap();
    }

    #[task(priority = 2, shared=[di0_state, current_offset, active_settings, gpio_dac_spi])]
    async fn update_offset_dac(mut c: update_offset_dac::Context) {
        let new_offset = c.shared.active_settings.lock(|settings| {
            c.shared.di0_state.lock(|di0_state| {
                if *di0_state {
                    settings.frontend_offset_high
                } else {
                    settings.frontend_offset_low
                }
            })
        });

        // Update current offset and write to DAC only if it changed
        let should_update = c.shared.current_offset.lock(|current_offset| {
            if *current_offset != new_offset {
                *current_offset = new_offset;
                true
            } else {
                false
            }
        });

        if should_update {
            c.shared.gpio_dac_spi.lock(|gpio_dac_spi| {
                if let Err(err) = gpio_dac_spi.write(&[new_offset]) {
                    // After extensive debugging, it's been confirmed this error is benign
                    // in this specific context. The DAC is write-only, the data transmits
                    // successfully, and hardware/firmware pull-downs do not resolve the
                    // spurious receive flag. The error can be safely ignored.
                    if !matches!(err, hal::spi::Error::DuplexFailed) {
                        log::error!(
                            "Failed to update frontend offset DAC: {:?}",
                            err
                        );
                    }
                }
            });
        }
    }

    // #[task(priority = 1, local=[cpu_dac1], shared=[network, settings])]
    // async fn cpu_dac_update(mut c: cpu_dac_update::Context) {
    //     c.shared.settings.lock(|settings| {
    //         c.local.cpu_dac1.set_value(settings.dual_iir.cpu_dac1);
    //     });
    // }

    #[task(priority = 1, shared=[network, settings, telemetry], local=[cpu_temp_sensor])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let telemetry: TelemetryBuffer =
                c.shared.telemetry.lock(|telemetry| *telemetry);

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (settings.dual_iir.afe, settings.dual_iir.telemetry_period)
                });

            c.shared.network.lock(|net| {
                net.telemetry.publish(&telemetry.finalize(
                    gains[0],
                    gains[1],
                    c.local.cpu_temp_sensor.get_temperature().unwrap(),
                ))
            });

            Systick::delay((telemetry_period as u32).secs()).await;
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
