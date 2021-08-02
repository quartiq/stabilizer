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
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Livestreaming
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stabilizer::net::data_stream](../stabilizer/net/data_stream/index.html) for more information.
#![deny(warnings)]
#![no_std]
#![no_main]

use core::sync::atomic::{fence, Ordering};

use mutex_trait::prelude::*;

use dsp::iir;
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        embedded_hal::digital::v2::InputPin,
        hal,
        signal_generator::{self, SignalGenerator},
        system_timer::SystemTimer,
        DigitalInput0, DigitalInput1, AFE0, AFE1,
    },
    net::{
        self,
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Miniconf,
        serde::Deserialize,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;

#[derive(Clone, Copy, Debug, Deserialize, Miniconf)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    afe: [Gain; 2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    /// * <m> specifies which cascade to configure. <m> := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],

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
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// See [signal_generator::BasicConfig#miniconf]
    signal_generator: [signal_generator::BasicConfig; 2],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
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

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, monotonic = stabilizer::hardware::system_timer::SystemTimer)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        digital_inputs: (DigitalInput0, DigitalInput1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        network: NetworkUsers<Settings, Telemetry>,
        generator: FrameGenerator,
        signal_generator: [SignalGenerator; 2],

        settings: Settings,
        telemetry: TelemetryBuffer,

        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
    }

    #[init(spawn=[telemetry, settings_update, ethernet_link])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup::setup(
            c.core,
            c.device,
            BATCH_SIZE,
            1 << SAMPLE_TICKS_LOG2,
        );

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            stabilizer.cycle_counter,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
            net::parse_or_default_broker(option_env!("BROKER")),
        );

        let generator = network
            .configure_streaming(StreamFormat::AdcDacData, BATCH_SIZE as u8);

        // Spawn a settings update for default settings.
        c.spawn.settings_update().unwrap();
        c.spawn.telemetry().unwrap();

        // Spawn the ethernet link period check task.
        c.spawn.ethernet_link().unwrap();

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        let settings = Settings::default();

        init::LateResources {
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            generator,
            network,
            digital_inputs: stabilizer.digital_inputs,
            telemetry: TelemetryBuffer::default(),
            settings,
            signal_generator: [
                SignalGenerator::new(
                    settings.signal_generator[0]
                        .try_into_config(SAMPLE_TICKS_LOG2)
                        .unwrap(),
                ),
                SignalGenerator::new(
                    settings.signal_generator[1]
                        .try_into_config(SAMPLE_TICKS_LOG2)
                        .unwrap(),
                ),
            ],
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
    #[task(binds=DMA1_STR4, resources=[adcs, digital_inputs, dacs, iir_state, settings, signal_generator, telemetry, generator], priority=2)]
    #[inline(never)]
    #[link_section = ".itcm.process"]
    fn process(mut c: process::Context) {
        let process::Resources {
            adcs: (ref mut adc0, ref mut adc1),
            dacs: (ref mut dac0, ref mut dac1),
            ref digital_inputs,
            ref settings,
            ref mut iir_state,
            ref mut telemetry,
            ref mut generator,
            ref mut signal_generator,
        } = c.resources;

        let digital_inputs = [
            digital_inputs.0.is_high().unwrap(),
            digital_inputs.1.is_high().unwrap(),
        ];
        telemetry.digital_inputs = digital_inputs;

        let hold =
            settings.force_hold || (digital_inputs[1] && settings.allow_hold);

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
                                ch.update(state, yi, hold)
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
            const N: usize = BATCH_SIZE * core::mem::size_of::<u16>();
            generator.add::<_, { N * 4 }>(|buf| {
                for (data, buf) in adc_samples
                    .iter()
                    .chain(dac_samples.iter())
                    .zip(buf.chunks_exact_mut(N))
                {
                    let data = unsafe {
                        core::slice::from_raw_parts(
                            data.as_ptr() as *const u8,
                            N,
                        )
                    };
                    buf.copy_from_slice(data)
                }
            });

            // Update telemetry measurements.
            telemetry.adcs =
                [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[1][0])];

            telemetry.dacs =
                [DacCode(dac_samples[0][0]), DacCode(dac_samples[1][0])];

            // Preserve instruction and data ordering w.r.t. DMA flag access.
            fence(Ordering::SeqCst);
        });
    }

    #[idle(resources=[network], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.resources.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    c.spawn.settings_update().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, resources=[network, afes, settings, signal_generator])]
    fn settings_update(mut c: settings_update::Context) {
        // Update the IIR channels.
        let settings = c.resources.network.miniconf.settings();
        c.resources.settings.lock(|current| *current = *settings);

        // Update AFEs
        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);

        // Update the signal generators
        for (i, &config) in settings.signal_generator.iter().enumerate() {
            match config.try_into_config(SAMPLE_TICKS_LOG2) {
                Ok(config) => {
                    c.resources
                        .signal_generator
                        .lock(|generator| generator[i].update_waveform(config));
                }
                Err(err) => log::error!(
                    "Failed to update signal generation on DAC{}: {:?}",
                    i,
                    err
                ),
            }
        }

        let target = settings.stream_target.into();
        c.resources.network.direct_stream(target);
    }

    #[task(priority = 1, resources=[network, settings, telemetry], schedule=[telemetry])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.resources.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = c
            .resources
            .settings
            .lock(|settings| (settings.afe, settings.telemetry_period));

        c.resources
            .network
            .telemetry
            .publish(&telemetry.finalize(gains[0], gains[1]));

        // Schedule the telemetry task in the future.
        c.schedule
            .telemetry(
                c.scheduled
                    + SystemTimer::ticks_from_secs(telemetry_period as u32),
            )
            .unwrap();
    }

    #[task(priority = 1, resources=[network], schedule=[ethernet_link])]
    fn ethernet_link(c: ethernet_link::Context) {
        c.resources.network.processor.handle_link();
        c.schedule
            .ethernet_link(c.scheduled + SystemTimer::ticks_from_secs(1))
            .unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
