//! # Lockin
//!
//! THe `lockin` application implements a lock-in amplifier using either an external or internally
//! generated
//!
//! ## Features
//! * Up to 800 kHz sampling
//! * Up to 400 kHz modulation frequency
//! * Supports internal and external reference sources:
//!     1. Internal: Generate reference internally and output on one of the channel outputs
//!     2. External: Reciprocal PLL, reference input applied to DI0.
//! * Adjustable PLL and locking time constants
//! * Adjustable phase offset and harmonic index
//! * Run-time configurable output modes (in-phase, quadrature, magnitude, log2 power, phase, frequency)
//! * Input/output data streamng via UDP
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

use dsp::{Accu, Complex, ComplexExt, Lockin, RPLL};
use stabilizer::{
    configuration,
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        design_parameters,
        embedded_hal::digital::v2::InputPin,
        hal,
        input_stamper::InputStamper,
        signal_generator,
        system_timer::SystemTimer,
        DigitalInput0, DigitalInput1, AFE0, AFE1,
    },
    net::{
        data_stream::{BlockGenerator, StreamTarget},
        miniconf::Miniconf,
        serde::Deserialize,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
enum Conf {
    /// Output the lockin magnitude.
    Magnitude,
    /// Output the phase of the lockin
    Phase,
    /// Output the lockin reference frequency as a sinusoid
    ReferenceFrequency,
    /// Output the logarithmic power of the lockin
    LogPower,
    /// Output the in-phase component of the lockin signal.
    InPhase,
    /// Output the quadrature component of the lockin signal.
    Quadrature,
    /// Output the lockin internal modulation frequency as a sinusoid
    Modulation,
}

#[derive(Copy, Clone, Debug, Miniconf, Deserialize, PartialEq)]
enum LockinMode {
    /// Utilize an internally generated reference for demodulation
    Internal,
    /// Utilize an external modulation signal supplied to DI0
    External,
}

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
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

    /// Specifies the operational mode of the lockin.
    ///
    /// # Path
    /// `lockin_mode`
    ///
    /// # Value
    /// One of the variants of [LockinMode] enclosed in double quotes.
    lockin_mode: LockinMode,

    /// Specifis the PLL time constant.
    ///
    /// # Path
    /// `pll_tc/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// The PLL time constant as an unsigned byte (0-255).
    pll_tc: [u8; 2],

    /// Specifies the lockin time constant.
    ///
    /// # Path
    /// `lockin_tc`
    ///
    /// # Value
    /// The lockin low-pass time constant as an unsigned byte (0-255).
    lockin_tc: u8,

    /// Specifies which harmonic to use for the lockin.
    ///
    /// # Path
    /// `lockin_harmonic`
    ///
    /// # Value
    /// Harmonic index of the LO. -1 to _de_modulate the fundamental (complex conjugate)
    lockin_harmonic: i32,

    /// Specifies the LO phase offset.
    ///
    /// # Path
    /// `lockin_phase`
    ///
    /// # Value
    /// Demodulation LO phase offset. Units are in terms of i32, where [i32::MIN] is equivalent to
    /// -pi and [i32::MAX] is equivalent to +pi.
    lockin_phase: i32,

    /// Specifies DAC output mode.
    ///
    /// # Path
    /// `output_conf/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// One of the variants of [Conf] enclosed in double quotes.
    output_conf: [Conf; 2],

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
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [Gain::G1; 2],

            lockin_mode: LockinMode::External,

            pll_tc: [21, 21], // frequency and phase settling time (log2 counter cycles)

            lockin_tc: 6,        // lockin lowpass time constant
            lockin_harmonic: -1, // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
            lockin_phase: 0,     // Demodulation LO phase offset

            output_conf: [Conf::InPhase, Conf::Quadrature],
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, monotonic = stabilizer::hardware::system_timer::SystemTimer)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        telemetry: TelemetryBuffer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        generator: BlockGenerator,
        signal_generator: signal_generator::SignalGenerator,

        timestamper: InputStamper,
        pll: RPLL,
        lockin: Lockin<4>,
    }

    #[init(spawn=[settings_update, telemetry, ethernet_link])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) =
            hardware::setup::setup(c.core, c.device);

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            stabilizer.cycle_counter,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
        );

        let generator = network.enable_streaming();

        let settings = Settings::default();

        let pll = RPLL::new(
            configuration::ADC_SAMPLE_TICKS_LOG2
                + configuration::SAMPLE_BUFFER_SIZE_LOG2,
        );

        // Spawn a settings and telemetry update for default settings.
        c.spawn.settings_update().unwrap();
        c.spawn.telemetry().unwrap();

        // Spawn the ethernet link servicing task.
        c.spawn.ethernet_link().unwrap();

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
            network,
            digital_inputs: stabilizer.digital_inputs,
            timestamper: stabilizer.timestamper,
            telemetry: TelemetryBuffer::default(),
            signal_generator: signal_generator::SignalGenerator::new(
                signal_generator::Config {
                    // Same frequency as batch size.
                    frequency: (1u64
                        << (32 - design_parameters::SAMPLE_BUFFER_SIZE_LOG2))
                        as u32,

                    // 1V Amplitude
                    amplitude: DacCode::from(1.0).into(),

                    signal: signal_generator::SignalConfig::Cosine,
                },
            ),

            settings,
            generator,

            pll,
            lockin: Lockin::default(),
        }
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of a externally (DI0) referenced PLL lockin on the ADC0 signal.
    /// It outputs either I/Q or power/phase on DAC0/DAC1. Data is normalized to full scale.
    /// PLL bandwidth, filter bandwidth, slope, and x/y or power/phase post-filters are available.
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, lockin, timestamper, pll, settings, telemetry, generator, signal_generator], priority=2)]
    #[inline(never)]
    #[link_section = ".itcm.process"]
    fn process(mut c: process::Context) {
        let process::Resources {
            adcs: (ref mut adc0, ref mut adc1),
            dacs: (ref mut dac0, ref mut dac1),
            ref settings,
            ref mut telemetry,
            ref mut lockin,
            ref mut pll,
            ref mut timestamper,
            ref mut generator,
            ref mut signal_generator,
        } = c.resources;

        let (reference_phase, reference_frequency) = match settings.lockin_mode
        {
            LockinMode::External => {
                let timestamp = timestamper.latest_timestamp().unwrap_or(None); // Ignore data from timer capture overflows.
                let (pll_phase, pll_frequency) = pll.update(
                    timestamp.map(|t| t as i32),
                    settings.pll_tc[0],
                    settings.pll_tc[1],
                );
                (
                    pll_phase,
                    (pll_frequency >> configuration::SAMPLE_BUFFER_SIZE_LOG2)
                        as i32,
                )
            }
            LockinMode::Internal => {
                // Reference phase and frequency are known.
                (
                    1i32 << 30,
                    1i32 << (32 - configuration::SAMPLE_BUFFER_SIZE_LOG2),
                )
            }
        };

        let sample_frequency =
            reference_frequency.wrapping_mul(settings.lockin_harmonic);
        let sample_phase = settings.lockin_phase.wrapping_add(
            reference_phase.wrapping_mul(settings.lockin_harmonic),
        );

        (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
            let adc_samples = [adc0, adc1];
            let mut dac_samples = [dac0, dac1];

            // Preserve instruction and data ordering w.r.t. DMA flag access.
            fence(Ordering::SeqCst);

            let output: Complex<i32> = adc_samples[0]
                .iter()
                // Zip in the LO phase.
                .zip(Accu::new(sample_phase, sample_frequency))
                // Convert to signed, MSB align the ADC sample, update the Lockin (demodulate, filter)
                .map(|(&sample, phase)| {
                    let s = (sample as i16 as i32) << 16;
                    lockin.update(s, phase, settings.lockin_tc)
                })
                // Decimate
                .last()
                .unwrap()
                * 2; // Full scale assuming the 2f component is gone.

            // Convert to DAC data.
            for (channel, samples) in dac_samples.iter_mut().enumerate() {
                for sample in samples.iter_mut() {
                    let value = match settings.output_conf[channel] {
                        Conf::Magnitude => output.abs_sqr() as i32 >> 16,
                        Conf::Phase => output.arg() >> 16,
                        Conf::LogPower => (output.log2() << 24) as i32 >> 16,
                        Conf::ReferenceFrequency => {
                            reference_frequency as i32 >> 16
                        }
                        Conf::InPhase => output.re >> 16,
                        Conf::Quadrature => output.im >> 16,

                        Conf::Modulation => {
                            signal_generator.next().unwrap() as i32
                        }
                    };

                    *sample = DacCode::from(value as i16).0;
                }
            }

            // Stream data
            generator.send(&adc_samples, &dac_samples);

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

    #[task(priority = 1, resources=[network, settings, afes])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.resources.network.miniconf.settings();

        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);

        c.resources.settings.lock(|current| *current = *settings);

        let target = settings.stream_target.into();
        c.resources.network.direct_stream(target);
    }

    #[task(priority = 1, resources=[network, digital_inputs, settings, telemetry], schedule=[telemetry])]
    fn telemetry(mut c: telemetry::Context) {
        let mut telemetry: TelemetryBuffer =
            c.resources.telemetry.lock(|telemetry| *telemetry);

        telemetry.digital_inputs = [
            c.resources.digital_inputs.0.is_high().unwrap(),
            c.resources.digital_inputs.1.is_high().unwrap(),
        ];

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

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
