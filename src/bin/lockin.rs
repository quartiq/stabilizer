#![deny(warnings)]
#![no_std]
#![no_main]

use embedded_hal::digital::v2::InputPin;

use serde::Deserialize;

use dsp::{Accu, Complex, ComplexExt, Lockin, RPLL};

use stabilizer::net;

use stabilizer::hardware::{
    design_parameters, setup, Adc0Input, Adc1Input, AdcCode, AfeGain,
    Dac0Output, Dac1Output, DacCode, DigitalInput0, DigitalInput1,
    InputStamper, SystemTimer, AFE0, AFE1,
};

use miniconf::Miniconf;
use net::{NetworkState, NetworkUsers, Telemetry, TelemetryBuffer};

// A constant sinusoid to send on the DAC output.
// Full-scale gives a +/- 10.24V amplitude waveform. Scale it down to give +/- 1V.
const ONE: i16 = ((1.0 / 10.24) * i16::MAX as f32) as _;
const SQRT2: i16 = (ONE as f32 * 0.707) as _;
const DAC_SEQUENCE: [i16; design_parameters::SAMPLE_BUFFER_SIZE] =
    [ONE, SQRT2, 0, -SQRT2, -ONE, -SQRT2, 0, SQRT2];

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
enum Conf {
    Magnitude,
    Phase,
    ReferenceFrequency,
    LogPower,
    InPhase,
    Quadrature,
    Modulation,
}

#[derive(Copy, Clone, Debug, Miniconf, Deserialize, PartialEq)]
enum LockinMode {
    Internal,
    External,
}

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
pub struct Settings {
    afe: [AfeGain; 2],
    lockin_mode: LockinMode,

    pll_tc: [u8; 2],

    lockin_tc: u8,
    lockin_harmonic: i32,
    lockin_phase: i32,

    output_conf: [Conf; 2],
    telemetry_period: u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [AfeGain::G1; 2],

            lockin_mode: LockinMode::External,

            pll_tc: [21, 21], // frequency and phase settling time (log2 counter cycles)

            lockin_tc: 6,        // lockin lowpass time constant
            lockin_harmonic: -1, // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
            lockin_phase: 0,     // Demodulation LO phase offset

            output_conf: [Conf::InPhase, Conf::Quadrature],
            // The default telemetry period in seconds.
            telemetry_period: 10,
        }
    }
}

macro_rules! flatten_closures {
    ($fn:ident, $e:ident, $fun:block) => {
        $e.$fn(|$e| $fun ).unwrap()
    };
    ($fn:ident, $e:ident, $($es:ident),+, $fun:block) => {
        $e.$fn(|$e| flatten_closures!($fn, $($es),*, $fun)).unwrap()
    };
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = stabilizer::hardware::SystemTimer)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        telemetry: TelemetryBuffer,
        digital_inputs: (DigitalInput0, DigitalInput1),

        timestamper: InputStamper,
        pll: RPLL,
        lockin: Lockin<4>,
    }

    #[init(spawn=[settings_update, telemetry])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = setup(c.core, c.device);

        let network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            stabilizer.cycle_counter,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
        );

        let settings = Settings::default();

        let pll = RPLL::new(
            design_parameters::ADC_SAMPLE_TICKS_LOG2
                + design_parameters::SAMPLE_BUFFER_SIZE_LOG2,
        );

        // Spawn a settings and telemetry update for default settings.
        c.spawn.settings_update().unwrap();
        c.spawn.telemetry().unwrap();

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
            telemetry: net::TelemetryBuffer::default(),

            settings,

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
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, lockin, timestamper, pll, settings, telemetry], priority=2)]
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
                    (pll_frequency
                        >> design_parameters::SAMPLE_BUFFER_SIZE_LOG2)
                        as i32,
                )
            }
            LockinMode::Internal => {
                // Reference phase and frequency are known.
                (
                    1i32 << 30,
                    1i32 << (32 - design_parameters::SAMPLE_BUFFER_SIZE_LOG2),
                )
            }
        };

        let sample_frequency =
            reference_frequency.wrapping_mul(settings.lockin_harmonic);
        let sample_phase = settings.lockin_phase.wrapping_add(
            reference_phase.wrapping_mul(settings.lockin_harmonic),
        );

        flatten_closures!(with_buffer, adc0, adc1, dac0, dac1, {
            let adc_samples = [adc0, adc1];
            let mut dac_samples = [dac0, dac1];

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
                for (i, sample) in samples.iter_mut().enumerate() {
                    let value = match settings.output_conf[channel] {
                        Conf::Magnitude => output.abs_sqr() as i32 >> 16,
                        Conf::Phase => output.arg() >> 16,
                        Conf::LogPower => (output.log2() << 24) as i32 >> 16,
                        Conf::ReferenceFrequency => {
                            reference_frequency as i32 >> 16
                        }
                        Conf::InPhase => output.re >> 16,
                        Conf::Quadrature => output.im >> 16,
                        Conf::Modulation => DAC_SEQUENCE[i] as i32,
                    };

                    *sample = DacCode::from(value as i16).0;
                }
            }
            // Update telemetry measurements.
            telemetry.adcs =
                [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[1][0])];

            telemetry.dacs =
                [DacCode(dac_samples[0][0]), DacCode(dac_samples[1][0])];
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

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { stm32h7xx_hal::ethernet::interrupt_handler() }
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
