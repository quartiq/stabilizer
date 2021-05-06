#![deny(warnings)]
#![no_std]
#![no_main]

use generic_array::typenum::U4;

use serde::Deserialize;

use dsp::{Accu, Complex, ComplexExt, Lockin, RPLL};

use stabilizer::net;

use stabilizer::hardware::{
    design_parameters, setup, Adc0Input, Adc1Input, AfeGain, Dac0Output,
    Dac1Output, InputStamper, AFE0, AFE1,
};

use miniconf::Miniconf;
use stabilizer::net::{Action, MqttInterface};

// A constant sinusoid to send on the DAC output.
// Full-scale gives a +/- 10.24V amplitude waveform. Scale it down to give +/- 1V.
const ONE: i16 = ((1.0 / 10.24) * u16::MAX as f32) as _;
const SQRT2: i16 = (ONE as f32 * 0.707) as _;
const DAC_SEQUENCE: [i16; design_parameters::SAMPLE_BUFFER_SIZE] =
    [ONE, SQRT2, 0, -SQRT2, -ONE, -SQRT2, 0, SQRT2];

#[derive(Copy, Clone, Debug, Deserialize, Miniconf)]
enum Conf {
    Power,
    Phase,
    PllFrequency,
    FrequencyDiscriminator,
    QuadratureReal,
    QuadratureImaginary,
    Reference,
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
    telemetry_period_secs: u16,
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

            output_conf: [Conf::QuadratureReal, Conf::QuadratureImaginary],
            telemetry_period_secs: 10,
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        mqtt: MqttInterface<Settings>,
        settings: Settings,

        timestamper: InputStamper,
        pll: RPLL,
        lockin: Lockin<U4>,
    }

    #[init(spawn=[settings_update])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = setup(c.core, c.device);

        let mqtt = MqttInterface::new(
            stabilizer.net.stack,
            "",
            &net::get_device_prefix(
                env!("CARGO_BIN_NAME"),
                stabilizer.net.mac_address,
            ),
            stabilizer.net.phy,
            stabilizer.cycle_counter,
        );

        let settings = Settings::default();

        let pll = RPLL::new(
            design_parameters::ADC_SAMPLE_TICKS_LOG2
                + design_parameters::SAMPLE_BUFFER_SIZE_LOG2,
        );

        // Spawn a settings update for default settings.
        c.spawn.settings_update().unwrap();

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
            mqtt,
            timestamper: stabilizer.timestamper,

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
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, lockin, timestamper, pll, settings], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        let lockin = c.resources.lockin;
        let settings = c.resources.settings;

        let mut pll_frequency = 0;

        let (sample_phase, sample_frequency) = match settings.lockin_mode {
            LockinMode::External => {
                let timestamp =
                    c.resources.timestamper.latest_timestamp().unwrap_or(None); // Ignore data from timer capture overflows.
                let (pll_phase, frequency) = c.resources.pll.update(
                    timestamp.map(|t| t as i32),
                    settings.pll_tc[0],
                    settings.pll_tc[1],
                );

                pll_frequency = frequency;

                let sample_frequency = ((pll_frequency
                    >> design_parameters::SAMPLE_BUFFER_SIZE_LOG2)
                    as i32)
                    .wrapping_mul(settings.lockin_harmonic);
                let sample_phase = settings.lockin_phase.wrapping_add(
                    pll_phase.wrapping_mul(settings.lockin_harmonic),
                );

                (sample_phase, sample_frequency)
            }

            LockinMode::Internal => {
                // Reference phase and frequency are known.
                let pll_phase = 0i32;
                let pll_frequency =
                    1i32 << (32 - design_parameters::SAMPLE_BUFFER_SIZE_LOG2);

                // Demodulation LO phase offset
                let phase_offset: i32 = 1 << 30;

                let sample_frequency = (pll_frequency as i32)
                    .wrapping_mul(settings.lockin_harmonic);
                let sample_phase = phase_offset.wrapping_add(
                    pll_phase.wrapping_mul(settings.lockin_harmonic),
                );

                (sample_phase, sample_frequency)
            }
        };

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
        for i in 0..dac_samples[0].len() {
            for channel in 0..2 {
                let value = match settings.output_conf[0] {
                    Conf::Power => output.abs_sqr() as i32 >> 16,
                    Conf::Phase => output.arg() >> 16,
                    Conf::FrequencyDiscriminator => {
                        (output.log2() << 24) as i32 >> 16
                    }
                    Conf::PllFrequency => pll_frequency as i32 >> 16,
                    Conf::QuadratureReal => output.re >> 16,
                    Conf::QuadratureImaginary => output.im >> 16,
                    Conf::Reference => DAC_SEQUENCE[i] as i32,
                };

                dac_samples[channel][i] = value as u16 ^ 0x8000;
            }
        }
    }

    #[idle(resources=[mqtt], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.resources.mqtt.lock(|mqtt| mqtt.update()) {
                Some(Action::Sleep) => cortex_m::asm::wfi(),
                Some(Action::UpdateSettings) => {
                    c.spawn.settings_update().unwrap()
                }
                _ => {}
            }
        }
    }

    #[task(priority = 1, resources=[mqtt, settings, afes])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = c.resources.mqtt.settings();

        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);

        c.resources.settings.lock(|current| *current = *settings);
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
