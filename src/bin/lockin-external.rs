#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

use miniconf::{
    embedded_nal::{IpAddr, Ipv4Addr},
    minimq, MqttInterface, StringSet,
};

use serde::Deserialize;
use stabilizer::{hardware, hardware::design_parameters};

use dsp::{lockin::Lockin, rpll::RPLL, Accu};

use hardware::{
    Adc0Input, Adc1Input, AfeGain, Dac0Output, Dac1Output, InputStamper, AFE0,
    AFE1,
};

#[derive(Debug, Clone, Copy, Deserialize, StringSet)]
pub struct DspData {
    // frequency settling time (log2 counter cycles)
    frequency_settling_time: u8,

    // phase settling time
    phase_settling_time: u8,

    // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
    harmonic: i32,

    // Demodulation LO phase offset
    phase_offset: i32,

    // Log2 lowpass time constant
    time_constant: u8,
}

impl Default for DspData {
    fn default() -> Self {
        Self {
            frequency_settling_time: 21,
            phase_settling_time: 21,
            harmonic: -1,
            phase_offset: 0,
            time_constant: 6,
        }
    }
}

#[derive(Debug, Deserialize, StringSet)]
pub struct Settings {
    afe: [AfeGain; 2],
    dsp: DspData,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [AfeGain::G1, AfeGain::G1],
            dsp: DspData::default(),
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        clock: hardware::CycleCounter,
        mqtt_interface: MqttInterface<
            Settings,
            hardware::NetworkStack,
            minimq::consts::U256,
        >,

        timestamper: InputStamper,
        pll: RPLL,

        lockin: Lockin,
        parameters: DspData,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let mqtt_interface = {
            let mqtt_client = {
                let broker = IpAddr::V4(Ipv4Addr::new(10, 34, 16, 1));
                minimq::MqttClient::new(
                    broker,
                    "stabilizer",
                    stabilizer.net.stack,
                )
                .unwrap()
            };

            MqttInterface::new(mqtt_client, "stabilizer", Settings::default())
                .unwrap()
        };

        let pll = RPLL::new(
            design_parameters::ADC_SAMPLE_TICKS_LOG2
                + design_parameters::SAMPLE_BUFFER_SIZE_LOG2,
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
            mqtt_interface,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            timestamper: stabilizer.timestamper,
            clock: stabilizer.cycle_counter,
            pll,
            lockin: Lockin::default(),
            parameters: DspData::default(),
        }
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of a externally (DI0) referenced PLL lockin on the ADC0 signal.
    /// It outputs either I/Q or power/phase on DAC0/DAC1. Data is normalized to full scale.
    /// PLL bandwidth, filter bandwidth, slope, and x/y or power/phase post-filters are available.
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, lockin, timestamper, pll, parameters], priority=2)]
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
        let params = c.resources.parameters;

        let timestamp = c
            .resources
            .timestamper
            .latest_timestamp()
            .unwrap_or(None) // Ignore data from timer capture overflows.
            .map(|t| t as i32);
        let (pll_phase, pll_frequency) = c.resources.pll.update(
            timestamp,
            params.frequency_settling_time,
            params.phase_settling_time,
        );

        let sample_frequency = ((pll_frequency
            // half-up rounding bias
            // .wrapping_add(1 << design_parameters::SAMPLE_BUFFER_SIZE_LOG2 - 1)
            >> design_parameters::SAMPLE_BUFFER_SIZE_LOG2)
            as i32)
            // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
            .wrapping_mul(params.harmonic);
        let sample_phase = params
            .phase_offset
            .wrapping_add(pll_phase.wrapping_mul(params.harmonic));

        let output = adc_samples[0]
            .iter()
            .zip(Accu::new(sample_phase, sample_frequency))
            // Convert to signed, MSB align the ADC sample.
            .map(|(&sample, phase)| {
                lockin.update(sample as i16, phase, params.time_constant)
            })
            .last()
            .unwrap();

        let conf = "frequency_discriminator";
        let output = match conf {
            // Convert from IQ to power and phase.
            "power_phase" => [(output.log2() << 24) as _, output.arg()],
            "frequency_discriminator" => [pll_frequency as _, output.arg()],
            _ => [output.0, output.1],
        };

        // Convert to DAC data.
        for i in 0..dac_samples[0].len() {
            dac_samples[0][i] = (output[0] >> 16) as u16 ^ 0x8000;
            dac_samples[1][i] = (output[1] >> 16) as u16 ^ 0x8000;
        }
    }

    #[idle(resources=[mqtt_interface, clock], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        let clock = c.resources.clock;
        loop {
            let sleep = c.resources.mqtt_interface.lock(|interface| {
                !interface.network_stack().poll(clock.current_ms())
            });

            match c
                .resources
                .mqtt_interface
                .lock(|interface| interface.update().unwrap())
            {
                miniconf::Action::Continue => {
                    if sleep {
                        cortex_m::asm::wfi();
                    }
                }
                miniconf::Action::CommitSettings => {
                    c.spawn.settings_update().unwrap()
                }
            }
        }
    }

    #[task(priority = 1, resources=[mqtt_interface, afes, parameters])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = &c.resources.mqtt_interface.settings;

        // Update AFEs
        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);

        // Update DSP parameters.
        c.resources.parameters.lock(|params| *params = settings.dsp);
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
