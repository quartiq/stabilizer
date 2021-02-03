#![deny(warnings)]
#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

use stm32h7xx_hal as hal;

use rtic::cyccnt::{Instant, U32Ext};

use stabilizer::{hardware, ADC_SAMPLE_TICKS_LOG2, SAMPLE_BUFFER_SIZE_LOG2};

use miniconf::{
    embedded_nal::{IpAddr, Ipv4Addr},
    MqttInterface, StringSet,
};
use serde::Deserialize;

use dsp::{iir, iir_int, lockin::Lockin, rpll::RPLL, Accu};
use hardware::{
    Adc0Input, Adc1Input, Dac0Output, Dac1Output, InputStamper, AFE0, AFE1,
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

#[derive(Debug, Deserialize, StringSet)]
pub struct Settings {
    iir: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
}

impl Settings {
    pub fn new() -> Self {
        Self {
            iir: [[iir::IIR::default(); IIR_CASCADE_LENGTH]; 2],
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        mqtt_interface: MqttInterface<Settings, hardware::NetworkStack>,

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

        let broker = IpAddr::V4(Ipv4Addr::new(10, 0, 0, 2));
        let mqtt_interface = MqttInterface::new(
            stabilizer.net.stack,
            "stabilizer/lockin",
            broker,
            Settings::new(),
        )
        .unwrap();

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

        init::LateResources {
            mqtt_interface,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
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

    #[idle(resources=[mqtt_interface], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
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

            let sleep = c
                .resources
                .mqtt_interface
                .lock(|interface| !interface.network_stack().poll(time));

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

    #[task(priority = 1, resources=[mqtt_interface, afes, iir_ch])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = &c.resources.mqtt_interface.settings;
        c.resources.iir_ch.lock(|iir| *iir = settings.iir);
        // TODO: Update AFEs
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
