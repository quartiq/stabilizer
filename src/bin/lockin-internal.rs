#![deny(warnings)]
#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

use dsp::{iir_int, lockin::Lockin, Accu};
use hardware::{Adc1Input, Dac0Output, Dac1Output, AFE0, AFE1};
use stabilizer::{hardware, SAMPLE_BUFFER_SIZE, SAMPLE_BUFFER_SIZE_LOG2};

// A constant sinusoid to send on the DAC output.
// Full-scale gives a +/- 10V amplitude waveform. Scale it down to give +/- 1V.
const ONE: i16 = (0.1 * u16::MAX as f32) as _;
const SQRT2: i16 = (ONE as f32 * 0.707) as _;
const DAC_SEQUENCE: [i16; SAMPLE_BUFFER_SIZE] =
    [0, SQRT2, ONE, SQRT2, 0, -SQRT2, -ONE, -SQRT2]; // TODO: rotate by -2 samples to start with ONE

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adc1: Adc1Input,
        dacs: (Dac0Output, Dac1Output),

        lockin: Lockin,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let lockin = Lockin::new(
            iir_int::Vec5::lowpass(1e-3, 0.707, 2.), // TODO: expose
        );

        // Enable ADC/DAC events
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            lockin,
            afes: stabilizer.afes,
            adc1: stabilizer.adcs.1,
            dacs: stabilizer.dacs,
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
    ///
    /// TODO: Document
    #[task(binds=DMA1_STR4, resources=[adc1, dacs, lockin], priority=2)]
    fn process(c: process::Context) {
        let lockin = c.resources.lockin;
        let adc_samples = c.resources.adc1.acquire_buffer();
        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        // DAC0 always generates a fixed sinusoidal output.
        dac_samples[0]
            .iter_mut()
            .zip(DAC_SEQUENCE.iter())
            .for_each(|(d, s)| *d = *s as u16 ^ 0x8000);

        // Reference phase and frequency are known.
        let pll_phase = 0;
        let pll_frequency = 1i32 << (32 - SAMPLE_BUFFER_SIZE_LOG2);

        // Harmonic index of the LO: -1 to _de_modulate the fundamental
        let harmonic: i32 = -1;

        // Demodulation LO phase offset
        let phase_offset: i32 = (0.7495 * i32::MAX as f32) as i32; // TODO: adapt to sequence rotation above
        let sample_frequency = (pll_frequency as i32).wrapping_mul(harmonic);
        let sample_phase = phase_offset
            .wrapping_add((pll_phase as i32).wrapping_mul(harmonic));

        if let Some(output) = adc_samples
            .iter()
            // Zip in the LO phase.
            .zip(Accu::new(sample_phase, sample_frequency))
            // Convert to signed, MSB align the ADC sample, update the Lockin (demodulate, filter)
            .map(|(&sample, phase)| {
                lockin.update((sample as i16 as i32) << 16, phase)
            })
            // Decimate
            .last()
        {
            // Convert from IQ to power and phase.
            let _power = output.abs_sqr();
            let phase = output.arg() >> 16;

            for value in dac_samples[1].iter_mut() {
                *value = phase as u16 ^ 0x8000;
            }
        }
    }

    #[idle(resources=[afes])]
    fn idle(_: idle::Context) -> ! {
        loop {
            // TODO: Implement network interface.
            cortex_m::asm::wfi();
        }
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
