#![deny(warnings)]
#![no_std]
#![no_main]

use dsp::{Accu, Complex, ComplexExt, Lockin};
use generic_array::typenum::U2;
use hardware::{Adc1Input, Dac0Output, Dac1Output, AFE0, AFE1};
use stabilizer::{hardware, hardware::design_parameters};

// A constant sinusoid to send on the DAC output.
// Full-scale gives a +/- 10V amplitude waveform. Scale it down to give +/- 1V.
const ONE: i16 = (0.1 * u16::MAX as f32) as _;
const SQRT2: i16 = (ONE as f32 * 0.707) as _;
const DAC_SEQUENCE: [i16; design_parameters::SAMPLE_BUFFER_SIZE] =
    [ONE, SQRT2, 0, -SQRT2, -ONE, -SQRT2, 0, SQRT2];

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adc: Adc1Input,
        dacs: (Dac0Output, Dac1Output),

        lockin: Lockin<U2>,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        // Enable ADC/DAC events
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            lockin: Lockin::default(),
            afes: stabilizer.afes,
            adc: stabilizer.adcs.1,
            dacs: stabilizer.dacs,
        }
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of an internal-reference lockin on the ADC1 signal.
    /// The reference at f_sample/8 is output on DAC0 and the phase of the demodulated
    /// signal on DAC1.
    #[task(binds=DMA1_STR4, resources=[adc, dacs, lockin], priority=2)]
    fn process(c: process::Context) {
        let lockin = c.resources.lockin;
        let adc_samples = c.resources.adc.acquire_buffer();
        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        // Reference phase and frequency are known.
        let pll_phase = 0i32;
        let pll_frequency =
            1i32 << (32 - design_parameters::SAMPLE_BUFFER_SIZE_LOG2);

        // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
        let harmonic: i32 = -1;

        // Demodulation LO phase offset
        let phase_offset: i32 = 1 << 30;

        // Log2 lowpass time constant.
        let time_constant: u8 = 8;

        let sample_frequency = (pll_frequency as i32).wrapping_mul(harmonic);
        let sample_phase =
            phase_offset.wrapping_add(pll_phase.wrapping_mul(harmonic));

        let output: Complex<i32> = adc_samples
            .iter()
            // Zip in the LO phase.
            .zip(Accu::new(sample_phase, sample_frequency))
            // Convert to signed, MSB align the ADC sample, update the Lockin (demodulate, filter)
            .map(|(&sample, phase)| {
                let s = (sample as i16 as i32) << 16;
                lockin.update(s, phase, time_constant)
            })
            // Decimate
            .last()
            .unwrap()
            * 2; // Full scale assuming the 2f component is gone.

        // Convert to DAC data.
        for i in 0..dac_samples[0].len() {
            // DAC0 always generates a fixed sinusoidal output.
            dac_samples[0][i] = DAC_SEQUENCE[i] as u16 ^ 0x8000;
            dac_samples[1][i] = (output.arg() >> 16) as u16 ^ 0x8000;
        }
    }

    #[idle(resources=[afes])]
    fn idle(_: idle::Context) -> ! {
        loop {
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
