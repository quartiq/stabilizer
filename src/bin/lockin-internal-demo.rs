#![deny(warnings)]
#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

// A constant sinusoid to send on the DAC output.
const DAC_SEQUENCE: [f32; 8] =
    [0.0, 0.707, 1.0, 0.707, 0.0, -0.707, -1.0, -0.707];

use dsp::{iir, iir_int, lockin::Lockin};
use hardware::{Adc1Input, Dac0Output, Dac1Output, AFE0, AFE1};
use stabilizer::{hardware, ADC_SAMPLE_TICKS};

const SCALE: f32 = ((1 << 15) - 1) as f32;

const PHASE_SCALING: f32 = 1e12;

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adc1: Adc1Input,
        dacs: (Dac0Output, Dac1Output),

        #[init([0.; 5])]
        iir_state: iir::IIRState,

        #[init(iir::IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE })]
        iir: iir::IIR,

        lockin: Lockin,
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        // The desired corner frequency is always
        let desired_corner_frequency = 10e3;
        let gain = 1000.0;

        // Calculate the IIR corner freuqency parameter as a function of the sample rate.
        let corner_frequency = {
            let sample_rate = 1.0 / (10e-9 * ADC_SAMPLE_TICKS as f32);
            desired_corner_frequency / sample_rate
        };

        let lockin = Lockin::new(
            &iir_int::IIRState::lowpass(corner_frequency, 0.707, gain), // TODO: expose
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
    #[task(binds=DMA1_STR4, resources=[adc1, dacs, iir_state, iir, lockin], priority=2)]
    fn process(mut c: process::Context) {
        let adc_samples = c.resources.adc1.acquire_buffer();
        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        // DAC0 always generates a fixed sinusoidal output.
        for (i, value) in DAC_SEQUENCE.iter().enumerate() {
            // Full-scale gives a +/- 12V amplitude waveform. Scale it down to give +/- 100mV.
            let y = value * i16::MAX as f32 / 120.0;
            // Note(unsafe): The DAC_SEQUENCE values are guaranteed to be normalized.
            let y = unsafe { y.to_int_unchecked::<i16>() };

            // Convert to DAC code
            dac_samples[0][i] = y as u16 ^ 0x8000;
        }

        let pll_phase = 0i32;
        let pll_frequency = 1i32 << (32 - 3); // 1/8 of the sample rate

        // Harmonic index of the LO: -1 to _de_modulate the fundamental
        let harmonic: i32 = -1;

        // Demodulation LO phase offset
        let phase_offset: i32 = 0;
        let sample_frequency = (pll_frequency as i32).wrapping_mul(harmonic);
        let mut sample_phase = phase_offset
            .wrapping_add((pll_phase as i32).wrapping_mul(harmonic));

        let mut phase = 0f32;

        for sample in adc_samples.iter() {
            // Convert to signed, MSB align the ADC sample.
            let input = (*sample as i16 as i32) << 16;

            // Obtain demodulated, filtered IQ sample.
            let output = c.resources.lockin.update(input, sample_phase);

            // Advance the sample phase.
            sample_phase = sample_phase.wrapping_add(sample_frequency);

            // Convert from IQ to phase.
            phase = output.phase() as _;
        }

        // Filter phase through an IIR.
        phase = c.resources.iir.update(&mut c.resources.iir_state, phase)
            * PHASE_SCALING;

        for value in dac_samples[1].iter_mut() {
            *value = phase as u16 ^ 0x8000
        }
    }

    #[idle(resources=[iir_state, iir, afes])]
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
