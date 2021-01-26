use super::{cossin, iir_int, Complex};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Lockin {
    iir: iir_int::IIR,
    iir_state: [iir_int::IIRState; 2],
}

impl Lockin {
    pub fn new(ba: &iir_int::IIRState) -> Self {
        let mut iir = iir_int::IIR::default();
        iir.ba.0.copy_from_slice(&ba.0);
        Lockin {
            iir,
            iir_state: [iir_int::IIRState::default(); 2],
        }
    }

    pub fn update(&mut self, signal: i32, phase: i32) -> Complex<i32> {
        // Get the LO signal for demodulation.
        let m = cossin(phase);

        // Mix with the LO signal, filter with the IIR lowpass,
        // return IQ (in-phase and quadrature) data.
        // Note: 32x32 -> 64 bit multiplications are pretty much free.
        Complex(
            self.iir.update(
                &mut self.iir_state[0],
                ((signal as i64 * m.0 as i64) >> 32) as _,
            ),
            self.iir.update(
                &mut self.iir_state[1],
                ((signal as i64 * m.1 as i64) >> 32) as _,
            ),
        )
    }
}

#[cfg(test)]
mod test {
    use crate::{
        atan2,
        iir_int::IIRState,
        lockin::Lockin,
        rpll::RPLL,
        testing::{isclose, max_error},
        Complex,
    };

    use std::f64::consts::PI;
    use std::vec::Vec;

    /// ADC full scale in machine units (16 bit signed).
    const ADC_SCALE: f64 = ((1 << 15) - 1) as _;

    struct PllLockin {
        harmonic: i32,
        phase: i32,
        lockin: Lockin,
    }

    impl PllLockin {
        pub fn new(harmonic: i32, phase: i32, iir: &IIRState) -> Self {
            PllLockin {
                harmonic,
                phase,
                lockin: Lockin::new(iir),
            }
        }

        pub fn update(
            &mut self,
            input: Vec<i16>,
            phase: i32,
            frequency: i32,
        ) -> Complex<i32> {
            let sample_frequency = frequency.wrapping_mul(self.harmonic);
            let mut sample_phase =
                self.phase.wrapping_add(phase.wrapping_mul(self.harmonic));
            input
                .iter()
                .map(|&s| {
                    let input = (s as i32) << 16;
                    let signal =
                        self.lockin.update(input, sample_phase.wrapping_neg());
                    sample_phase = sample_phase.wrapping_add(sample_frequency);
                    signal
                })
                .last()
                .unwrap_or(Complex::default())
        }
    }

    /// Single-frequency sinusoid.
    #[derive(Copy, Clone)]
    struct Tone {
        // Frequency (in Hz).
        frequency: f64,
        // Phase offset (in radians).
        phase: f64,
        // Amplitude in dBFS (decibels relative to full-scale).
        // A 16-bit ADC has a minimum dBFS for each sample of -90.
        amplitude_dbfs: f64,
    }

    /// Convert dBFS to a linear ratio.
    fn linear(dbfs: f64) -> f64 {
        10f64.powf(dbfs / 20.)
    }

    impl Tone {
        fn eval(&self, time: f64) -> f64 {
            linear(self.amplitude_dbfs)
                * (self.phase + self.frequency * time).cos()
        }
    }

    /// Generate a full batch of samples with size `sample_buffer_size` starting at `time_offset`.
    fn sample_tones(
        tones: &Vec<Tone>,
        time_offset: f64,
        sample_buffer_size: u32,
    ) -> Vec<i16> {
        (0..sample_buffer_size)
            .map(|i| {
                let time = 2. * PI * (time_offset + i as f64);
                let x: f64 = tones.iter().map(|t| t.eval(time)).sum();
                assert!(-1. < x && x < 1.);
                (x * ADC_SCALE) as i16
            })
            .collect()
    }

    /// Total maximum noise amplitude of the input signal after 2nd order lowpass filter.
    /// Constructive interference is assumed.
    ///
    /// # Args
    /// * `tones` - Noise sources at the ADC input.
    /// * `frequency` - Frequency of the signal of interest.
    /// * `corner` - Low-pass filter 3dB corner cutoff frequency.
    ///
    /// # Returns
    /// Upper bound of the total amplitude of all noise sources in linear units full scale.
    fn sampled_noise_amplitude(
        tones: &Vec<Tone>,
        frequency: f64,
        corner: f64,
    ) -> f64 {
        tones
            .iter()
            .map(|t| {
                let df = (t.frequency - frequency) / corner;
                // Assuming a 2nd order lowpass filter: 40dB/decade.
                linear(t.amplitude_dbfs - 40. * df.abs().max(1.).log10())
            })
            .sum::<f64>()
            .max(1. / ADC_SCALE / 2.) // 1/2 LSB from quantization
    }
}
