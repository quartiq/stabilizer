//! Lock-in amplifier.
//!
//! Lock-in processing is performed through a combination of the
//! following modular processing blocks: demodulation, filtering,
//! decimation and computing the magnitude and phase from a complex
//! signal. These processing blocks are mutually independent.
//!
//! # Terminology
//!
//! * _demodulation signal_ - A copy of the reference signal that is
//! optionally frequency scaled and phase shifted. This is a complex
//! signal. The demodulation signals are used to demodulate the ADC
//! sampled signal.
//! * _internal clock_ - A fast internal clock used to increment a
//! counter for determining the 0-phase points of a reference signal.
//! * _reference signal_ - A constant-frequency signal used to derive
//! the demodulation signal.
//! * _timestamp_ - Timestamps record the timing of the reference
//! signal's 0-phase points. For instance, if a reference signal is
//! provided externally, a fast internal clock increments a
//! counter. When the external reference reaches the 0-phase point
//! (e.g., a positive edge), the value of the counter is recorded as a
//! timestamp. These timestamps are used to determine the frequency
//! and phase of the reference signal.
//!
//! # Usage
//!
//! The first step is to initialize a `Lockin` instance with
//! `Lockin::new()`. This provides the lock-in algorithms with
//! necessary information about the demodulation and filtering steps,
//! such as whether to demodulate with a harmonic of the reference
//! signal and the IIR biquad filter to use. There are then 4
//! different processing steps that can be used:
//!
//! * `demodulate` - Computes the phase of the demodulation signal
//! corresponding to each ADC sample, uses this phase to compute the
//! demodulation signal, and multiplies this demodulation signal by
//! the ADC-sampled signal. This is a method of `Lockin` since it
//! requires information about how to modify the reference signal for
//! demodulation.
//! * `filter` - Performs IIR biquad filtering of a complex
//! signals. This is commonly performed on the signal provided by the
//! demodulation step, but can be performed at any other point in the
//! processing chain or omitted entirely. `filter` is a method of
//! `Lockin` since it must hold onto the filter configuration and
//! state.
//! * `decimate` - This decimates a signal to reduce the load on the
//! DAC output. It does not require any state information and is
//! therefore a normal function.
//! * `magnitude_phase` - Computes the magnitude and phase of the
//! component of the ADC-sampled signal whose frequency is equal to
//! the demodulation frequency. This does not require any state
//! information and is therefore a normal function.

use super::complex::Complex;
use super::iir::{IIRState, IIR};
use core::f32::consts::PI;

/// The number of ADC samples in one batch.
pub const ADC_SAMPLE_BUFFER_SIZE: usize = 16;
/// The number of outputs sent to the DAC for each ADC batch.
pub const DECIMATED_BUFFER_SIZE: usize = 1;

/// Treat the 2-element array as a FIFO. This allows new elements to
/// be pushed into the array, existing elements to shift back in the
/// array, and the last element to fall off the array.
trait Fifo2<T> {
    fn push(&mut self, new_element: Option<T>);
}

impl<T: Copy> Fifo2<T> for [Option<T>; 2] {
    /// Push a new element into the array. The existing elements move
    /// backward in the array by one location, and the current last
    /// element is discarded.
    ///
    /// # Arguments
    ///
    /// * `new_element` - New element pushed into the front of the
    /// array.
    fn push(&mut self, new_element: Option<T>) {
        // For array sizes greater than 2 it would be preferable to
        // use a rotating index to avoid unnecessary data
        // copying. However, this would somewhat complicate the use of
        // iterators and for 2 elements, shifting is inexpensive.
        self[1] = self[0];
        self[0] = new_element;
    }
}

/// Performs lock-in amplifier processing of a signal.
pub struct Lockin {
    phase_offset: f32,
    sample_period: u32,
    harmonic: u32,
    timestamps: [Option<i32>; 2],
    iir: IIR,
    iirstate: [IIRState; 2],
}

impl Lockin {
    /// Initialize a new `Lockin` instance.
    ///
    /// # Arguments
    ///
    /// * `phase_offset` - Phase offset (in radians) applied to the
    /// demodulation signal.
    /// * `sample_period` - ADC sampling period in terms of the
    /// internal clock period.
    /// * `harmonic` - Integer scaling factor used to adjust the
    /// demodulation frequency. E.g., 2 would demodulate with the
    /// first harmonic.
    /// * `iir` - IIR biquad filter.
    ///
    /// # Returns
    ///
    /// New `Lockin` instance.
    pub fn new(
        phase_offset: f32,
        sample_period: u32,
        harmonic: u32,
        iir: IIR,
    ) -> Self {
        Lockin {
            phase_offset: phase_offset,
            sample_period: sample_period,
            harmonic: harmonic,
            timestamps: [None, None],
            iir: iir,
            iirstate: [[0.; 5]; 2],
        }
    }

    /// Demodulate an input signal with the complex reference signal.
    ///
    /// # Arguments
    ///
    /// * `adc_samples` - One batch of ADC samples.
    /// * `timestamps` - Counter values corresponding to the edges of
    /// an external reference signal. The counter is incremented by a
    /// fast internal clock.
    ///
    /// # Returns
    ///
    /// The demodulated complex signal as a `Result`. When there are
    /// an insufficient number of timestamps to perform processing,
    /// `Err` is returned.
    ///
    /// # Assumptions
    ///
    /// `demodulate` expects that the timestamp counter value is equal
    /// to 0 when the ADC samples its first input in a batch. This can
    /// be achieved by configuring the timestamp counter to overflow
    /// at the end of the ADC batch sampling period.
    pub fn demodulate(
        &mut self,
        adc_samples: &[i16],
        timestamps: &[u16],
    ) -> Result<[Complex; ADC_SAMPLE_BUFFER_SIZE], &str> {
        // update old timestamps for new ADC batch
        let sample_period = self.sample_period as i32;
        self.timestamps.iter_mut().for_each(|t| match *t {
            Some(timestamp) => {
                // Existing timestamps have aged by one ADC batch
                // period since the last ADC batch.
                *t = Some(
                    timestamp - ADC_SAMPLE_BUFFER_SIZE as i32 * sample_period,
                );
            }
            None => (),
        });

        // record new timestamps
        timestamps
            .iter()
            .take(timestamps.len())
            .rev()
            .take(2)
            .rev()
            .for_each(|t| self.timestamps.push(Some(*t as i32)));

        // return prematurely if there aren't enough timestamps for
        // processing
        if self.timestamps.iter().filter(|t| t.is_some()).count() < 2 {
            return Err("insufficient timestamps");
        }

        // compute ADC sample phases, sines/cosines and demodulate
        let reference_period =
            self.timestamps[0].unwrap() - self.timestamps[1].unwrap();
        let mut signal = [Complex::new(0., 0.); ADC_SAMPLE_BUFFER_SIZE];
        signal
            .iter_mut()
            .zip(adc_samples.iter())
            .enumerate()
            .for_each(|(n, (s, sample))| {
                let integer_phase: i32 = (n as i32 * self.sample_period as i32
                    - self.timestamps[0].unwrap())
                    * self.harmonic as i32;
                let phase = self.phase_offset
                    + 2. * PI * integer_phase as f32 / reference_period as f32;
                let (sine, cosine) = libm::sincosf(phase);
                let sample = *sample as f32;
                s.re = sine * sample;
                s.im = cosine * sample;
            });

        Ok(signal)
    }

    /// Filter the complex signal using the supplied biquad IIR. The
    /// signal array is modified in place.
    ///
    /// # Arguments
    ///
    /// * `signal` - Complex signal to filter.
    pub fn filter(&mut self, signal: &mut [Complex]) {
        signal.iter_mut().for_each(|s| {
            s.re = self.iir.update(&mut self.iirstate[0], s.re);
            s.im = self.iir.update(&mut self.iirstate[1], s.im);
        });
    }
}

/// Decimate the complex signal to `DECIMATED_BUFFER_SIZE`. The ratio
/// of `ADC_SAMPLE_BUFFER_SIZE` to `DECIMATED_BUFFER_SIZE` must be a
/// power of 2.
///
/// # Arguments
///
/// * `signal` - Complex signal to decimate.
///
/// # Returns
///
/// The decimated signal.
pub fn decimate(
    signal: [Complex; ADC_SAMPLE_BUFFER_SIZE],
) -> [Complex; DECIMATED_BUFFER_SIZE] {
    let n_k = ADC_SAMPLE_BUFFER_SIZE / DECIMATED_BUFFER_SIZE;
    debug_assert!(
        ADC_SAMPLE_BUFFER_SIZE == DECIMATED_BUFFER_SIZE || n_k % 2 == 0
    );

    let mut signal_decimated = [Complex::new(0., 0.); DECIMATED_BUFFER_SIZE];

    signal_decimated
        .iter_mut()
        .zip(signal.iter().step_by(n_k))
        .for_each(|(s_d, s)| {
            s_d.re = s.re;
            s_d.im = s.im;
        });

    signal_decimated
}

/// Compute the magnitude and phase from the complex signal. The
/// signal array is modified in place.
///
/// # Arguments
///
/// * `signal` - Complex signal to decimate.
pub fn magnitude_phase(signal: &mut [Complex]) {
    signal.iter_mut().for_each(|s| {
        let new_i = s.abs();
        let new_q = s.arg();
        s.re = new_i;
        s.im = new_q;
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn f32_is_close(a: f32, b: f32) -> bool {
        (a - b).abs() <= a.abs().max(b.abs()) * f32::EPSILON
    }

    fn complex_is_close(a: Complex, b: Complex) -> bool {
        f32_is_close(a.re, b.re) && f32_is_close(a.im, b.im)
    }

    fn complex_array_is_close(a: &[Complex], b: &[Complex]) -> bool {
        let mut result: bool = true;
        a.iter().zip(b.iter()).for_each(|(i, j)| {
            result &= complex_is_close(*i, *j);
        });
        result
    }

    fn within_tolerance(
        a: f32,
        b: f32,
        relative_tolerance: f32,
        fixed_tolerance: f32,
    ) -> bool {
        (a - b).abs()
            <= a.abs().max(b.abs()) * relative_tolerance + fixed_tolerance
    }

    fn complex_within_tolerance(
        a: Complex,
        b: Complex,
        relative_tolerance: f32,
        fixed_tolerance: f32,
    ) -> bool {
        within_tolerance(a.re, b.re, relative_tolerance, fixed_tolerance)
            && within_tolerance(a.im, b.im, relative_tolerance, fixed_tolerance)
    }

    fn complex_array_within_tolerance(
        a: &[Complex],
        b: &[Complex],
        relative_tolerance: f32,
        fixed_tolerance: f32,
    ) -> bool {
        let mut result: bool = true;
        a.iter().zip(b.iter()).for_each(|(i, j)| {
            result &= complex_within_tolerance(
                *i,
                *j,
                relative_tolerance,
                fixed_tolerance,
            );
        });
        result
    }

    #[test]
    fn array_push() {
        let mut arr: [Option<u32>; 2] = [None, None];
        arr.push(Some(1));
        assert_eq!(arr, [Some(1), None]);
        arr.push(Some(2));
        assert_eq!(arr, [Some(2), Some(1)]);
        arr.push(Some(10));
        assert_eq!(arr, [Some(10), Some(2)]);
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_1() {
        let mut signal: [Complex; 1] = [Complex::new(1., 1.)];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(2_f32.sqrt(), PI / 4.)]
        ));

        signal = [Complex::new(3_f32.sqrt() / 2., 1. / 2.)];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(1., PI / 6.)]
        ));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_2() {
        let mut signal = [Complex::new(-1., 1.)];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(2_f32.sqrt(), 3. * PI / 4.)]
        ));

        signal = [Complex::new(-1. / 2., 3_f32.sqrt() / 2.)];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(1_f32, 2. * PI / 3.)]
        ));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_3() {
        let mut signal = [Complex::new(-1. / 2_f32.sqrt(), -1. / 2_f32.sqrt())];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(1_f32.sqrt(), -3. * PI / 4.)]
        ));

        signal = [Complex::new(-1. / 2., -2_f32.sqrt())];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new((3. / 2.) as f32, -1.91063323625 as f32)]
        ));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_4() {
        let mut signal = [Complex::new(1. / 2_f32.sqrt(), -1. / 2_f32.sqrt())];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(1_f32.sqrt(), -1. * PI / 4.)]
        ));

        signal = [Complex::new(3_f32.sqrt() / 2., -1. / 2.)];
        magnitude_phase(&mut signal);
        assert!(complex_array_is_close(
            &signal,
            &[Complex::new(1_f32, -PI / 6.)]
        ));
    }

    #[test]
    fn decimate_sample_16_decimated_1() {
        let signal: [Complex; ADC_SAMPLE_BUFFER_SIZE] = [
            Complex::new(0.0, 1.6),
            Complex::new(0.1, 1.7),
            Complex::new(0.2, 1.8),
            Complex::new(0.3, 1.9),
            Complex::new(0.4, 2.0),
            Complex::new(0.5, 2.1),
            Complex::new(0.6, 2.2),
            Complex::new(0.7, 2.3),
            Complex::new(0.8, 2.4),
            Complex::new(0.9, 2.5),
            Complex::new(1.0, 2.6),
            Complex::new(1.1, 2.7),
            Complex::new(1.2, 2.8),
            Complex::new(1.3, 2.9),
            Complex::new(1.4, 3.0),
            Complex::new(1.5, 3.1),
        ];
        assert_eq!(decimate(signal), [Complex::new(0.0, 1.6)]);
    }

    #[test]
    fn lockin_demodulate_valid_0() {
        let mut lockin = Lockin::new(
            0.,
            200,
            1,
            IIR {
                ba: [0_f32; 5],
                y_offset: 0.,
                y_min: -(1 << 15) as f32,
                y_max: (1 << 15) as f32 - 1.,
            },
        );
        assert_eq!(
            lockin.demodulate(&[0; ADC_SAMPLE_BUFFER_SIZE], &[]),
            Err("insufficient timestamps")
        );
    }

    #[test]
    fn lockin_demodulate_valid_1() {
        let mut lockin = Lockin::new(
            0.,
            200,
            1,
            IIR {
                ba: [0_f32; 5],
                y_offset: 0.,
                y_min: -(1 << 15) as f32,
                y_max: (1 << 15) as f32 - 1.,
            },
        );
        assert_eq!(
            lockin.demodulate(&[0; ADC_SAMPLE_BUFFER_SIZE], &[0],),
            Err("insufficient timestamps")
        );
    }

    #[test]
    fn lockin_demodulate_valid_2() {
        let adc_period: u32 = 200;
        let mut lockin = Lockin::new(
            0.,
            adc_period,
            1,
            IIR {
                ba: [0_f32; 5],
                y_offset: 0.,
                y_min: -(1 << 15) as f32,
                y_max: (1 << 15) as f32 - 1.,
            },
        );
        let adc_samples: [i16; ADC_SAMPLE_BUFFER_SIZE] =
            [-8, 7, -7, 6, -6, 5, -5, 4, -4, 3, -3, 2, -2, -1, 1, 0];
        let reference_period: u16 = 2800;
        let initial_phase_integer: u16 = 200;
        let timestamps: &[u16] = &[
            initial_phase_integer,
            initial_phase_integer + reference_period,
        ];
        let initial_phase: f32 =
            -(initial_phase_integer as f32) / reference_period as f32 * 2. * PI;
        let phase_increment: f32 =
            adc_period as f32 / reference_period as f32 * 2. * PI;
        let mut signal = [Complex::new(0., 0.); ADC_SAMPLE_BUFFER_SIZE];
        for (n, s) in signal.iter_mut().enumerate() {
            let adc_phase = initial_phase + n as f32 * phase_increment;
            let sine = adc_phase.sin();
            let cosine = adc_phase.cos();
            s.re = sine * adc_samples[n] as f32;
            s.im = cosine * adc_samples[n] as f32;
        }
        let result = lockin.demodulate(&adc_samples, timestamps).unwrap();
        assert!(
            complex_array_within_tolerance(&result, &signal, 0., 1e-5),
            "\nsignal computed: {:?},\nsignal expected: {:?}",
            result,
            signal
        );
    }
}
