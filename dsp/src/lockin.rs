//! Lock-in amplifier.
//!
//! Lock-in processing is performed through a combination of the
//! following modular processing blocks: demodulation, filtering,
//! decimation and computing the magnitude and phase from the in-phase
//! and quadrature signals. These processing blocks are mutually
//! independent.
//!
//! # Terminology
//!
//! * _demodulation signal_ - A copy of the reference signal that is
//! optionally frequency scaled and phase shifted. There are two
//! copies of this signal. The first copy is in-phase with the
//! reference signal (before any optional phase shifting). The second
//! is 90 degrees out of phase (in quadrature) with the first
//! copy. The demodulation signals are used to demodulate the ADC
//! sampled signal.
//! * _in-phase_ and _quadrature_ - These terms are used to delineate
//! between the two components of the demodulation signal and the
//! resulting two signals at any step downstream of the demodulation
//! step. The in-phase signal is in-phase with the reference signal
//! prior to any phase shifts. The quadrature signal is 90 degrees out
//! of phase with the in-phase signal.
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
//! in-phase and quadrature demodulation signals, and multiplies these
//! demodulation signals by the ADC-sampled signal. This is a method
//! of `Lockin` since it requires information about how to modify the
//! reference signal for demodulation.
//! * `filter` - Performs IIR biquad filtering of in-phase and
//! quadrature signals. This is commonly performed on the in-phase and
//! quadrature components provided by the demodulation step, but can
//! be performed at any other point in the processing chain or omitted
//! entirely. `filter` is a method of `Lockin` since it must hold onto
//! the filter configuration and state.
//! * `decimate` - This decimates the in-phase and quadrature signals
//! to reduce the load on the DAC output. It does not require any
//! state information and is therefore a normal function.
//! * `magnitude_phase` - Computes the magnitude and phase of the
//! component of the ADC-sampled signal whose frequency is equal to
//! the demodulation frequency. This does not require any state
//! information and is therefore a normal function.

use super::iir::{IIRState, IIR};
use core::f32::consts::PI;

/// The number of ADC samples in one batch.
pub const ADC_SAMPLE_BUFFER_SIZE: usize = 16;
/// The maximum number of timestamps in the period for one ADC
/// batch. Each timestamp corresponds to the time of an external
/// reference clock edge.
pub const TIMESTAMP_BUFFER_SIZE: usize = ADC_SAMPLE_BUFFER_SIZE / 2;
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
    iir: [IIR; 2],
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
    /// * `iir` - IIR biquad filter. Two identical copies of this IIR
    /// filter are used: one for the in-phase signal and the other for
    /// the quadrature signal.
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
            iir: [iir, iir],
            iirstate: [[0.; 5]; 2],
        }
    }

    /// Demodulate an input signal with in-phase and quadrature
    /// reference signals.
    ///
    /// # Arguments
    ///
    /// * `adc_samples` - One batch of ADC samples.
    /// * `timestamps` - Counter values corresponding to the edges of
    /// an external reference signal. The counter is incremented by a
    /// fast internal clock.
    /// * `valid_timestamps` - The number of valid timestamps in
    /// `timestamps`. Only `&timestamps[..valid_timestamps]` are used;
    /// every other value in the `timestamps` array is ignored.
    ///
    /// # Returns
    ///
    /// The demodulated in-phase and quadrature signals as an
    /// `Option`. When there are an insufficient number of timestamps
    /// to perform processing, `None` is returned.
    ///
    /// # Assumptions
    ///
    /// `demodulate` expects that the timestamp counter value is equal
    /// to 0 when the ADC samples its first input in a batch. This can
    /// be achieved by configuring the timestamp counter to overflow
    /// at the end of the ADC batch sampling period.
    pub fn demodulate(
        &mut self,
        adc_samples: [i16; ADC_SAMPLE_BUFFER_SIZE],
        timestamps: [u16; TIMESTAMP_BUFFER_SIZE],
        valid_timestamps: u16,
    ) -> Option<([f32; ADC_SAMPLE_BUFFER_SIZE], [f32; ADC_SAMPLE_BUFFER_SIZE])>
    {
        // update old timestamps for new ADC batch
        let sample_period = self.sample_period as i32;
        self.timestamps.iter_mut().for_each(|t| match *t {
            Some(i) => {
                *t = Some(i - ADC_SAMPLE_BUFFER_SIZE as i32 * sample_period);
            }
            None => (),
        });

        // record new timestamps
        timestamps
            .iter()
            .take(valid_timestamps as usize)
            .rev()
            .take(2)
            .rev()
            .for_each(|t| self.timestamps.push(Some(*t as i32)));

        // return prematurely if there aren't enough timestamps for
        // processing
        if self.timestamps.iter().filter(|t| t.is_some()).count() < 2 {
            return None;
        }

        // compute ADC sample phases, sines/cosines and demodulate
        let reference_period =
            self.timestamps[0].unwrap() - self.timestamps[1].unwrap();
        let mut in_phase = [0f32; ADC_SAMPLE_BUFFER_SIZE];
        let mut quadrature = [0f32; ADC_SAMPLE_BUFFER_SIZE];
        in_phase
            .iter_mut()
            .zip(quadrature.iter_mut())
            .zip(adc_samples.iter())
            .enumerate()
            .for_each(|(n, ((i, q), sample))| {
                let integer_phase: i32 = (n as i32 * self.sample_period as i32
                    - self.timestamps[0].unwrap())
                    * self.harmonic as i32;
                let phase = self.phase_offset
                    + 2. * PI * integer_phase as f32 / reference_period as f32;
                let (sine, cosine) = libm::sincosf(phase);
                let sample = *sample as f32;
                *i = sine * sample;
                *q = cosine * sample;
            });

        Some((in_phase, quadrature))
    }

    /// Filter the in-phase and quadrature signals using the supplied
    /// biquad IIR. The signal arrays are modified in place.
    ///
    /// # Arguments
    ///
    /// * `in_phase` - In-phase signal.
    /// * `quadrature` - Quadrature signal.
    pub fn filter(&mut self, in_phase: &mut [f32], quadrature: &mut [f32]) {
        in_phase
            .iter_mut()
            .zip(quadrature.iter_mut())
            .for_each(|(i, q)| {
                *i = self.iir[0].update(&mut self.iirstate[0], *i);
                *q = self.iir[1].update(&mut self.iirstate[1], *q);
            });
    }
}

/// Decimate the in-phase and quadrature signals to
/// `DECIMATED_BUFFER_SIZE`. The ratio of `ADC_SAMPLE_BUFFER_SIZE` to
/// `DECIMATED_BUFFER_SIZE` must be a power of 2.
///
/// # Arguments
///
/// * `in_phase` - In-phase signal.
/// * `quadrature` - Quadrature signal.
///
/// # Returns
///
/// The decimated in-phase and quadrature signals.
pub fn decimate(
    in_phase: [f32; ADC_SAMPLE_BUFFER_SIZE],
    quadrature: [f32; ADC_SAMPLE_BUFFER_SIZE],
) -> ([f32; DECIMATED_BUFFER_SIZE], [f32; DECIMATED_BUFFER_SIZE]) {
    let n_k = ADC_SAMPLE_BUFFER_SIZE / DECIMATED_BUFFER_SIZE;
    debug_assert!(
        ADC_SAMPLE_BUFFER_SIZE == DECIMATED_BUFFER_SIZE || n_k % 2 == 0
    );

    let mut in_phase_decimated = [0f32; DECIMATED_BUFFER_SIZE];
    let mut quadrature_decimated = [0f32; DECIMATED_BUFFER_SIZE];

    in_phase_decimated
        .iter_mut()
        .zip(quadrature_decimated.iter_mut())
        .zip(in_phase.iter().step_by(n_k))
        .zip(quadrature.iter().step_by(n_k))
        .for_each(|(((i_decimated, q_decimated), i_original), q_original)| {
            *i_decimated = *i_original;
            *q_decimated = *q_original;
        });

    (in_phase_decimated, quadrature_decimated)
}

/// Compute the magnitude and phase from the in-phase and quadrature
/// signals. The in-phase and quadrature arrays are modified in place.
///
/// # Arguments
///
/// * `in_phase` - In-phase signal.
/// * `quadrature` - Quadrature signal.
pub fn magnitude_phase(in_phase: &mut [f32], quadrature: &mut [f32]) {
    in_phase
        .iter_mut()
        .zip(quadrature.iter_mut())
        .for_each(|(i, q)| {
            let new_i = libm::sqrtf([*i, *q].iter().map(|i| i * i).sum());
            let new_q = libm::atan2f(*q, *i);
            *i = new_i;
            *q = new_q;
        });
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;

    fn f32_is_close(a: f32, b: f32) -> bool {
        (a - b).abs() <= a.abs().max(b.abs()) * f32::EPSILON
    }

    fn f32_array_is_close(a: &[f32], b: &[f32]) -> bool {
        let mut result: bool = true;
        a.iter().zip(b.iter()).for_each(|(i, j)| {
            result &= f32_is_close(*i, *j);
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

    fn array_within_tolerance(
        a: &[f32],
        b: &[f32],
        relative_tolerance: f32,
        fixed_tolerance: f32,
    ) -> bool {
        let mut result: bool = true;
        a.iter().zip(b.iter()).for_each(|(i, j)| {
            result &=
                within_tolerance(*i, *j, relative_tolerance, fixed_tolerance);
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
        let mut in_phase: [f32; 1] = [1.];
        let mut quadrature: [f32; 1] = [1.];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[2_f32.sqrt()]));
        assert!(f32_array_is_close(&quadrature, &[PI / 4.]));

        in_phase = [3_f32.sqrt() / 2.];
        quadrature = [1. / 2.];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[1_f32]));
        assert!(f32_array_is_close(&quadrature, &[PI / 6.]));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_2() {
        let mut in_phase: [f32; 1] = [-1.];
        let mut quadrature: [f32; 1] = [1.];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[2_f32.sqrt()]));
        assert!(f32_array_is_close(&quadrature, &[3. * PI / 4.]));

        in_phase = [-1. / 2.];
        quadrature = [3_f32.sqrt() / 2.];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[1_f32]));
        assert!(f32_array_is_close(&quadrature, &[2. * PI / 3.]));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_3() {
        let mut in_phase: [f32; 1] = [-1. / 2_f32.sqrt()];
        let mut quadrature: [f32; 1] = [-1. / 2_f32.sqrt()];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[1_f32.sqrt()]));
        assert!(f32_array_is_close(&quadrature, &[-3. * PI / 4.]));

        in_phase = [-1. / 2.];
        quadrature = [-2_f32.sqrt()];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[(3. / 2.) as f32]));
        assert!(f32_array_is_close(&quadrature, &[-1.91063323625 as f32]));
    }

    #[test]
    fn magnitude_phase_length_1_quadrant_4() {
        let mut in_phase: [f32; 1] = [1. / 2_f32.sqrt()];
        let mut quadrature: [f32; 1] = [-1. / 2_f32.sqrt()];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[1_f32.sqrt()]));
        assert!(f32_array_is_close(&quadrature, &[-1. * PI / 4.]));

        in_phase = [3_f32.sqrt() / 2.];
        quadrature = [-1. / 2.];
        magnitude_phase(&mut in_phase, &mut quadrature);
        assert!(f32_array_is_close(&in_phase, &[1_f32]));
        assert!(f32_array_is_close(&quadrature, &[-PI / 6.]));
    }

    #[test]
    fn decimate_sample_16_decimated_1() {
        let in_phase: [f32; ADC_SAMPLE_BUFFER_SIZE] = [
            0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2,
            1.3, 1.4, 1.5,
        ];
        let quadrature: [f32; ADC_SAMPLE_BUFFER_SIZE] = [
            1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8,
            2.9, 3.0, 3.1,
        ];
        assert_eq!(decimate(in_phase, quadrature), ([0.0], [1.6]));
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
            lockin.demodulate(
                [0; ADC_SAMPLE_BUFFER_SIZE],
                [0; TIMESTAMP_BUFFER_SIZE],
                0
            ),
            None
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
            lockin.demodulate(
                [0; ADC_SAMPLE_BUFFER_SIZE],
                [0; TIMESTAMP_BUFFER_SIZE],
                1
            ),
            None
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
        let timestamps: [u16; TIMESTAMP_BUFFER_SIZE] = [
            initial_phase_integer,
            initial_phase_integer + reference_period,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let initial_phase: f32 =
            -(initial_phase_integer as f32) / reference_period as f32 * 2. * PI;
        let phase_increment: f32 =
            adc_period as f32 / reference_period as f32 * 2. * PI;
        let mut in_phase: [f32; ADC_SAMPLE_BUFFER_SIZE] =
            [0.; ADC_SAMPLE_BUFFER_SIZE];
        let mut quadrature: [f32; ADC_SAMPLE_BUFFER_SIZE] =
            [0.; ADC_SAMPLE_BUFFER_SIZE];
        for (n, (i, q)) in
            in_phase.iter_mut().zip(quadrature.iter_mut()).enumerate()
        {
            let adc_phase = initial_phase + n as f32 * phase_increment;
            let sine = adc_phase.sin();
            let cosine = adc_phase.cos();
            *i = sine * adc_samples[n] as f32;
            *q = cosine * adc_samples[n] as f32;
        }
        let (result_in_phase, result_quadrature) =
            lockin.demodulate(adc_samples, timestamps, 2).unwrap();
        assert!(
            array_within_tolerance(&result_in_phase, &in_phase, 0., 1e-5),
            "\nin_phase computed: {:?},\nin_phase expected: {:?}",
            result_in_phase,
            in_phase
        );
        assert!(
            array_within_tolerance(&result_quadrature, &quadrature, 0., 1e-5),
            "\nquadrature computed: {:?},\nquadrature expected: {:?}",
            result_quadrature,
            quadrature
        );
    }
}
