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

use super::iir_int::{IIRState, IIR};
use super::pll::PLL;
use super::trig::{atan2, cossin};
use super::{divide_round, Complex};

/// TODO these constants are copied from main.rs and should be
/// shared. Additionally, we should probably store the log2 values and
/// compute the actual values from these in main, as is done here.
pub const SAMPLE_BUFFER_SIZE_LOG2: usize = 0;
pub const SAMPLE_BUFFER_SIZE: usize = 1 << SAMPLE_BUFFER_SIZE_LOG2;

pub const ADC_SAMPLE_TICKS_LOG2: usize = 8;
pub const ADC_SAMPLE_TICKS: usize = 1 << ADC_SAMPLE_TICKS_LOG2;

pub const ADC_BATCHES_LOG2: usize =
    32 - SAMPLE_BUFFER_SIZE_LOG2 - ADC_SAMPLE_TICKS_LOG2;
pub const ADC_BATCHES: usize = 1 << ADC_BATCHES_LOG2;

pub const DECIMATED_BUFFER_SIZE: usize = 1;

/// Performs lock-in amplifier processing of a signal.
pub struct Lockin {
    harmonic: u32,
    phase_offset: u32,
    batch_index: u32,
    last_phase: Option<i64>,
    last_frequency: Option<i64>,
    pll: PLL,
    pll_shift_frequency: u8,
    pll_shift_phase: u8,
    iir: IIR,
    iirstate: [IIRState; 2],
}

impl Lockin {
    /// Initialize a new `Lockin` instance.
    ///
    /// # Arguments
    ///
    /// * `harmonic` - Integer scaling factor used to adjust the
    /// demodulation frequency. E.g., 2 would demodulate with the
    /// first harmonic.
    /// * `phase_offset` - Phase offset applied to the demodulation
    /// signal.
    /// * `iir` - IIR biquad filter.
    /// * `pll_shift_frequency` - See PLL::update().
    /// * `pll_shift_phase` - See PLL::update().
    ///
    /// # Returns
    ///
    /// New `Lockin` instance.
    pub fn new(
        harmonic: u32,
        phase_offset: u32,
        iir: IIR,
        pll_shift_frequency: u8,
        pll_shift_phase: u8,
    ) -> Self {
        Lockin {
            harmonic,
            phase_offset,
            batch_index: 0,
            last_phase: None,
            last_frequency: None,
            pll: PLL::default(),
            pll_shift_frequency,
            pll_shift_phase,
            iir,
            iirstate: [[0; 5]; 2],
        }
    }

    /// Demodulate an input signal with the complex reference signal.
    ///
    /// # Arguments
    ///
    /// * `adc_samples` - One batch of ADC samples.
    /// * `timestamp` - Counter value corresponding to the edges of an
    /// external reference signal. The counter is incremented by a
    /// fast internal clock. Each ADC sample batch can contain 0 or 1
    /// timestamps.
    ///
    /// # Returns
    ///
    /// The demodulated complex signal as a `Result`. When there are
    /// an insufficient number of timestamps to perform processing,
    /// `Err` is returned.
    pub fn demodulate(
        &mut self,
        adc_samples: &[i16],
        timestamp: Option<u32>,
    ) -> Result<[Complex<i32>; SAMPLE_BUFFER_SIZE], &str> {
        let frequency: i64;
        let phase: i64;

        match timestamp {
            Some(t) => {
                let res = self.pll.update(
                    t as i32,
                    self.pll_shift_frequency,
                    self.pll_shift_phase,
                );
                phase = res.0 as u32 as i64;
                frequency = res.1 as u32 as i64;
                self.last_phase = Some(phase);
                self.last_frequency = Some(frequency);
            }
            None => match self.last_phase {
                Some(t) => {
                    phase = t;
                    frequency = self.last_frequency.unwrap();
                }
                None => {
                    self.batch_index += 1;
                    if self.batch_index == ADC_BATCHES as u32 {
                        self.batch_index = 0;
                    }
                    return Err("insufficient timestamps");
                }
            },
        }

        let demodulation_frequency = divide_round(
            1 << (64 - SAMPLE_BUFFER_SIZE_LOG2 - ADC_BATCHES_LOG2),
            frequency,
        ) as u32;
        let demodulation_initial_phase = divide_round(
            (((self.batch_index as i64) << (32 - ADC_BATCHES_LOG2)) - phase)
                << 32,
            frequency,
        ) as u32;

        let mut demodulation_signal = [(0_i32, 0_i32); SAMPLE_BUFFER_SIZE];

        demodulation_signal
            .iter_mut()
            .zip(adc_samples.iter())
            .enumerate()
            .for_each(|(i, (s, sample))| {
                let sample_phase = (self.harmonic.wrapping_mul(
                    (demodulation_frequency.wrapping_mul(i as u32))
                        .wrapping_add(demodulation_initial_phase),
                ))
                .wrapping_add(self.phase_offset);
                let (cos, sin) = cossin(sample_phase as i32);
                // cos/sin take up 32 bits and sample takes up 16
                // bits. Make this fit into a 32 bit result.
                s.0 = ((*sample as i64 * cos as i64) >> 16) as i32;
                s.1 = ((*sample as i64 * sin as i64) >> 16) as i32;
            });

        if self.batch_index < ADC_BATCHES as u32 - 1 {
            self.batch_index += 1;
        } else {
            self.batch_index = 0;
            self.last_phase = Some(self.last_phase.unwrap() - (1 << 32));
        }

        Ok(demodulation_signal)
    }

    /// Filter the complex signal using the supplied biquad IIR. The
    /// signal array is modified in place.
    ///
    /// # Arguments
    ///
    /// * `signal` - Complex signal to filter.
    pub fn filter(&mut self, signal: &mut [Complex<i32>]) {
        signal.iter_mut().for_each(|s| {
            s.0 = self.iir.update(&mut self.iirstate[0], s.0);
            s.1 = self.iir.update(&mut self.iirstate[1], s.1);
        });
    }
}

/// Decimate the complex signal to `DECIMATED_BUFFER_SIZE`. The ratio
/// of `SAMPLE_BUFFER_SIZE` to `DECIMATED_BUFFER_SIZE` must be a power
/// of 2.
///
/// # Arguments
///
/// * `signal` - Complex signal to decimate.
///
/// # Returns
///
/// The decimated signal.
pub fn decimate(
    signal: [Complex<i32>; SAMPLE_BUFFER_SIZE],
) -> [Complex<i32>; DECIMATED_BUFFER_SIZE] {
    let n_k = SAMPLE_BUFFER_SIZE / DECIMATED_BUFFER_SIZE;
    debug_assert!(SAMPLE_BUFFER_SIZE == DECIMATED_BUFFER_SIZE || n_k % 2 == 0);

    let mut signal_decimated = [(0_i32, 0_i32); DECIMATED_BUFFER_SIZE];

    signal_decimated
        .iter_mut()
        .zip(signal.iter().step_by(n_k))
        .for_each(|(s_d, s)| {
            s_d.0 = s.0;
            s_d.1 = s.1;
        });

    signal_decimated
}

/// Compute the magnitude and phase from the complex signal. The
/// signal array is modified in place.
///
/// # Arguments
///
/// * `signal` - Complex signal for which the magnitude and phase
/// should be computed. TODO currently, we compute the square of the
/// magnitude. This should be changed to be the actual magnitude.
pub fn magnitude_phase(signal: &mut [Complex<i32>]) {
    signal.iter_mut().for_each(|s| {
        let new_i = [s.0, s.1].iter().map(|i| i * i).sum();
        let new_q = atan2(s.1, s.0);
        s.0 = new_i;
        s.1 = new_q;
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    /// Ensure that the demodulation signals are within some tolerance
    /// band of the target value given the phase and frequency values
    /// provided by the PLL.
    fn demodulate() {
        const PLL_SHIFT_FREQUENCY: u8 = 4;
        const PLL_SHIFT_PHASE: u8 = 3;
        const HARMONIC: u32 = 1;
        const PHASE_OFFSET: u32 = 0;
        let mut lockin = Lockin::new(
            HARMONIC,
            PHASE_OFFSET,
            IIR { ba: [0; 5] },
            PLL_SHIFT_FREQUENCY,
            PLL_SHIFT_PHASE,
        );

        // Duplicate the PLL outside demodulate so that we don't test
        // its behavior.
        let mut tracking_pll = PLL::default();
        let mut tracking_phase: i32 = 0;
        let mut tracking_frequency: i32 = 0;

        const REFERENCE_FREQUENCY: usize = 10_000;
        let mut reference_edge: usize = REFERENCE_FREQUENCY;

        // Ensure that we receive at most 1 timestamp per batch.
        debug_assert!(
            REFERENCE_FREQUENCY >= SAMPLE_BUFFER_SIZE * ADC_SAMPLE_TICKS
        );

        for batch in 0..100_000 {
            let tick: usize = batch * ADC_SAMPLE_TICKS * SAMPLE_BUFFER_SIZE;
            let timestamp: Option<u32>;

            // When the reference edge occurred during the current
            // batch acquisition, register the timestamp and update
            // the tracking PLL.
            if reference_edge >= tick
                && reference_edge < tick + ADC_SAMPLE_TICKS * SAMPLE_BUFFER_SIZE
            {
                timestamp = Some(reference_edge as u32);

                let tracking_update = tracking_pll.update(
                    reference_edge as i32,
                    PLL_SHIFT_FREQUENCY,
                    PLL_SHIFT_PHASE,
                );
                tracking_phase = tracking_update.0;
                tracking_frequency = tracking_update.1;

                reference_edge += REFERENCE_FREQUENCY;
            } else {
                timestamp = None;
            }

            let timestamp_before_batch = if tracking_phase > tick as i32 {
                // There can be at most 1 reference edge per batch, so
                // this will necessarily place the timestamp prior to
                // the current batch.
                tracking_phase - tracking_frequency
            } else {
                tracking_phase
            };

            let initial_phase = (((tick as f64
                - timestamp_before_batch as f64)
                / tracking_frequency as f64
                * (1_i64 << 32) as f64)
                .round()
                % u32::MAX as f64) as u32;
            let frequency = ((ADC_SAMPLE_TICKS as f64
                / tracking_frequency as f64
                * (1_i64 << 32) as f64)
                .round()
                % u32::MAX as f64) as u32;

            match lockin.demodulate(&[i16::MAX; SAMPLE_BUFFER_SIZE], timestamp)
            {
                Ok(v) => {
                    println!("batch      : {}", batch);
                    for sample in 0..SAMPLE_BUFFER_SIZE {
                        const TOL: i32 = 50_000;
                        let cos = v[sample].0;
                        let sin = v[sample].1;

                        let (mut target_cos, mut target_sin) = cossin(
                            HARMONIC
                                .wrapping_mul(
                                    (frequency.wrapping_mul(sample as u32))
                                        .wrapping_add(initial_phase),
                                )
                                .wrapping_add(PHASE_OFFSET)
                                as i32,
                        );
                        target_cos /= 2;
                        target_sin /= 2;

                        println!("sample     : {}", sample);
                        println!("tol        : {}", TOL);
                        println!("cos, target: {}, {}", cos, target_cos);
                        println!("sin, target: {}, {}", sin, target_sin);
                        assert!((cos - target_cos).abs() < TOL);
                        assert!((sin - target_sin).abs() < TOL);
                    }
                }
                Err(_) => {}
            }
        }
    }
}
