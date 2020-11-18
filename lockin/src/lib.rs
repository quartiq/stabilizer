#![no_std]
use core::cmp::Ord;
use core::convert::From;
use core::ops::SubAssign;
use core::f32::consts::PI;

extern crate libm;

mod trig;
use trig::{atan2, cos, sin};

// TODO temporary, pending new DMA code
pub const SAMPLE_BUFFER_SIZE: usize = 16;
pub const TSTAMP_BUFFER_SIZE: usize = SAMPLE_BUFFER_SIZE / 2;
pub const OUTPUT_BUFFER_SIZE: usize = 1;

/// Slow external reference edge timestamps.
#[derive(Copy, Clone)]
pub struct TimeStamp {
    // Timestamp value.
    pub count: u32,
    // Number of sequences before the current one that the timestamp
    // occurred. A sequence is a set of `SAMPLE_BUFFER_SIZE` ADC
    // samples. E.g., a timestamp from the current sequence has this
    // set to 0, a timestamp from the previous sequence has this set
    // to 1, etc. A value of -1 indicates an invalid timestamp (i.e.,
    // one that has not yet been set).
    pub sequences_old: i32,
}

impl TimeStamp {
    // Initialize a new TimeStamp.
    pub fn new() -> Self {
        TimeStamp {
            count: 0,
            sequences_old: -1,
        }
    }

    /// Increments `sequences_old` if TimeStamp is valid. This is
    /// called at the end of processing a sequence of ADC samples.
    pub fn new_sequence(&mut self) {
        if self.sequences_old != -1 {
            self.sequences_old += 1;
        }
    }

    /// Returns true if TimeStamp is valid.
    pub fn is_valid(&self) -> bool {
        self.sequences_old != -1
    }

    /// Set a new count value for TimeStamp. This also changes
    /// `sequences_old` to 0 to indicate the TimeStamp belongs to the
    /// current processing sequence.
    ///
    /// # Arguments
    ///
    /// * `newval` - New count value.
    pub fn new_count(&mut self, newval: u16) {
        self.count = newval as u32;
        self.sequences_old = 0;
    }
}

/// Compute the reference signal phase increment for each ADC sample
/// and use them to demodulate the ADC inputs.
macro_rules! prefilt_no_decimate {
    ( $adc_samples:expr, $ref_tstamps:expr, $valid_tstamps:expr, $phi:expr, $tadc:expr, $fscale:expr, $tstamps_mem:expr) => {{
        record_new_tstamps($ref_tstamps, $valid_tstamps, $tstamps_mem);
        let ts_valid_count = tstamps_valid_count($tstamps_mem);

        if ts_valid_count < 2 {
            return ([0.; OUTPUT_BUFFER_SIZE], [0.; OUTPUT_BUFFER_SIZE]);
        }

        let thetas = adc_phases(
            $ref_tstamps[0] as u32,
            $tstamps_mem,
            $phi,
            $fscale,
            $tadc,
        );
        let mut sines: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
        let mut cosines: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
        for i in 0..SAMPLE_BUFFER_SIZE {
            sines[i] = sin(thetas[i]);
            cosines[i] = cos(thetas[i]);
        }

        increment_tstamp_sequence($tstamps_mem);
        demod($adc_samples, sines, cosines)
    }};
}

/// Unfiltered in-phase and quadrature signals.
///
/// * `adc_samples` - ADC samples.
/// * `ref_tstamps` - Slow external reference clock counter values.
/// * `valid_tstamps` - Number of valid timestamps in `ref_tstamps`.
/// * `phi` - Demodulation phase offset specified as a number of counts
/// of the internal clock period.
/// * `tadc` - Number of internal clock periods in one ADC sampling
/// period.
/// * `fscale` - Scaling factor for the demodulation frequency. For
/// instance, 2 would demodulate with the first harmonic of the reference
/// frequency.
/// * `tstamps_mem` - Last two external reference timestamps (i.e., recorded
/// values of `ref_tstamps`.)
///
/// # Latency (ADC batch size = 16): 5.6us
///
/// * 2.0us to compute ADC phase values
/// * 3.4us to compute sin and cos
pub fn prefilt(
    adc_samples: [i16; SAMPLE_BUFFER_SIZE],
    ref_tstamps: [u16; TSTAMP_BUFFER_SIZE],
    valid_tstamps: usize,
    phi: u32,
    tadc: u32,
    fscale: u32,
    tstamps_mem: &mut [TimeStamp; 2],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) = prefilt_no_decimate!(
        adc_samples,
        ref_tstamps,
        valid_tstamps,
        phi,
        tadc,
        fscale,
        tstamps_mem
    );
    decimate(i, q)
}

/// Filtered in-phase and quadrature signals.
///
/// # Arguments
///
/// See `prefilt`.
/// * `iir` - IIR biquad for filtering in-phase and quadrature
/// components.
/// * `iirstate` - IIR biquad state for in-phase and quadrature
/// components.
///
/// # Latency (ADC batch size = 16, DAC size = 1, filter then decimate): 9.4us
/// # Latency (ADC batch size = 16, DAC size = 1, boxcar then filter): 6.2us
///
/// * 5.6us from `prefilt`
/// * 3.5us from `filter`
pub fn postfilt_iq(
    adc_samples: [i16; SAMPLE_BUFFER_SIZE],
    ref_tstamps: [u16; TSTAMP_BUFFER_SIZE],
    valid_tstamps: usize,
    phi: u32,
    tadc: u32,
    fscale: u32,
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
    tstamps_mem: &mut [TimeStamp; 2],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) = prefilt_no_decimate!(
        adc_samples,
        ref_tstamps,
        valid_tstamps,
        phi,
        tadc,
        fscale,
        tstamps_mem
    );

    // TODO need to decide whether to filter first then decimate, or
    // average and filter. Then, remove the other implementation.

    // filter then decimate
    let (ifilt, qfilt) = filter(i, q, iir, iirstate);
    decimate(ifilt, qfilt)

    // // average then filter
    // let iavg = avg(i);
    // let qavg = avg(q);
    // let ifilt = iir[0].update(&mut iirstate[0], iavg);
    // let qfilt = iir[1].update(&mut iirstate[1], qavg);
    // // TODO averaging only works for output size of 1.
    // ([ifilt], [qfilt])
}

/// Filtered magnitude and angle signals.
///
/// # Arguments
///
/// See `postfilt_iq`.
///
/// # Latency (ADC batch size = 16, DAC size = 1, filter then decimate): 9.6us
/// # Latency (ADC batch size = 16, DAC size = 1, boxcar then filter): 6.4us
pub fn postfilt_at(
    adc_samples: [i16; SAMPLE_BUFFER_SIZE],
    ref_tstamps: [u16; TSTAMP_BUFFER_SIZE],
    valid_tstamps: usize,
    phi: u32,
    tadc: u32,
    fscale: u32,
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
    tstamps_mem: &mut [TimeStamp; 2],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let (i, q) = {
        let (ipre, qpre) = prefilt_no_decimate!(
            adc_samples,
            ref_tstamps,
            valid_tstamps,
            phi,
            tadc,
            fscale,
            tstamps_mem
        );
        // TODO need to decide whether to filter first then decimate,
        // or average and filter. Then, remove the other
        // implementation.

        // filter then decimate
        let (ifilt, qfilt) = filter(ipre, qpre, iir, iirstate);
        decimate(ifilt, qfilt)

        // // average then filter
        // let iavg = avg(ipre);
        // let qavg = avg(qpre);
        // let ifilt = iir[0].update(&mut iirstate[0], iavg);
        // let qfilt = iir[1].update(&mut iirstate[1], qavg);
        // // TODO averaging only works for output size of 1.
        // ([ifilt], [qfilt])
    };

    // 198ns for size of 1.
    iq_to_at_map(i, q)
}

/// ARR (counter overflow value).
///
/// # Arguments
///
/// * `tadc` - Number of internal clock periods in one ADC sampling
/// period.
/// * `n` - Number of ADC samples in one batch.
pub const fn arr(tadc: u32, n: u16) -> u16 {
    tadc as u16 * n
}

/// Arithmetic average.
///
/// TODO can remove if we filter then decimate (only needed for
/// average then filter).
///
/// # Arguments
///
/// * `x` - Array of samples to average.
fn avg(x: [f32; SAMPLE_BUFFER_SIZE]) -> f32 {
    let mut total: f32 = 0.;
    for val in x.iter() {
        total += val;
    }
    total
}

/// Count number of valid TimeStamps from `tstamps`.
fn tstamps_valid_count(tstamps: &[TimeStamp; 2]) -> usize {
    let mut valid_count: usize = 0;
    for i in 0..2 {
        if tstamps[i].is_valid() {
            valid_count += 1;
        }
    }
    valid_count
}

/// Add new timestamps to the TimeStamp memory.
///
/// # Arguments
///
/// * `ref_tstamps` - New timestamp values.
/// * `valid_tstamps` - Number of valid timestamps.
/// * `tstamps_mem` - Last 2 recorded timestamps.
fn record_new_tstamps(
    ref_tstamps: [u16; TSTAMP_BUFFER_SIZE],
    valid_tstamps: usize,
    tstamps_mem: &mut [TimeStamp; 2],
) {
    if valid_tstamps > 1 {
        tstamps_mem[1].new_count(ref_tstamps[valid_tstamps - 2]);
        tstamps_mem[0].new_count(ref_tstamps[valid_tstamps - 1]);
    } else if valid_tstamps == 1 {
        tstamps_mem[1].count = tstamps_mem[0].count;
        tstamps_mem[1].sequences_old = tstamps_mem[0].sequences_old;
        tstamps_mem[0].new_count(ref_tstamps[valid_tstamps - 1]);
    }
}

/// Map `iq_to_a` and `iq_to_t` to each pair of `i` and `q`.
fn iq_to_at_map(
    i: [f32; OUTPUT_BUFFER_SIZE],
    q: [f32; OUTPUT_BUFFER_SIZE],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    let mut a: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    let mut t: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    for k in 0..OUTPUT_BUFFER_SIZE {
        a[k] = iq_to_a(i[k], q[k]);
        t[k] = iq_to_t(i[k], q[k]);
    }
    (a, t)
}

/// Returns magnitude from in-phase and quadrature signals.
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn iq_to_a(i: f32, q: f32) -> f32 {
    2. * sqrt(pow2(i) + pow2(q))
}

/// Returns angle from in-phase and quadrature signals.
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn iq_to_t(i: f32, q: f32) -> f32 {
    atan2(q, i)
}

/// Demodulation phase values corresponding to each ADC sample.
///
/// # Arguments
///
/// * `first_t` - First timestamp value from the current processing
/// period. The value provided here is ignored if there were no
/// timestamps in the current processing period.
/// * `tstamps_mem` - Recorded TimeStamps.
/// * `phi` - Reference phase offset.
/// * `fscale` - Frequency scaling factor for the demodulation signal.
/// * `tadc` - ADC sampling period.
///
/// # Latency (ADC batch size = 16): 3.3us
///
/// 2.9us from computing `real_phase`.
fn adc_phases(
    first_t: u32,
    tstamps_mem: &mut [TimeStamp; 2],
    phi: u32,
    fscale: u32,
    tadc: u32,
) -> [f32; SAMPLE_BUFFER_SIZE] {
    let overflow_count: u32 = tadc * SAMPLE_BUFFER_SIZE as u32;
    let tref_count: u32 = tstamps_diff(tstamps_mem, overflow_count);
    let mut thetas: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut theta_count: u32;

    // 68ns
    if tstamps_mem[0].sequences_old == 0 {
        theta_count = tref_count - first_t;
    } else {
        theta_count = tstamps_diff(
            &[
                TimeStamp {
                    count: 0,
                    sequences_old: 0,
                },
                tstamps_mem[0],
            ],
            overflow_count,
        );
    }

    thetas[0] = real_phase(theta_count, fscale, tref_count, phi);
    for i in 1..SAMPLE_BUFFER_SIZE {
        theta_count += tadc;
        thetas[i] = real_phase(theta_count, fscale, tref_count, phi);
    }

    thetas
}

/// Number of fast clock counts between two consecutive
/// TimeStamps. This requires that `tstamps[0]` is more recent than
/// `tstamps[1]` but otherwise imposes no restrictions on them. For
/// instance, they can be from different processing periods and this
/// will still count the number of counts between them, accounting for
/// overflow wrapping. Both TimeStamps must be valid for this to be
/// used.
///
/// # Arguments
///
/// * `tstamps_mem` - Recorded TimeStamps.
/// * `overflow_count` - Max timestamp value.
fn tstamps_diff(tstamps_mem: &[TimeStamp; 2], overflow_count: u32) -> u32 {
    debug_assert!(tstamps_valid_count(tstamps_mem) == 2);

    if tstamps_mem[0].sequences_old == tstamps_mem[1].sequences_old {
        return tstamps_mem[0].count - tstamps_mem[1].count;
    }

    let rem0: u32 = tstamps_mem[0].count;
    let rem1: u32 = overflow_count - tstamps_mem[1].count;
    let empty_sequences =
        tstamps_mem[1].sequences_old - tstamps_mem[0].sequences_old - 1;

    rem0 + rem1 + overflow_count * empty_sequences as u32
}

/// Increment `sequences_old` in each TimeStamp of `tstamps_mem`.
fn increment_tstamp_sequence(tstamps_mem: &mut [TimeStamp; 2]) {
    for tstamp in tstamps_mem {
        tstamp.new_sequence();
    }
}

/// Compute the phase (in radians) for a phase given in counts
/// relative to some period in counts.
///
/// # Arguments
///
/// * `theta_count` - Phase in counts. This can be greater than the
/// period in counts.
/// * `fscale` - Frequency scaling factor for harmonic demodulation.
/// * `period_count` - Number of counts in 1 period.
/// * `phase_count` - Phase offset in counts.
///
/// # Latency: 138ns
fn real_phase(
    theta_count: u32,
    fscale: u32,
    period_count: u32,
    phase_count: u32,
) -> f32 {
    let total_angle =
        modulo::<u32>(theta_count * fscale + phase_count, period_count);
    2. * PI * (total_angle as f32 / period_count as f32)
}

/// Arithmetic modulo operation. This requires that the dividend is
/// non-negative and the divisor is positive. Although this may work
/// in some other cases, these cases are not supported.
fn modulo<T: SubAssign + Ord + Copy + From<u8>>(dividend: T, divisor: T) -> T {
    debug_assert!(dividend >= T::from(0u8) && divisor > T::from(0u8));

    let mut rem: T = dividend;
    while rem > divisor {
        rem -= divisor;
    }
    rem
}

/// Filter in-phase and quadrature signals with the IIR biquad filter.
///
/// TODO this currently does not offer enough filtering flexibility. For
/// instance, we might want to filter with two consecutive biquads.
///
/// # Arguments
///
/// `i` - In-phase signals.
/// `q` - Quadrature signals.
/// `iir` - IIR filters for the in phase (index 0) and quadrature
/// (index 1) signals.
/// `iirstate` - State of each IIR filter.
///
/// Latency (ADC batch size = 16): 3.5us
fn filter(
    i: [f32; SAMPLE_BUFFER_SIZE],
    q: [f32; SAMPLE_BUFFER_SIZE],
    iir: [iir::IIR; 2],
    iirstate: &mut [iir::IIRState; 2],
) -> ([f32; SAMPLE_BUFFER_SIZE], [f32; SAMPLE_BUFFER_SIZE]) {
    let mut filt_i: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut filt_q: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];

    for n in 0..SAMPLE_BUFFER_SIZE {
        filt_i[n] = iir[0].update(&mut iirstate[0], i[n]);
        filt_q[n] = iir[1].update(&mut iirstate[1], q[n]);
    }

    (filt_i, filt_q)
}

/// Decimate from `SAMPLE_BUFFER_SIZE` to `OUTPUT_BUFFER_SIZE`
/// samples. `SAMPLE_BUFFER_SIZE`/`OUTPUT_BUFFER_SIZE` is assumed to
/// be equal to 2**n, where n is some non-negative integer. Decimates
/// the in-phase and quadrature signals separately and returns the
/// result as (i, q).
///
/// # Arguments
///
/// `i` - In-phase signal.
/// `q` - Quadrature signal.
fn decimate(
    i: [f32; SAMPLE_BUFFER_SIZE],
    q: [f32; SAMPLE_BUFFER_SIZE],
) -> ([f32; OUTPUT_BUFFER_SIZE], [f32; OUTPUT_BUFFER_SIZE]) {
    debug_assert!(
        SAMPLE_BUFFER_SIZE == OUTPUT_BUFFER_SIZE
            || (SAMPLE_BUFFER_SIZE / OUTPUT_BUFFER_SIZE) % 2 == 0
    );

    let n_k: usize = SAMPLE_BUFFER_SIZE / OUTPUT_BUFFER_SIZE;
    let mut res_i: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];
    let mut res_q: [f32; OUTPUT_BUFFER_SIZE] = [0.; OUTPUT_BUFFER_SIZE];

    let mut n: usize = 0;
    let mut k: usize = 0;
    while n < SAMPLE_BUFFER_SIZE {
        res_i[k] = i[n];
        res_q[k] = q[n];
        k += 1;
        n += n_k;
    }

    (res_i, res_q)
}

/// Demodulate ADC inputs with in-phase and quadrature signals.
///
/// # Arguments
///
/// * `adc_samples` - ADC samples.
/// * `sines` - Reference sine signal.
/// * `cosines` - Reference cosine signal.
fn demod(
    adc_samples: [i16; SAMPLE_BUFFER_SIZE],
    sines: [f32; SAMPLE_BUFFER_SIZE],
    cosines: [f32; SAMPLE_BUFFER_SIZE],
) -> ([f32; SAMPLE_BUFFER_SIZE], [f32; SAMPLE_BUFFER_SIZE]) {
    let mut i: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];
    let mut q: [f32; SAMPLE_BUFFER_SIZE] = [0.; SAMPLE_BUFFER_SIZE];

    for n in 0..SAMPLE_BUFFER_SIZE {
        let xf_n: f32 = adc_samples[n] as f32;
        i[n] = xf_n * sines[n];
        q[n] = xf_n * cosines[n];
    }
    (i, q)
}

fn sqrt(x: f32) -> f32 {
    libm::sqrtf(x)
}

fn pow2(x: f32) -> f32 {
    x * x
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;

    // TODO we should test behavior for different batch sizes and
    // output sizes. However, this isn't possible until const generics
    // are stabilized.

    fn abs(x: f32) -> f32 {
        if x >= 0. {
            x
        } else {
            -x
        }
    }

    fn max(x: f32, y: f32) -> f32 {
        if x > y {
            x
        } else {
            y
        }
    }

    fn f32_is_close(a: f32, b: f32) -> bool {
        abs(a - b) <= (max(a.abs(), b.abs()) * f32::EPSILON)
    }

    fn f32_is_close_mod(a: f32, b: f32, divisor: f32) -> bool {
        (abs(a - b) <= (max(a.abs(), b.abs()) * f32::EPSILON))
            || (abs(abs(a - divisor) - b)
                <= (max(a.abs(), b.abs()) * f32::EPSILON))
    }

    #[test]
    fn f32_is_close_mod_test() {
        let divisor: f32 = 6.28;
        assert_eq!(
            f32_is_close_mod(
                (divisor as f64 * (1. - f32::EPSILON as f64 / 2.)) as f32,
                0.,
                divisor
            ),
            true
        );
        assert_eq!(
            f32_is_close_mod(
                0.,
                (divisor as f64 * (1. - f32::EPSILON as f64 / 2.)) as f32,
                divisor
            ),
            true
        );
        assert_eq!(
            f32_is_close_mod(
                1. + (f32::EPSILON as f64 / 2.) as f32,
                1.,
                divisor
            ),
            true
        );
        assert_eq!(
            f32_is_close_mod(
                (divisor as f64 * (1. - 2. * f32::EPSILON as f64)) as f32,
                0.,
                divisor
            ),
            false
        );
        assert_eq!(
            f32_is_close_mod(
                0.,
                (divisor as f64 * (1. - 2. * f32::EPSILON as f64)) as f32,
                divisor
            ),
            false
        );
        assert_eq!(
            f32_is_close_mod(
                1. + (2. * f32::EPSILON as f64) as f32,
                1.,
                divisor
            ),
            false
        );
    }

    #[test]
    fn arr_n_16_ffast_1e6_fadc_5e5() {
        let tadc: u32 = 200;
        let n: u16 = 16;
        assert_eq!(arr(tadc, n), 3200);
    }

    #[test]
    fn decimate_n16_k1() {
        let i_in: [f32; 16] = [
            0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.10, 0.11,
            0.12, 0.13, 0.14, 0.15, 0.16,
        ];
        let q_in: [f32; 16] = [
            0.17, 0.18, 0.19, 0.20, 0.21, 0.22, 0.23, 0.24, 0.25, 0.26, 0.27,
            0.28, 0.29, 0.30, 0.31, 0.32,
        ];
        assert!(decimate(i_in, q_in) == ([0.01], [0.17]));
    }

    #[test]
    fn real_phase_per_1000_phi_0() {
        let period_count: u32 = 1000;
        let phi_real: f32 = 0.;
        let phi: u32 = (phi_real * period_count as f32) as u32;
        // PI/4 increments
        assert!(f32_is_close_mod(
            real_phase(0, 1, period_count, phi),
            0.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(125, 1, period_count, phi),
            PI / 4.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(125, 2, period_count, phi),
            PI / 2.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(250, 1, period_count, phi),
            PI / 2.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(375, 1, period_count, phi),
            3. * PI / 4.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(375, 2, period_count, phi),
            3. * PI / 2.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(500, 1, period_count, phi),
            PI,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(625, 1, period_count, phi),
            5. * PI / 4.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(750, 1, period_count, phi),
            3. * PI / 2.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(875, 1, period_count, phi),
            7. * PI / 4.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(875, 2, period_count, phi),
            3. * PI / 2.,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(1000, 1, period_count, phi),
            0.,
            2. * PI
        ));
        // other, < 1000
        assert!(f32_is_close_mod(
            real_phase(1, 1, period_count, phi),
            6.28318530718e-3,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(7, 1, period_count, phi),
            0.0439822971503,
            2. * PI
        ));
        assert!(f32_is_close_mod(
            real_phase(763, 1, period_count, phi),
            4.79407038938,
            2. * PI
        ));
        // > 1000
        for angle_count in 0..period_count as usize - 1 {
            for p in 0..3 {
                assert!(f32_is_close_mod(
                    real_phase(
                        (angle_count as f32 + p as f32 * period_count as f32)
                            as u32,
                        1,
                        period_count,
                        phi
                    ),
                    real_phase(angle_count as u32, 1, period_count, phi),
                    2. * PI
                ));
            }
        }
    }

    #[test]
    fn real_phase_per_1000_phi_adjust() {
        let period_count: u32 = 1000;
        for theta in [0, 20, 611, 987].iter() {
            for phi in 0..period_count as usize - 1 {
                assert!(f32_is_close(
                    real_phase(*theta, 1, period_count, phi as u32),
                    real_phase(*theta + phi as u32, 1, period_count, 0)
                ))
            }
        }
    }

    #[test]
    fn real_phase_per_1000_fscale_2_phi_adjust() {
        let period_count: u32 = 1000;
        let fscale: u32 = 2;
        for theta in [0, 20, 611, 987].iter() {
            for phi in 0..period_count as usize - 1 {
                assert!(f32_is_close(
                    real_phase(*theta, fscale, period_count, phi as u32),
                    real_phase(*theta * fscale + phi as u32, 1, period_count, 0)
                ))
            }
        }
    }

    #[test]
    fn increment_tstamp_sequence_valid_invalid() {
        let mut tstamps = [
            TimeStamp {
                count: 0,
                sequences_old: 0,
            },
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
        ];
        increment_tstamp_sequence(&mut tstamps);
        assert_eq!(tstamps[0].sequences_old, 1);
        assert_eq!(tstamps[1].sequences_old, -1);
    }

    #[test]
    fn tstamps_valid_count_test() {
        for (valid_num, old1, old2) in
            [(0, -1, -1), (1, 1, -1), (1, -1, 1), (2, 1, 1)].iter()
        {
            assert_eq!(
                tstamps_valid_count(&[
                    TimeStamp {
                        count: 5,
                        sequences_old: *old1 as i32,
                    },
                    TimeStamp {
                        count: 0,
                        sequences_old: *old2 as i32,
                    }
                ]),
                *valid_num as usize
            );
        }
    }

    #[test]
    fn record_new_tstamps_m8_r1() {
        let mut tstamps = [
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
        ];
        record_new_tstamps([1, 0, 0, 0, 0, 0, 0, 0], 1, &mut tstamps);
        assert_eq!(tstamps[0].count, 1);
        assert_eq!(tstamps[0].sequences_old, 0);
        assert_eq!(tstamps[1].count, 0);
        assert_eq!(tstamps[1].sequences_old, -1);
    }

    #[test]
    fn record_new_tstamps_m8_r2() {
        let mut tstamps = [
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
        ];
        record_new_tstamps([1, 2, 0, 0, 0, 0, 0, 0], 2, &mut tstamps);
        assert_eq!(tstamps[0].count, 2);
        assert_eq!(tstamps[0].sequences_old, 0);
        assert_eq!(tstamps[1].count, 1);
        assert_eq!(tstamps[1].sequences_old, 0);
    }

    #[test]
    fn record_new_tstamps_m8_r3() {
        let mut tstamps = [
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
            TimeStamp {
                count: 0,
                sequences_old: -1,
            },
        ];
        record_new_tstamps([1, 2, 3, 0, 0, 0, 0, 0], 3, &mut tstamps);
        assert_eq!(tstamps[0].count, 3);
        assert_eq!(tstamps[0].sequences_old, 0);
        assert_eq!(tstamps[1].count, 2);
        assert_eq!(tstamps[1].sequences_old, 0);
    }

    #[test]
    fn iq_to_a_test() {
        assert!(f32_is_close(
            iq_to_a(1. / 2f32.sqrt(), 1. / 2f32.sqrt()),
            2.
        ));
        assert!(f32_is_close(iq_to_a(0.1, 1.6), 3.20624390838));
        assert!(f32_is_close(iq_to_a(-0.1, 1.6), 3.20624390838));
        assert!(f32_is_close(iq_to_a(0.1, -1.6), 3.20624390838));
    }

    #[test]
    fn tstamps_diff_ofcount_1000() {
        let overflow_count: u32 = 1000;
        // same sequence
        let tstamps = [
            TimeStamp {
                count: 126,
                sequences_old: 0,
            },
            TimeStamp {
                count: 33,
                sequences_old: 0,
            },
        ];
        assert_eq!(tstamps_diff(&tstamps, overflow_count), 93);
        // adjacent sequences
        let tstamps = [
            TimeStamp {
                count: 9,
                sequences_old: 0,
            },
            TimeStamp {
                count: 33,
                sequences_old: 1,
            },
        ];
        assert_eq!(tstamps_diff(&tstamps, overflow_count), 976);
        let tstamps = [
            TimeStamp {
                count: 35,
                sequences_old: 0,
            },
            TimeStamp {
                count: 33,
                sequences_old: 1,
            },
        ];
        assert_eq!(tstamps_diff(&tstamps, overflow_count), 1002);
        // non-adjacent sequences
        let tstamps = [
            TimeStamp {
                count: 9,
                sequences_old: 0,
            },
            TimeStamp {
                count: 33,
                sequences_old: 2,
            },
        ];
        assert_eq!(tstamps_diff(&tstamps, overflow_count), 1976);
        let tstamps = [
            TimeStamp {
                count: 35,
                sequences_old: 0,
            },
            TimeStamp {
                count: 33,
                sequences_old: 2,
            },
        ];
        assert_eq!(tstamps_diff(&tstamps, overflow_count), 2002);
    }

    #[test]
    fn demod_n16() {
        let adc_phases: [i16; SAMPLE_BUFFER_SIZE] =
            [-1, 0, 1, -18, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        let sines: [f32; SAMPLE_BUFFER_SIZE] = [
            0.1, -0.3, 18.76, -33.1, 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
            0., 0.,
        ];
        let cosines: [f32; SAMPLE_BUFFER_SIZE] = [
            -0.2389, 0.1823, 0.123, -0.5839, 0., 0., 0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0.,
        ];
        let (i, q) = demod(adc_phases, sines, cosines);
        for n in 0..SAMPLE_BUFFER_SIZE {
            assert!(f32_is_close(i[n], adc_phases[n] as f32 * sines[n]));
            assert!(f32_is_close(q[n], adc_phases[n] as f32 * cosines[n]));
        }
    }
}
