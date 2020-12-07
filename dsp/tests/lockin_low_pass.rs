use dsp::iir::IIR;
use dsp::lockin::{
    decimate, magnitude_phase, Lockin, ADC_SAMPLE_BUFFER_SIZE,
    DECIMATED_BUFFER_SIZE,
};
use dsp::Complex;

use std::f64::consts::PI;
use std::vec::Vec;

const ADC_MAX: f64 = 1.;
const ADC_MAX_COUNT: f64 = (1 << 15) as f64;

/// Single-frequency sinusoid.
#[derive(Copy, Clone)]
struct PureSine {
    // Frequency (in Hz).
    frequency: f64,
    // Amplitude in dBFS (decibels relative to full-scale). A 16-bit
    // ADC has a minimum dBFS for each sample of -90.
    amplitude_dbfs: f64,
    // Phase offset (in radians).
    phase_offset: f64,
}

/// Convert a dBFS voltage ratio to a linear ratio.
///
/// # Arguments
///
/// * `dbfs` - dB ratio relative to full scale.
///
/// # Returns
///
/// Linear value.
fn linear(dbfs: f64) -> f64 {
    let base = 10_f64;
    ADC_MAX * base.powf(dbfs / 20.)
}

/// Convert a linear voltage ratio to a dBFS ratio.
///
/// # Arguments
///
/// * `linear` - Linear voltage ratio.
///
/// # Returns
///
/// dBFS value.
fn dbfs(linear: f64) -> f64 {
    20. * (linear / ADC_MAX).log10()
}

/// Convert a real ADC input value in the range `-ADC_MAX` to
/// `+ADC_MAX` to an equivalent 16-bit ADC sampled value. This models
/// the ideal ADC transfer function.
///
/// # Arguments
///
/// * `x` - Real ADC input value.
///
/// # Returns
///
/// Sampled ADC value.
fn real_to_adc_sample(x: f64) -> i16 {
    let max: i32 = i16::MAX as i32;
    let min: i32 = i16::MIN as i32;

    let xi: i32 = (x / ADC_MAX * ADC_MAX_COUNT) as i32;

    // It's difficult to characterize the correct output result when
    // the inputs are clipped, so panic instead.
    if xi > max {
        panic!("Input clipped to maximum, result is unlikely to be correct.");
    } else if xi < min {
        panic!("Input clipped to minimum, result is unlikely to be correct.");
    }

    xi as i16
}

/// Generate `ADC_SAMPLE_BUFFER_SIZE` values of an ADC-sampled signal
/// starting at `timestamp_start`.
///
/// # Arguments
///
/// * `pure_signals` - Pure sinusoidal components of the ADC-sampled
/// signal.
/// * `timestamp_start` - Starting time of ADC-sampled signal in terms
/// of the internal clock count.
/// * `internal_frequency` - Internal clock frequency (in Hz).
/// * `adc_frequency` - ADC sampling frequency (in Hz).
///
/// # Returns
///
/// The sampled signal at the ADC input.
fn adc_sampled_signal(
    pure_signals: &Vec<PureSine>,
    timestamp_start: u64,
    internal_frequency: f64,
    adc_frequency: f64,
) -> [i16; ADC_SAMPLE_BUFFER_SIZE] {
    // amplitude of each pure signal
    let mut amplitude: Vec<f64> = Vec::<f64>::new();
    // initial phase value for each pure signal
    let mut initial_phase: Vec<f64> = Vec::<f64>::new();
    // phase increment at each ADC sample for each pure signal
    let mut phase_increment: Vec<f64> = Vec::<f64>::new();
    let adc_period = internal_frequency / adc_frequency;

    // For each pure sinusoid, compute the amplitude, phase
    // corresponding to the first ADC sample, and phase increment for
    // each subsequent ADC sample.
    for pure_signal in pure_signals.iter() {
        let signal_period = internal_frequency / pure_signal.frequency;
        let phase_offset_count =
            pure_signal.phase_offset / (2. * PI) * signal_period;
        let initial_phase_count =
            (phase_offset_count + timestamp_start as f64) % signal_period;

        amplitude.push(linear(pure_signal.amplitude_dbfs));
        initial_phase.push(2. * PI * initial_phase_count / signal_period);
        phase_increment.push(2. * PI * adc_period / signal_period);
    }

    // Compute the input signal corresponding to each ADC sample by
    // summing the contributions from each pure sinusoid.
    let mut signal: [i16; ADC_SAMPLE_BUFFER_SIZE] = [0; ADC_SAMPLE_BUFFER_SIZE];
    signal.iter_mut().enumerate().for_each(|(n, s)| {
        *s = real_to_adc_sample(
            amplitude
                .iter()
                .zip(initial_phase.iter())
                .zip(phase_increment.iter())
                .fold(0., |acc, ((a, phi), theta)| {
                    acc + a * (phi + theta * n as f64).sin()
                }),
        );
    });

    signal
}

/// Reference clock timestamp values in one ADC batch period starting
/// at `timestamp_start`. Also returns the number of valid timestamps.
///
/// # Arguments
///
/// * `reference_frequency` - External reference signal frequency (in
/// Hz).
/// * `timestamp_start` - Start time in terms of the internal clock
/// count. This is the start time of the current processing sequence
/// (i.e., for the current `ADC_SAMPLE_BUFFER_SIZE` ADC samples).
/// * `timestamp_stop` - Stop time in terms of the internal clock
/// count.
/// * `internal_frequency` - Internal clock frequency (in Hz).
///
/// # Returns
///
/// Tuple consisting of the number of valid timestamps in the ADC
/// batch period, followed by an array of the timestamp values.
fn adc_batch_timestamps(
    reference_frequency: f64,
    timestamps: &mut [u16],
    timestamp_start: u64,
    timestamp_stop: u64,
    internal_frequency: f64,
) -> &[u16] {
    let reference_period = internal_frequency / reference_frequency;
    let start_count = timestamp_start as f64 % reference_period;
    let mut valid_timestamps: usize = 0;

    let mut timestamp = (reference_period - start_count) % reference_period;
    while timestamp < (timestamp_stop - timestamp_start) as f64 {
        timestamps[valid_timestamps] = timestamp as u16;
        timestamp += reference_period;
        valid_timestamps += 1;
    }

    &timestamps[..valid_timestamps]
}

/// Lowpass biquad filter using cutoff and sampling frequencies.
/// Taken from:
/// https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
///
/// # Arguments
///
/// * `corner_frequency` - Corner frequency, or 3dB cutoff frequency
/// (in Hz).
/// * `sampling_frequency` - Sampling frequency (in Hz).
///
/// # Returns
///
/// 2nd-order IIR filter coefficients in the form [b0,b1,b2,a1,a2]. a0
/// is set to -1.
fn lowpass_iir_coefficients(
    corner_frequency: f64,
    sampling_frequency: f64,
) -> [f32; 5] {
    let normalized_angular_frequency: f64 =
        2. * PI * corner_frequency / sampling_frequency;
    let quality_factor: f64 = 1. / 2f64.sqrt();
    let alpha: f64 = normalized_angular_frequency.sin() / (2. * quality_factor);
    // All b coefficients have been multiplied by a factor of 2 in
    // comparison with the link above in order to set the passband
    // gain to 2.
    let mut b0: f64 = 1. - normalized_angular_frequency.cos();
    let mut b1: f64 = 2. * (1. - normalized_angular_frequency.cos());
    let mut b2: f64 = b0;
    let a0: f64 = 1. + alpha;
    let mut a1: f64 = -2. * normalized_angular_frequency.cos();
    let mut a2: f64 = 1. - alpha;
    b0 /= a0;
    b1 /= a0;
    b2 /= a0;
    a1 /= -a0;
    a2 /= -a0;

    [b0 as f32, b1 as f32, b2 as f32, a1 as f32, a2 as f32]
}

/// Check that a measured value is within some tolerance of the actual
/// value. This allows setting both fixed and relative tolerances.
///
/// # Arguments
///
/// * `actual` - Actual value with respect to which the magnitude of
/// the relative tolerance is computed.
/// * `computed` - Computed value. This is compared with the actual
/// value, `actual`.
/// * `fixed_tolerance` - Fixed tolerance.
/// * `relative_tolerance` - Relative tolerance.
/// `relative_tolerance`*`actual` gives the total contribution of the
/// relative tolerance.
///
/// # Returns
///
/// `true` if the `actual` and `computed` values are within the
/// specified tolerance of one another, and `false` otherwise.
fn tolerance_check(
    actual: f32,
    computed: f32,
    fixed_tolerance: f32,
    relative_tolerance: f32,
) -> bool {
    (actual - computed).abs()
        < max_error(actual, fixed_tolerance, relative_tolerance)
}

/// Maximum acceptable error from an actual value given fixed and
/// relative tolerances.
///
/// # Arguments
///
/// * `actual` - Actual value with respect to which the magnitude of the
/// relative tolerance is computed.
/// * `fixed_tolerance` - Fixed tolerance.
/// * `relative_tolerance` - Relative tolerance.
/// `relative_tolerance`*`actual` gives the total contribution of the
/// relative tolerance.
///
/// # Returns
///
/// Maximum acceptable error.
fn max_error(
    actual: f32,
    fixed_tolerance: f32,
    relative_tolerance: f32,
) -> f32 {
    relative_tolerance * actual.abs() + fixed_tolerance
}

/// Total noise amplitude of the input signal after sampling by the
/// ADC. This computes an upper bound of the total noise amplitude,
/// rather than its actual value.
///
/// # Arguments
///
/// * `noise_inputs` - Noise sources at the ADC input.
/// * `demodulation_frequency` - Frequency of the demodulation signal
/// (in Hz).
/// * `corner_frequency` - Low-pass filter 3dB corner (cutoff)
/// frequency.
///
/// # Returns
///
/// Upper bound of the total amplitude of all noise sources.
fn sampled_noise_amplitude(
    noise_inputs: &Vec<PureSine>,
    demodulation_frequency: f64,
    corner_frequency: f64,
) -> f64 {
    // There is not a simple way to compute the amplitude of a
    // superpostition of sinusoids with different frequencies and
    // phases. Although we can compute the amplitude in special cases
    // (e.g., two signals whose periods have a common multiple), these
    // do not help us in the general case. However, we can say that
    // the total amplitude will not be greater than the sum of the
    // amplitudes of the individual noise sources. We treat this as an
    // upper bound, and use it as an approximation of the actual
    // amplitude.

    let mut noise: f64 = noise_inputs
        .iter()
        .map(|n| {
            // Noise inputs create an oscillation at the output, where the
            // oscillation magnitude is determined by the strength of the
            // noise and its attenuation (attenuation is determined by its
            // proximity to the demodulation frequency and filter
            // rolloff).
            let octaves = ((n.frequency - demodulation_frequency).abs()
                / corner_frequency)
                .log2();
            // 2nd-order filter. Approximately 12dB/octave rolloff.
            let attenuation = -2. * 20. * 2_f64.log10() * octaves;
            linear(n.amplitude_dbfs + attenuation)
        })
        .sum();

    // Add in 1/2 LSB for the maximum amplitude deviation resulting
    // from quantization.
    noise += 1. / ADC_MAX_COUNT / 2.;

    noise
}

/// Compute the maximum effect of input noise on the lock-in magnitude
/// computation.
///
/// The maximum effect of noise on the magnitude computation is given
/// by:
///
/// | sqrt((I+n*sin(x))**2 + (Q+n*cos(x))**2) - sqrt(I**2 + Q**2) |
///
/// * I is the in-phase component of the portion of the input signal
/// with the same frequency as the demodulation signal.
/// * Q is the quadrature component.
/// * n is the total noise amplitude (from all contributions, after
/// attenuation from filtering).
/// * x is the phase of the demodulation signal.
///
/// We need to find the demodulation phase (x) that maximizes this
/// expression. We can ignore the absolute value operation by also
/// considering the expression minimum. The locations of the minimum
/// and maximum can be computed analytically by finding the value of x
/// when the derivative of this expression with respect to x is
/// 0. When we solve this equation, we find:
///
/// x = atan(I/Q)
///
/// It's worth noting that this solution is technically only valid
/// when cos(x)!=0 (i.e., x!=pi/2,-pi/2). However, this is not a
/// problem because we only get these values when Q=0. Rust correctly
/// computes atan(inf)=pi/2, which is precisely what we want because
/// x=pi/2 maximizes sin(x) and therefore also the noise effect.
///
/// The other maximum or minimum is pi radians away from this
/// value.
///
/// # Arguments
///
/// * `total_noise_amplitude` - Combined amplitude of all noise
/// sources sampled by the ADC.
/// * `in_phase_actual` - Value of the in-phase component if no noise
/// were present at the ADC input.
/// * `quadrature_actual` - Value of the quadrature component if no
/// noise were present at the ADC input.
/// * `desired_input_amplitude` - Amplitude of the desired input
/// signal. That is, the input signal component with the same
/// frequency as the demodulation signal.
///
/// # Returns
///
/// Approximation of the maximum effect on the magnitude computation
/// due to noise sources at the ADC input.
fn magnitude_noise(
    total_noise_amplitude: f64,
    in_phase_actual: f64,
    quadrature_actual: f64,
    desired_input_amplitude: f64,
) -> f64 {
    // See function documentation for explanation.
    let noise = |in_phase_delta: f64, quadrature_delta: f64| -> f64 {
        (((in_phase_actual + in_phase_delta).powf(2.)
            + (quadrature_actual + quadrature_delta).powf(2.))
        .sqrt()
            - desired_input_amplitude)
            .abs()
    };

    let phase = (in_phase_actual / quadrature_actual).atan();
    let max_noise_1 = noise(
        total_noise_amplitude * phase.sin(),
        total_noise_amplitude * phase.cos(),
    );
    let max_noise_2 = noise(
        total_noise_amplitude * (phase + PI).sin(),
        total_noise_amplitude * (phase + PI).cos(),
    );

    max_noise_1.max(max_noise_2)
}

/// Compute the maximum phase deviation from the correct value due to
/// the input noise sources.
///
/// The maximum effect of noise on the phase computation is given by:
///
/// | atan2(Q+n*cos(x), I+n*sin(x)) - atan2(Q, I) |
///
/// See `magnitude_noise` for an explanation of the terms in this
/// mathematical expression.
///
/// This expression is harder to compute analytically than the
/// expression in `magnitude_noise`. We could compute it numerically,
/// but that's expensive. However, we can use heuristics to try to
/// guess the values of x that will maximize the noise
/// effect. Intuitively, the difference will be largest when the
/// Y-argument of the atan2 function (Q+n*cos(x)) is pushed in the
/// opposite direction of the noise effect on the X-argument (i.e.,
/// cos(x) and sin(x) have different signs). We can use:
///
/// * sin(x)=+-1 (+- denotes plus or minus), cos(x)=0,
/// * sin(x)=0, cos(x)=+-1, and
/// * the value of x that maximizes |sin(x)-cos(x)| (when
/// sin(x)=1/sqrt(2) and cos(x)=-1/sqrt(2), or when the signs are
/// flipped)
///
/// The first choice addresses cases in which |I|>>|Q|, the second
/// choice addresses cases in which |Q|>>|I|, and the third choice
/// addresses cases in which |I|~|Q|. We can test all of these cases
/// as an approximation for the real maximum.
///
/// # Arguments
///
/// * `total_noise_amplitude` - Total amplitude of all input noise
/// sources.
/// * `in_phase_actual` - Value of the in-phase component if no noise
/// were present at the input.
/// * `quadrature_actual` - Value of the quadrature component if no
/// noise were present at the input.
///
/// # Returns
///
/// Approximation of the maximum effect on the phase computation due
/// to noise sources at the ADC input.
fn phase_noise(
    total_noise_amplitude: f64,
    in_phase_actual: f64,
    quadrature_actual: f64,
) -> f64 {
    // See function documentation for explanation.
    let noise = |in_phase_delta: f64, quadrature_delta: f64| -> f64 {
        ((quadrature_actual + quadrature_delta)
            .atan2(in_phase_actual + in_phase_delta)
            - quadrature_actual.atan2(in_phase_actual))
        .abs()
    };

    let mut max_noise: f64 = 0.;
    for (in_phase_delta, quadrature_delta) in [
        (
            total_noise_amplitude / 2_f64.sqrt(),
            total_noise_amplitude / -2_f64.sqrt(),
        ),
        (
            total_noise_amplitude / -2_f64.sqrt(),
            total_noise_amplitude / 2_f64.sqrt(),
        ),
        (total_noise_amplitude, 0.),
        (-total_noise_amplitude, 0.),
        (0., total_noise_amplitude),
        (0., -total_noise_amplitude),
    ]
    .iter()
    {
        max_noise = max_noise.max(noise(*in_phase_delta, *quadrature_delta));
    }

    max_noise
}

/// Lowpass filter test for in-phase/quadrature and magnitude/phase
/// computations.
///
/// This attempts to "intelligently" model acceptable tolerance ranges
/// for the measured in-phase, quadrature, magnitude and phase results
/// of lock-in processing for a typical low-pass filter
/// application. So, instead of testing whether the lock-in processing
/// extracts the true magnitude and phase (or in-phase and quadrature
/// components) of the input signal, it attempts to calculate what the
/// lock-in processing should compute given any set of input noise
/// sources. For example, if a noise source of sufficient strength
/// differs in frequency by 1kHz from the reference frequency and the
/// filter cutoff frequency is also 1kHz, testing if the lock-in
/// amplifier extracts the amplitude and phase of the input signal
/// whose frequency is equal to the demodulation frequency is doomed
/// to failure. Instead, this function tests whether the lock-in
/// correctly adheres to its actual transfer function, whether or not
/// it was given reasonable inputs. The logic for computing acceptable
/// tolerance ranges is performed in `sampled_noise_amplitude`,
/// `magnitude_noise`, and `phase_noise`.
///
/// # Arguments
///
/// * `internal_frequency` - Internal clock frequency (Hz). The
/// internal clock increments timestamp counter values used to
/// record the edges of the external reference.
/// * `adc_frequency` - ADC sampling frequency (in Hz).
/// * `reference_frequency` - External reference frequency (in Hz).
/// * `demodulation_phase_offset` - Phase offset applied to the
/// in-phase and quadrature demodulation signals.
/// * `harmonic` - Scaling factor for the demodulation
/// frequency. E.g., 2 would demodulate with the first harmonic of the
/// reference frequency.
/// * `corner_frequency` - Lowpass filter 3dB cutoff frequency.
/// * `desired_input` - `PureSine` giving the frequency, amplitude and
/// phase of the desired result.
/// * `noise_inputs` - Vector of `PureSine` for any noise inputs on top
/// of `desired_input`.
/// * `time_constant_factor` - Number of time constants after which
/// the output is considered valid.
/// * `tolerance` - Acceptable relative tolerance for the magnitude
/// and angle outputs. The outputs must remain within this tolerance
/// between `time_constant_factor` and `time_constant_factor+1` time
/// constants.
fn lowpass_test(
    internal_frequency: f64,
    adc_frequency: f64,
    reference_frequency: f64,
    demodulation_phase_offset: f64,
    harmonic: u32,
    corner_frequency: f64,
    desired_input: PureSine,
    noise_inputs: &mut Vec<PureSine>,
    time_constant_factor: f64,
    tolerance: f32,
) {
    let mut lockin = Lockin::new(
        demodulation_phase_offset as f32,
        (internal_frequency / adc_frequency) as u32,
        harmonic,
        IIR {
            ba: lowpass_iir_coefficients(corner_frequency, adc_frequency),
            y_offset: 0.,
            y_min: -ADC_MAX_COUNT as f32,
            y_max: (ADC_MAX_COUNT - 1.) as f32,
        },
    );

    let mut timestamp_start: u64 = 0;
    let time_constant: f64 = 1. / (2. * PI * corner_frequency);
    let samples =
        (time_constant_factor * time_constant * adc_frequency) as usize;
    // Ensure the result remains within tolerance for 1 time constant
    // after `time_constant_factor` time constants.
    let extra_samples = (time_constant * adc_frequency) as usize;
    let sample_count: u64 = (internal_frequency / adc_frequency) as u64
        * ADC_SAMPLE_BUFFER_SIZE as u64;

    let effective_phase_offset =
        desired_input.phase_offset - demodulation_phase_offset;
    let in_phase_actual =
        linear(desired_input.amplitude_dbfs) * effective_phase_offset.cos();
    let quadrature_actual =
        linear(desired_input.amplitude_dbfs) * effective_phase_offset.sin();

    let total_noise_amplitude = sampled_noise_amplitude(
        noise_inputs,
        reference_frequency * harmonic as f64,
        corner_frequency,
    );
    let total_magnitude_noise = magnitude_noise(
        total_noise_amplitude,
        in_phase_actual,
        quadrature_actual,
        linear(desired_input.amplitude_dbfs),
    );
    let total_phase_noise =
        phase_noise(total_noise_amplitude, in_phase_actual, quadrature_actual);

    let pure_signals = noise_inputs;
    pure_signals.push(desired_input);

    for n in 0..(samples + extra_samples) {
        let adc_signal: [i16; ADC_SAMPLE_BUFFER_SIZE] = adc_sampled_signal(
            &pure_signals,
            timestamp_start,
            internal_frequency,
            adc_frequency,
        );
        let mut timestamps_array = [0_u16; ADC_SAMPLE_BUFFER_SIZE / 2];
        let timestamps = adc_batch_timestamps(
            reference_frequency,
            &mut timestamps_array,
            timestamp_start,
            timestamp_start + sample_count - 1,
            internal_frequency,
        );

        let mut signal: [Complex<f32>; ADC_SAMPLE_BUFFER_SIZE];
        match lockin.demodulate(&adc_signal, timestamps) {
            Ok(s) => {
                signal = s;
            }
            Err(_) => {
                continue;
            }
        }

        lockin.filter(&mut signal);
        let signal_decimated = decimate(signal);

        let mut magnitude_phase_decimated = signal.clone();
        // let mut magnitude_decimated = in_phase_decimated.clone();
        // let mut phase_decimated = quadrature_decimated.clone();

        magnitude_phase(&mut magnitude_phase_decimated);

        // Ensure stable within tolerance for 1 time constant after
        // `time_constant_factor`.
        if n >= samples {
            for k in 0..DECIMATED_BUFFER_SIZE {
                let amplitude_normalized: f32 =
                    magnitude_phase_decimated[k].0 / ADC_MAX_COUNT as f32;
                assert!(
                    tolerance_check(linear(desired_input.amplitude_dbfs) as f32, amplitude_normalized, total_magnitude_noise as f32, tolerance),
                    "magnitude actual: {:.4} ({:.2} dBFS), magnitude computed: {:.4} ({:.2} dBFS), tolerance: {:.4}",
                    linear(desired_input.amplitude_dbfs),
                    desired_input.amplitude_dbfs,
                    amplitude_normalized,
                    dbfs(amplitude_normalized as f64),
                    max_error(linear(desired_input.amplitude_dbfs) as f32, total_magnitude_noise as f32, tolerance)
                );
                assert!(
                    tolerance_check(
                        effective_phase_offset as f32,
                        magnitude_phase_decimated[k].1,
                        total_phase_noise as f32,
                        tolerance
                    ),
                    "phase actual: {:.4}, phase computed: {:.4}, tolerance: {:.4}",
                    effective_phase_offset as f32,
                    magnitude_phase_decimated[k].1,
                    max_error(
                        effective_phase_offset as f32,
                        total_phase_noise as f32,
                        tolerance
                    )
                );

                let in_phase_normalized: f32 =
                    signal_decimated[k].0 / ADC_MAX_COUNT as f32;
                let quadrature_normalized: f32 =
                    signal_decimated[k].1 / ADC_MAX_COUNT as f32;
                assert!(
                    tolerance_check(
                        in_phase_actual as f32,
                        in_phase_normalized,
                        total_noise_amplitude as f32,
                        tolerance
                    ),
                    "in-phase actual: {:.4}, in-phase computed: {:.3}, tolerance: {:.4}",
                    in_phase_actual,
                    in_phase_normalized,
                    max_error(
                        in_phase_actual as f32,
                        total_noise_amplitude as f32,
                        tolerance
                    )
                );
                assert!(
                    tolerance_check(
                        quadrature_actual as f32,
                        quadrature_normalized,
                        total_noise_amplitude as f32,
                        tolerance
                    ),
                    "quadrature actual: {:.4}, quadrature computed: {:.4}, tolerance: {:.4}",
                    quadrature_actual,
                    quadrature_normalized,
                    max_error(
                        quadrature_actual as f32,
                        total_noise_amplitude as f32,
                        tolerance
                    )
                );
            }
        }

        timestamp_start += sample_count;
    }
}

#[test]
fn lowpass() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 100e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.1 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.9 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_demodulation_phase_offset_pi_2() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 100e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = PI / 2.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.1 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.9 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_phase_offset_pi_2() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 100e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: PI / 2.,
        },
        &mut vec![
            PureSine {
                frequency: 1.1 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.9 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_fundamental_111e3_phase_offset_pi_4() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 111e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: PI / 4.,
        },
        &mut vec![
            PureSine {
                frequency: 1.1 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.9 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_first_harmonic() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 50e3;
    let harmonic: u32 = 2;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_second_harmonic() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 50e3;
    let harmonic: u32 = 3;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_third_harmonic() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 50e3;
    let harmonic: u32 = 4;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_first_harmonic_phase_shift() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 50e3;
    let harmonic: u32 = 2;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: PI / 4.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_adc_frequency_1e6() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 1e6;
    let signal_frequency: f64 = 100e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_internal_frequency_125e6() {
    let internal_frequency: f64 = 125e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 100e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![
            PureSine {
                frequency: 1.2 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
            PureSine {
                frequency: 0.8 * demodulation_frequency,
                amplitude_dbfs: -20.,
                phase_offset: 0.,
            },
        ],
        time_constant_factor,
        tolerance,
    );
}

#[test]
fn lowpass_low_signal_frequency() {
    let internal_frequency: f64 = 100e6;
    let adc_frequency: f64 = 500e3;
    let signal_frequency: f64 = 10e3;
    let harmonic: u32 = 1;
    let corner_frequency: f64 = 1e3;
    let demodulation_frequency: f64 = harmonic as f64 * signal_frequency;
    let demodulation_phase_offset: f64 = 0.;
    let time_constant_factor: f64 = 5.;
    let tolerance: f32 = 1e-2;

    lowpass_test(
        internal_frequency,
        adc_frequency,
        signal_frequency,
        demodulation_phase_offset,
        harmonic,
        corner_frequency,
        PureSine {
            frequency: demodulation_frequency,
            amplitude_dbfs: -30.,
            phase_offset: 0.,
        },
        &mut vec![PureSine {
            frequency: 1.1 * demodulation_frequency,
            amplitude_dbfs: -20.,
            phase_offset: 0.,
        }],
        time_constant_factor,
        tolerance,
    );
}
