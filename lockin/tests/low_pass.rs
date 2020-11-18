extern crate lockin;
extern crate std;

use std::f64::consts::PI;
use std::vec::Vec;

use iir::{IIRState, IIR};
use lockin::{
    postfilt_at, postfilt_iq, TimeStamp, OUTPUT_BUFFER_SIZE,
    SAMPLE_BUFFER_SIZE, TSTAMP_BUFFER_SIZE,
};

const ADC_MAX: f64 = 1.;
const ADC_MAX_COUNTS: f64 = (1 << 15) as f64;

/// Pure sinusoid with a given frequency (in Hz), amplitude (in dBFS)
/// and phase offset.
struct PureSine {
    freq: f64,
    // 16-bit ADC has a min dBFS for each sample of -90.
    amp_dbfs: f64,
    phi: f64,
}

/// Convert a dBFS voltage ratio to a linear ratio.
///
/// # Arguments
///
/// * `dbfs` - dB ratio relative to full scale.
fn dbfs_to_linear(dbfs: f64) -> f64 {
    let base = 10.0_f64;
    ADC_MAX * base.powf(dbfs / 20.)
}

/// Convert a linear voltage ratio to a dBFS ratio.
///
/// # Arguments
///
/// * `linear` - Linear voltage ratio.
fn linear_to_dbfs(linear: f64) -> f64 {
    20. * (linear / ADC_MAX).log10()
}

/// Number of fast clock periods in one reference signal period.
///
/// # Arguments
///
/// * `freq` - Reference frequency (in Hz).
/// * `ffast` - Fast external clock frequency (in Hz).
fn freq_to_tcounts(freq: f64, ffast: f64) -> f64 {
    ffast / freq
}

/// Number of fast clock counts in one ADC sampling period.
///
/// # Arguments
///
/// * `fadc` - ADC sampling frequency (in Hz).
/// * `ffast` - Fast clock frequency (in Hz).
fn adc_counts(fadc: f64, ffast: f64) -> f64 {
    ffast / fadc
}

/// Convert a real input value in the range `-ADC_MAX` to `+ADC_MAX`
/// to an equivalent 16-bit ADC sampled value.
///
/// # Arguments
///
/// * `x` - Real ADC input value.
fn real_to_adc_sample(x: f64) -> i16 {
    let max: i64 = i16::MAX as i64;
    let min: i64 = i16::MIN as i64;

    let xi: i64 = (x / ADC_MAX * ADC_MAX_COUNTS) as i64;

    // Clip inputs. It's hard to characterize the correct output
    // result when the inputs are clipped, so panic instead.
    if xi > max {
        panic!("Input clipped to maximum, result is unlikely to be correct.");
    } else if xi < min {
        panic!("Input clipped to minimum, result is unlikely to be correct.");
    }

    xi as i16
}

/// Generate `SAMPLE_BUFFER_SIZE` values of an input signal starting
/// at `tstart`.
///
/// # Arguments
///
/// * `pure_sigs` - Sinusoidal components of input signal.
/// * `tstart` - Starting time of input signal in terms of fast clock
/// counts.
/// * `ffast` - Fast clock frequency (in Hz).
/// * `fadc` - ADC sampling frequency (in Hz).
fn input_signal(
    pure_sigs: &Vec<PureSine>,
    tstart: u64,
    ffast: f64,
    fadc: f64,
) -> [i16; SAMPLE_BUFFER_SIZE] {
    let mut sig: [i16; SAMPLE_BUFFER_SIZE] = [0; SAMPLE_BUFFER_SIZE];
    let mut amplitudes: Vec<f64> = Vec::<f64>::new();
    let mut theta_starts: Vec<f64> = Vec::<f64>::new();
    let mut theta_incs: Vec<f64> = Vec::<f64>::new();

    for elem in pure_sigs.iter() {
        let elem_period = freq_to_tcounts(elem.freq, ffast);
        let phi_counts = elem.phi / (2. * PI) * elem_period;
        let theta_start_counts = (phi_counts + tstart as f64) % elem_period;

        amplitudes.push(dbfs_to_linear(elem.amp_dbfs));
        theta_starts.push(2. * PI * theta_start_counts / elem_period);
        theta_incs.push(2. * PI * adc_counts(fadc, ffast) / elem_period);
    }

    for n in 0..SAMPLE_BUFFER_SIZE {
        let mut sigf_n: f64 = 0.;
        for i in 0..pure_sigs.len() {
            sigf_n += amplitudes[i]
                * (theta_starts[i] + theta_incs[i] * n as f64).sin();
        }
        sig[n] = real_to_adc_sample(sigf_n);
    }

    sig
}

/// Reference clock timestamp values in `SAMPLE_BUFFER_SIZE` ADC
/// periods starting at `tstart`. Also returns the number of valid
/// timestamps.
///
/// # Arguments
///
/// * `fref` - External reference signal frequency (in Hz).
/// * `phi` - External reference signal phase shift (in radians).
/// * `tstart` - Start time in fast clock counts. This is the
/// start time of the current processing sequence (i.e., for the
/// current `SAMPLE_BUFFER_SIZE` ADC samples).
/// * `tstop` - Stop time in fast clock counts.
/// * `ffast` - Fast clock frequency (in Hz).
fn tstamps(
    fref: f64,
    phi: f64,
    tstart: u64,
    tstop: u64,
    ffast: f64,
) -> (usize, [u16; TSTAMP_BUFFER_SIZE]) {
    // counts in one reference period
    let tref = ffast / fref;
    let phi_counts = (phi / (2. * PI)) * tref;
    let start_counts = (tstart as f64 + phi_counts) % tref;
    let mut tval = (tref - start_counts) % tref;
    let tdist: f64 = (tstop - tstart) as f64;
    let mut r: usize = 0;
    let mut t: [u16; TSTAMP_BUFFER_SIZE] = [0; TSTAMP_BUFFER_SIZE];

    while tval < tdist {
        t[r] = tval as u16;
        tval += tref;
        r += 1;
    }

    (r, t)
}

/// Lowpass biquad filter using cutoff and sampling frequencies.
/// Taken from: https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
///
/// # Arguments
///
/// `fc` - Corner frequency, or 3dB cutoff frequency (in Hz).
/// `fs` - Sampling frequency (in Hz).
fn lp_iir_ba(fc: f64, fs: f64) -> [f32; 5] {
    let w0: f64 = 2. * PI * fc / fs;
    let q: f64 = 1. / 2f64.sqrt();
    let alpha: f64 = w0.sin() / (2. * q);
    let mut b0: f64 = (1. - w0.cos()) / 2.;
    let mut b1: f64 = 1. - w0.cos();
    let mut b2: f64 = b0;
    let a0: f64 = 1. + alpha;
    let mut a1: f64 = -2. * w0.cos();
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
/// See `max_error`.
///
/// * `res` - Result. This is compared with the actual value, `act`.
fn tol_check(act: f32, res: f32, fixed_tol: f32, rel_tol: f32) -> bool {
    (act - res).abs() < max_error(act, fixed_tol, rel_tol)
}

/// Maximum error from an actual value given fixed and relative
/// tolerances.
///
/// # Arguments
///
/// * `act` - Actual value with respect to which the magnitude of the
/// relative tolerance is computed.
/// * `fixed_tol` - Fixed tolerance.
/// * `rel_tol` - Relative tolerance. `rel_tol` * `act` gives the total
/// contribution of the relative tolerance.
fn max_error(act: f32, fixed_tol: f32, rel_tol: f32) -> f32 {
    rel_tol * act.abs() + fixed_tol
}

/// Lowpass filter test for magnitude and angle computation.
///
/// # Arguments
///
/// * `ffast` - Fast clock frequency (Hz). The fast clock increments
/// timestamp counter values used to record the edges of the external
/// reference.
/// * `fadc` - ADC sampling frequency (in Hz).
/// * `fref` - External reference frequency (in Hz).
/// * `fscale` - Scaling factor for the demodulation frequency. For
/// instance, 2 would demodulate with the first harmonic of the reference
/// frequency.
/// * `fc` - Lowpass filter 3dB cutoff frequency.
/// * `desired_input` - `PureSine` giving the frequency, strength and
/// phase of the desired result.
/// * `noise_inputs` - Vector of `PureSine` for any noise inputs on top
/// of `desired_input`.
/// * `tau_factor` - Number of time constants after which the output
/// is considered valid.
/// * `tol` - Acceptable relative tolerance the magnitude and angle
/// outputs. The outputs must remain within this tolerance between
/// `tau_factor` and `tau_factor+1` time constants.
fn lp_test(
    ffast: f64,
    fadc: f64,
    fref: f64,
    fscale: u32,
    fc: f64,
    desired_input: PureSine,
    noise_inputs: &mut Vec<PureSine>,
    tau_factor: f64,
    tol: f32,
) {
    let sample_counts: u64 = (ffast / fadc) as u64 * SAMPLE_BUFFER_SIZE as u64;
    let tadc: u32 = (ffast / fadc) as u32;

    let tau: f64 = 1. / (2. * PI * fc);
    let n_samples = (tau_factor * tau * fadc) as usize;
    // Ensure stability after `tau_factor` time constants.
    let extra_samples = (tau * fadc) as usize;

    let in_dbfs: f64 = desired_input.amp_dbfs;
    let in_a: f64 = dbfs_to_linear(in_dbfs);
    let in_phi: f64 = desired_input.phi;
    let i_act = in_a / 2. * in_phi.cos();
    let q_act = in_a / 2. * in_phi.sin();

    let mut in_noise_amp_total: f64 = 0.;

    for noise_input in noise_inputs.iter() {
        // Noise inputs create an oscillation at the output, where the
        // oscillation magnitude is determined by the strength of the
        // noise and its attenuation (attenuation is determined by its
        // proximity to the demodulation frequency and filter
        // rolloff).
        let octaves =
            ((noise_input.freq - (fref * fscale as f64)).abs() / fc).log2();
        // 2nd-order filter.
        let attenuation = -12. * octaves;
        let noise_lin = dbfs_to_linear(noise_input.amp_dbfs + attenuation);
        in_noise_amp_total += noise_lin;
    }

    let quantization_noise = {
        let fnyq: f64 = fadc / 2.;
        let lsb: f64 = 1. / ADC_MAX_COUNTS;
        // Total RMS noise is q / sqrt(12) distributed evenly from DC
        // to the Nyquist frequency.
        let total_rms: f64 = lsb / 12_f64.sqrt();
        let total_amp: f64 = total_rms * 2_f64.sqrt();
        // Limit noise to cutoff bandwidth.
        total_amp * (fc / fnyq).sqrt()
    };
    in_noise_amp_total += quantization_noise;

    let iq_noise: f64 = in_noise_amp_total;

    // TODO I'm not entirely sure about this.
    let in_a_noise: f64 = {
        let variant1: f64 = (2.
            * ((i_act + in_noise_amp_total).powf(2.)
                + (q_act + in_noise_amp_total).powf(2.))
            .sqrt()
            - in_a)
            .abs();
        let variant2: f64 = (2.
            * ((i_act - in_noise_amp_total).powf(2.)
                + (q_act - in_noise_amp_total).powf(2.))
            .sqrt()
            - in_a)
            .abs();
        variant1.max(variant2)
    };

    // TODO I'm not entirely sure about this.
    let in_phi_noise: f64 = {
        // Take the worst case as the noise at I being out of phase
        // with the noise at Q.
        let variant1: f64 = ((q_act + in_noise_amp_total)
            .atan2(i_act - in_noise_amp_total)
            - q_act.atan2(i_act))
        .abs();
        let variant2: f64 = ((q_act - in_noise_amp_total)
            .atan2(i_act + in_noise_amp_total)
            - q_act.atan2(i_act))
        .abs();
        variant1.max(variant2)
    };

    let pure_sigs = noise_inputs;
    pure_sigs.push(desired_input);

    let iir = IIR {
        ba: lp_iir_ba(fc, fadc),
        y_offset: 0.,
        y_min: -ADC_MAX_COUNTS as f32,
        y_max: (ADC_MAX_COUNTS - 1.) as f32,
    };

    let iirs: [IIR; 2] = [iir, iir];
    let mut iir_states: [IIRState; 2] = [[0.; 5], [0.; 5]];

    let iirs_iq: [IIR; 2] = [iir, iir];
    let mut iir_states_iq: [IIRState; 2] = [[0.; 5], [0.; 5]];

    let mut timestamps = [
        TimeStamp::new(),
        TimeStamp::new(),
    ];

    let mut timestamps_iq = [
        TimeStamp::new(),
        TimeStamp::new(),
    ];

    let in_a: f32 = in_a as f32;
    let in_phi: f32 = in_phi as f32;

    for n in 0..(n_samples + extra_samples) {
        let tstart: u64 = n as u64 * sample_counts;
        let sig: [i16; SAMPLE_BUFFER_SIZE] =
            input_signal(&pure_sigs, tstart, ffast, fadc);
        let (r, ts) =
            tstamps(fref, 0., tstart, tstart + sample_counts - 1, ffast);
        let (a, t) = postfilt_at(
            sig,
            ts,
            r,
            0,
            tadc,
            fscale,
            iirs,
            &mut iir_states,
            &mut timestamps,
        );

        let (i, q) = postfilt_iq(
            sig,
            ts,
            r,
            0,
            tadc,
            fscale,
            iirs_iq,
            &mut iir_states_iq,
            &mut timestamps_iq,
        );

        // Ensure stable below tolerance for 1 time constant after `tau_factor`.
        if n >= n_samples {
            for k in 0..OUTPUT_BUFFER_SIZE {
                let a_norm: f32 = a[k] / ADC_MAX_COUNTS as f32;
                assert!(
                    tol_check(in_a, a_norm, in_a_noise as f32, tol),
                    "a_act: {:.4} ({:.2} dBFS), a_meas: {:.4} ({:.2} dBFS), tol: {:.4}",
                    in_a,
                    in_dbfs,
                    a_norm,
                    linear_to_dbfs(a_norm as f64),
                    max_error(in_a, in_a_noise as f32, tol)
                );
                assert!(
                    tol_check(in_phi, t[k], in_phi_noise as f32, tol),
                    "t_act: {:.4}, t_meas: {:.4}, tol: {:.4}",
                    in_phi,
                    t[k],
                    max_error(in_phi, in_phi_noise as f32, tol)
                );

                let i_norm: f32 = i[k] / ADC_MAX_COUNTS as f32;
                let q_norm: f32 = q[k] / ADC_MAX_COUNTS as f32;
                assert!(
                    tol_check(i_act as f32, i_norm, iq_noise as f32, tol),
                    "i_act: {:.4}, i_meas: {:.4}, tol: {:.4}",
                    i_act,
                    i_norm,
                    max_error(i_act as f32, iq_noise as f32, tol)
                );
                assert!(
                    tol_check(q_act as f32, q_norm, iq_noise as f32, tol),
                    "i_act: {:.4}, i_meas: {:.4}, tol: {:.4}",
                    q_act,
                    q_norm,
                    max_error(q_act as f32, iq_noise as f32, tol)
                );
            }
        }
    }
}

#[test]
fn lp_fundamental_noise_phi_0() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 100e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.1 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.9 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_fundamental_noise_phi_pi_2() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 100e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: PI / 2.,
        },
        &mut vec![
            PureSine {
                freq: 1.1 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.9 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_fundamental_111e3_noise_phi_pi_4() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 111e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: PI / 4.,
        },
        &mut vec![
            PureSine {
                freq: 1.1 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.9 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_first_harmonic_noise() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 50e3;
    let fscale: u32 = 2;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_second_harmonic_noise() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 50e3;
    let fscale: u32 = 3;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_third_harmonic_noise() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 50e3;
    let fscale: u32 = 4;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_first_harmonic_phase_shift() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 50e3;
    let fscale: u32 = 2;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: PI / 4.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_fadc_1e6() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 1e6;
    let fsig: f64 = 100e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_ffast_125e6() {
    let ffast: f64 = 125e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 100e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![
            PureSine {
                freq: 1.2 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
            PureSine {
                freq: 0.8 * fdemod,
                amp_dbfs: -20.,
                phi: 0.,
            },
        ],
        tau,
        tol,
    )
}

#[test]
fn lp_low_t() {
    let ffast: f64 = 100e6;
    let fadc: f64 = 500e3;
    let fsig: f64 = 10e3;
    let fscale: u32 = 1;
    let fc: f64 = 1e3;
    let fdemod: f64 = fscale as f64 * fsig;
    let tau: f64 = 5.;
    let tol: f32 = 1e-2;

    lp_test(
        ffast,
        fadc,
        fsig,
        fscale,
        fc,
        PureSine {
            freq: fdemod,
            amp_dbfs: -30.,
            phi: 0.,
        },
        &mut vec![PureSine {
            freq: 2. * fdemod,
            amp_dbfs: -20.,
            phi: 0.,
        }],
        tau,
        tol,
    )
}
