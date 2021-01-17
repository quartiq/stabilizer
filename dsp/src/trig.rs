use super::Complex;
use core::f64::consts::PI;

include!(concat!(env!("OUT_DIR"), "/cossin_table.rs"));

/// 2-argument arctangent function.
///
/// This implementation uses all integer arithmetic for fast
/// computation. It is designed to have high accuracy near the axes
/// and lower away from the axes. It is additionally designed so that
/// the error changes slowly with respect to the angle.
///
/// # Arguments
///
/// * `y` - Y-axis component.
/// * `x` - X-axis component.
///
/// # Returns
///
/// The angle between the x-axis and the ray to the point (x,y). The
/// result range is from i32::MIN to i32::MAX, where i32::MIN
/// represents -pi and, equivalently, +pi. i32::MAX represents one
/// count less than +pi.
pub fn atan2(y: i32, x: i32) -> i32 {
    let sign = (x < 0, y < 0);

    let mut y = y.wrapping_abs() as u32;
    let mut x = x.wrapping_abs() as u32;

    let y_greater = y > x;
    if y_greater {
        core::mem::swap(&mut y, &mut x);
    }

    let z = (16 - y.leading_zeros() as i32).max(0);

    x >>= z;
    if x == 0 {
        return 0;
    }
    y >>= z;
    let r = (y << 16) / x;
    debug_assert!(r <= 1 << 16);

    // Uses the general procedure described in the following
    // Mathematics stack exchange answer:
    //
    // https://math.stackexchange.com/a/1105038/583981
    //
    // The atan approximation method has been modified to be cheaper
    // to compute and to be more compatible with integer
    // arithmetic. The approximation technique used here is
    //
    // pi / 4 * r + C * r * (1 - abs(r))
    //
    // which is taken from Rajan 2006: Efficient Approximations for
    // the Arctangent Function.
    //
    // The least mean squared error solution is C = 0.279 (no the 0.285 that
    // Rajan uses). K = C*4/pi.
    // Q5 for K provides sufficient correction accuracy while preserving
    // as much smoothness of the quadratic correction as possible.
    const FP_K: usize = 5;
    const K: u32 = (0.35489 * (1 << FP_K) as f64) as u32;
    // debug_assert!(K == 11);

    // `r` is unsigned Q16.16 and <= 1
    // `angle` is signed Q1.31 with 1 << 31 == +- pi
    // Since K < 0.5 and r*(1 - r) <= 0.25 the correction product can use
    // 4 bits for K, and 15 bits for r and 1-r to remain within the u32 range.
    let mut angle = ((r << 13)
        + ((K * (r >> 1) * ((1 << 15) - (r >> 1))) >> (FP_K + 1)))
        as i32;

    if y_greater {
        angle = (1 << 30) - angle;
    }

    if sign.0 {
        angle = i32::MAX - angle;
    }

    if sign.1 {
        angle = angle.wrapping_neg();
    }

    angle
}

/// Compute the cosine and sine of an angle.
/// This is ported from the MiSoC cossin core.
/// (https://github.com/m-labs/misoc/blob/master/misoc/cores/cossin.py)
///
/// # Arguments
/// * `phase` - 32-bit phase.
///
/// # Returns
/// The cos and sin values of the provided phase as a `Complex<i32>`
/// value. With a 7-bit deep LUT there is 1e-5 max and 6e-8 RMS error
/// in each quadrature over 20 bit phase.
pub fn cossin(phase: i32) -> Complex<i32> {
    // Phase bits excluding the three highes MSB
    const OCTANT_BITS: usize = 32 - 3;

    // This is a slightly more compact way to compute the four flags for
    // octant mapping/unmapping used below.
    let mut octant = (phase as u32) >> OCTANT_BITS;
    octant ^= octant << 1;

    // Mask off octant bits. This leaves the angle in the range [0, pi/4).
    let mut phase = phase & ((1 << OCTANT_BITS) - 1);

    if octant & 1 != 0 {
        // phase = pi/4 - phase
        phase = (1 << OCTANT_BITS) - 1 - phase;
    }

    let lookup = COSSIN[(phase >> (OCTANT_BITS - COSSIN_DEPTH)) as usize];
    // 1/2 < cos(0 <= x <= pi/4) <= 1: Shift the cos
    // values and scale the sine values as encoded in the LUT.
    let mut cos = lookup.0 as i32 + u16::MAX as i32;
    let mut sin = (lookup.1 as i32) << 1;

    // 16 + 1 bits for cos/sin and 15 for dphi to saturate the i32 range.
    const ALIGN_MSB: usize = 32 - 16 - 1;
    phase >>= OCTANT_BITS - COSSIN_DEPTH - ALIGN_MSB;
    phase &= (1 << ALIGN_MSB) - 1;
    // The phase values used for the LUT are at midpoint for the truncated phase.
    // Interpolate relative to the LUT entry midpoint.
    phase -= (1 << (ALIGN_MSB - 1)) - (octant & 1) as i32;
    // Fixed point pi/4.
    const PI4: i32 = (PI / 4. * (1 << (32 - ALIGN_MSB)) as f64) as i32;
    // No rounding bias necessary here since we keep enough low bits.
    let dphi = (phase * PI4) >> (32 - ALIGN_MSB);

    // Make room for the sign bit.
    let dcos = (sin * dphi) >> (COSSIN_DEPTH + 1);
    let dsin = (cos * dphi) >> (COSSIN_DEPTH + 1);

    cos = (cos << (ALIGN_MSB - 1)) - dcos;
    sin = (sin << (ALIGN_MSB - 1)) + dsin;

    // Unmap using octant bits.
    if octant & 2 != 0 {
        core::mem::swap(&mut sin, &mut cos);
    }

    if octant & 4 != 0 {
        cos *= -1;
    }

    if octant & 8 != 0 {
        sin *= -1;
    }

    Complex(cos, sin)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    fn angle_to_axis(angle: f64) -> f64 {
        let angle = angle % (PI / 2.);
        (PI / 2. - angle).min(angle)
    }

    #[test]
    fn atan2_absolute_error() {
        const N: usize = 321;
        let mut test_vals = [0i32; N + 4];
        let scale = (1i64 << 31) as f64;
        for i in 0..N {
            test_vals[i] = (scale * (-1. + 2. * i as f64 / N as f64)) as i32;
        }

        assert!(test_vals.contains(&i32::MIN));
        test_vals[N] = i32::MAX;
        test_vals[N + 1] = 0;
        test_vals[N + 2] = -1;
        test_vals[N + 3] = 1;

        let mut rms_err = 0f64;
        let mut abs_err = 0f64;
        let mut rel_err = 0f64;

        for &x in test_vals.iter() {
            for &y in test_vals.iter() {
                let want = (y as f64 / scale).atan2(x as f64 / scale);
                let have = atan2(y, x) as f64 * PI / scale;

                let err = (have - want).abs();
                abs_err = abs_err.max(err);
                rms_err += err * err;
                if err > 3e-5 {
                    rel_err = rel_err.max(err / angle_to_axis(want));
                }
            }
        }
        rms_err = rms_err.sqrt() / test_vals.len() as f64;
        println!("max abs err: {:.2e}", abs_err);
        println!("rms abs err: {:.2e}", rms_err);
        println!("max rel err: {:.2e}", rel_err);
        assert!(abs_err < 5e-3);
        assert!(rms_err < 3e-3);
        assert!(rel_err < 0.6);
    }

    #[test]
    fn cossin_error_max_rms_all_phase() {
        // Constant amplitude error due to LUT data range.
        const AMPLITUDE: f64 = ((1i64 << 31) - (1i64 << 15)) as _;
        const MAX_PHASE: f64 = (1i64 << 32) as _;
        let mut rms_err = Complex(0., 0.);
        let mut sum_err = Complex(0., 0.);
        let mut max_err = Complex(0f64, 0.);
        let mut sum = Complex(0., 0.);
        let mut demod = Complex(0., 0.);

        // use std::{fs::File, io::{BufWriter, prelude::*}, path::Path};
        // let mut file = BufWriter::new(File::create(Path::new("data.bin")).unwrap());

        const PHASE_DEPTH: usize = 20;

        for phase in 0..(1 << PHASE_DEPTH) {
            let phase = (phase << (32 - PHASE_DEPTH)) as i32;
            let have = cossin(phase);
            // file.write(&have.0.to_le_bytes()).unwrap();
            // file.write(&have.1.to_le_bytes()).unwrap();

            let have = (have.0 as f64 / AMPLITUDE, have.1 as f64 / AMPLITUDE);

            let radian_phase = 2. * PI * phase as f64 / MAX_PHASE;
            let want = (radian_phase.cos(), radian_phase.sin());

            sum.0 += have.0;
            sum.1 += have.1;

            demod.0 += have.0 * want.0 - have.1 * want.1;
            demod.1 += have.1 * want.0 + have.0 * want.1;

            let err = (have.0 - want.0, have.1 - want.1);

            sum_err.0 += err.0;
            sum_err.1 += err.1;

            rms_err.0 += err.0 * err.0;
            rms_err.1 += err.1 * err.1;

            max_err.0 = max_err.0.max(err.0.abs());
            max_err.1 = max_err.1.max(err.1.abs());
        }
        rms_err.0 /= MAX_PHASE;
        rms_err.1 /= MAX_PHASE;

        println!("sum: {:.2e} {:.2e}", sum.0, sum.1);
        println!("demod: {:.2e} {:.2e}", demod.0, demod.1);
        println!("sum_err: {:.2e} {:.2e}", sum_err.0, sum_err.1);
        println!("rms: {:.2e} {:.2e}", rms_err.0.sqrt(), rms_err.1.sqrt());
        println!("max: {:.2e} {:.2e}", max_err.0, max_err.1);

        assert!(sum.0.abs() < 4e-10);
        assert!(sum.1.abs() < 4e-10);

        assert!(demod.0.abs() < 4e-10);
        assert!(demod.1.abs() < 4e-10);

        assert!(sum_err.0.abs() < 4e-10);
        assert!(sum_err.1.abs() < 4e-10);

        assert!(rms_err.0.sqrt() < 6e-8);
        assert!(rms_err.1.sqrt() < 6e-8);

        assert!(max_err.0 < 1.1e-5);
        assert!(max_err.1 < 1.1e-5);
    }
}
