use super::Complex;
use core::f64::consts::PI;

include!(concat!(env!("OUT_DIR"), "/cossin_table.rs"));

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

    (cos, sin)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn error_max_rms_all_phase() {
        // Constant amplitude error due to LUT data range.
        const AMPLITUDE: f64 = ((1i64 << 31) - (1i64 << 15)) as f64;
        const MAX_PHASE: f64 = (1i64 << 32) as f64;
        let mut rms_err: Complex<f64> = (0., 0.);
        let mut sum_err: Complex<f64> = (0., 0.);
        let mut max_err: Complex<f64> = (0., 0.);
        let mut sum: Complex<f64> = (0., 0.);
        let mut demod: Complex<f64> = (0., 0.);

        // use std::{fs::File, io::prelude::*, path::Path};
        // let mut file = File::create(Path::new("data.csv")).unwrap();

        const PHASE_DEPTH: usize = 20;

        for phase in 0..(1 << PHASE_DEPTH) {
            let phase = (phase << (32 - PHASE_DEPTH)) as i32;
            let have = cossin(phase);
            // writeln!(file, " {},{}", have.0, have.1).unwrap();

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
