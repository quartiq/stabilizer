use core::mem::swap;
use super::{Complex, shift_round};

const PHASE_BITS: i32 = 20;
const LUT_DEPTH: i32 = 8;
const LUT_SIZE: usize = 1 << LUT_DEPTH as usize;
const OCTANT_BITS: i32 = 3;
const INTERPOLATION_BITS: i32 = PHASE_BITS - LUT_DEPTH - OCTANT_BITS;
static COSSIN_TABLE: [(u16, u16); LUT_SIZE] = include!("cossin_table.txt");

// Approximate pi/4 with an integer multiplier and right bit
// shift. The numerator is designed to saturate the i32 range.
const PI_4_NUMERATOR: i32 = 50;
const PI_4_RIGHT_SHIFT: i32 = 6;

/// Compute the cosine and sine of an angle.
///
/// # Arguments
///
/// `phase` - 20-bit fixed-point phase value.
///
/// # Returns
///
/// The cos and sin values of the provided phase as a `Complex<i32>`
/// value.
pub fn cossin(phase: i32) -> Complex<i32> {
    let mut phase = phase;
    let octant = (
        (phase & (1 << (PHASE_BITS - 1))) >> (PHASE_BITS - 1),
        (phase & (1 << (PHASE_BITS - 2))) >> (PHASE_BITS - 2),
        (phase & (1 << (PHASE_BITS - 3))) >> (PHASE_BITS - 3),
    );

    // Mask off octant bits. This leaves the angle in the range [0,
    // pi/4).
    phase &= (1 << (PHASE_BITS - OCTANT_BITS)) - 1;

    if octant.2 == 1 {
        // phase = pi/4 - phase
        phase = (1 << (INTERPOLATION_BITS + LUT_DEPTH)) - 1 - phase;
    }

    let interpolation: i32 = phase & ((1 << INTERPOLATION_BITS) - 1);

    phase >>= INTERPOLATION_BITS;

    let (mut cos, mut sin) = {
        let lookup = COSSIN_TABLE[phase as usize];
        (
            // 1/2 < cos(0<=x<=pi/4) <= 1. So, to spread out the cos
            // values and use the space more efficiently, we can
            // subtract 1/2 and multiply by 2. Therefore, we add 1
            // back in here. The sin values must be multiplied by 2 to
            // have the same scale as the cos values.
            lookup.0 as i32 + u16::MAX as i32,
            (lookup.1 as i32) << 1,
        )
    };

    // The phase values used for the LUT are adjusted up by half the
    // phase step. The interpolation must accurately reflect this. So,
    // an interpolation phase offset less than half the maximum
    // involves a negative phase offset. The rest us a non-negative
    // phase offset.
    let interpolation_factor =
        (interpolation - (1 << (INTERPOLATION_BITS - 1))) * PI_4_NUMERATOR;
    let dsin = shift_round(
        cos * interpolation_factor,
        LUT_DEPTH + INTERPOLATION_BITS + PI_4_RIGHT_SHIFT,
    );
    let dcos = shift_round(
        -sin * interpolation_factor,
        LUT_DEPTH + INTERPOLATION_BITS + PI_4_RIGHT_SHIFT,
    );

    cos += dcos;
    sin += dsin;

    if octant.1 ^ octant.2 == 1 {
        swap(&mut sin, &mut cos);
    }

    if octant.0 ^ octant.1 == 1 {
        cos *= -1;
    }

    if octant.0 == 1 {
        sin *= -1;
    }

    (cos, sin)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    #[test]
    fn error_max_rms_all_phase() {
        let max_amplitude: f64 = ((1 << 15) - 1) as f64;
        let mut rms_err: Complex<f64> = (0., 0.);

        for i in 0..(1 << PHASE_BITS) {
            let phase = i as i32;
            let radian_phase: f64 =
                2. * PI * (phase as f64 + 0.5) / ((1 << PHASE_BITS) as f64);

            let actual: Complex<f64> = (
                max_amplitude * radian_phase.cos(),
                max_amplitude * radian_phase.sin(),
            );
            let computed = cossin(phase);

            let err = (
                computed.0 as f64 / 4. - actual.0,
                computed.1 as f64 / 4. - actual.1,
            );
            rms_err.0 += err.0 * err.0 / (1 << PHASE_BITS) as f64;
            rms_err.1 += err.1 * err.1 / (1 << PHASE_BITS) as f64;

            assert!(err.0.abs() < 0.89);
            assert!(err.1.abs() < 0.89);
        }
        assert!(rms_err.0.sqrt() < 0.41);
        assert!(rms_err.1.sqrt() < 0.41);
    }
}
