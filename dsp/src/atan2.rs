use super::{abs, shift_round};

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
/// corresponds to an angle of -pi and i32::MAX corresponds to an
/// angle of +pi.
pub fn atan2(y: i32, x: i32) -> i32 {
    let y = y >> 16;
    let x = x >> 16;

    let ux = abs::<i32>(x);
    let uy = abs::<i32>(y);

    // Uses the general procedure described in the following
    // Mathematics stack exchange answer:
    //
    // https://math.stackexchange.com/a/1105038/583981
    //
    // The atan approximation method has been modified to be cheaper
    // to compute and to be more compatible with integer
    // arithmetic. The approximation technique used here is
    //
    // pi / 4 * x + 0.285 * x * (1 - abs(x))
    //
    // which is taken from Rajan 2006: Efficient Approximations for
    // the Arctangent Function.
    let (min, max) = if ux < uy { (ux, uy) } else { (uy, ux) };

    if max == 0 {
        return 0;
    }

    let ratio = (min << 15) / max;

    let mut angle = {
        // pi/4, referenced to i16::MAX
        const PI_4_FACTOR: i32 = 25735;
        // 0.285, referenced to i16::MAX
        const FACTOR_0285: i32 = 9339;
        // 1/pi, referenced to u16::MAX
        const PI_INVERTED_FACTOR: i32 = 20861;

        let r1 = shift_round(ratio * PI_4_FACTOR, 15);
        let r2 = shift_round(
            (shift_round(ratio * FACTOR_0285, 15)) * (i16::MAX as i32 - ratio),
            15,
        );
        (r1 + r2) * PI_INVERTED_FACTOR
    };

    if uy > ux {
        angle = (i32::MAX >> 1) - angle;
    }

    if x < 0 {
        angle = i32::MAX - angle;
    }

    if y < 0 {
        angle *= -1;
    }

    angle
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;
    use crate::testing::isclose;

    fn angle_to_axis(angle: f64) -> f64 {
        let angle = angle % (PI / 2.);
        (PI / 2. - angle).min(angle)
    }

    #[test]
    fn absolute_error() {
        const NUM_VALS: usize = 1_001;
        let mut test_vals: [f64; NUM_VALS] = [0.; NUM_VALS];
        let val_bounds: (f64, f64) = (-1., 1.);
        let val_delta: f64 =
            (val_bounds.1 - val_bounds.0) / (NUM_VALS - 1) as f64;
        for i in 0..NUM_VALS {
            test_vals[i] = val_bounds.0 + i as f64 * val_delta;
        }

        for &x in test_vals.iter() {
            for &y in test_vals.iter() {
                let atol: f64 = 4e-5;
                let rtol: f64 = 0.127;
                let actual = (y.atan2(x) as f64 * i16::MAX as f64).round()
                    / i16::MAX as f64;
                let tol = atol + rtol * angle_to_axis(actual).abs();
                let computed = (atan2(
                    ((y * i16::MAX as f64) as i32) << 16,
                    ((x * i16::MAX as f64) as i32) << 16,
                ) >> 16) as f64
                    / i16::MAX as f64
                    * PI;

                if !isclose(computed, actual, 0., tol) {
                    println!("(x, y)   : {}, {}", x, y);
                    println!("actual   : {}", actual);
                    println!("computed : {}", computed);
                    println!("tolerance: {}\n", tol);
                    assert!(false);
                }
            }
        }
    }
}
