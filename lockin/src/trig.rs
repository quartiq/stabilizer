// `force_eval` is taken from libm/src/math/mod.rs.
macro_rules! force_eval {
    ($e:expr) => {
        unsafe {
            ::core::ptr::read_volatile(&$e);
        }
    };
}

#[path = "sinf.rs"]
mod sinf;
#[path = "k_sinf.rs"]
mod k_sinf;
#[path = "cosf.rs"]
mod cosf;
#[path = "k_cosf.rs"]
mod k_cosf;
#[path = "rem_pio2f.rs"]
mod rem_pio2f;

use sinf::sinf;
use cosf::cosf;

/// Computes the sine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which sine is computed. Must be in range
/// [0,2*PI).
pub fn sin(theta: f32) -> f32 {
    return sinf(theta);
}

/// Computes the cosine of a value `theta` in the range [0, 2*PI).
///
/// # Arguments
///
/// * `theta` - Angle for which cosine is computed. Must be in range
/// [0,2*PI).
pub fn cos(theta: f32) -> f32 {
    return cosf(theta);
}

/// Computes the four quadrant arctangent.
///
/// # Arguments
///
/// * `y` - Y coordinate value. Can be any real value.
/// * `x` - X coordinate value. Can be any real value.
pub fn atan2(y: f32, x: f32) -> f32 {
    return libm::atan2f(y, x);
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::f32::consts::PI;

    #[test]
    fn sin_tol_1en6() {
        let n = 100_000;
        let delta = 2. * std::f64::consts::PI / n as f64;
        let tol: f32 = 1e-6;

        for i in 0..n {
            let theta = (i as f64 * delta) as f32;
            let res = sin(theta);
            let act = theta.sin();
            assert!((res - act).abs() < tol, "theta: {}, res: {}, act: {}", theta, res, act);
        }

        // explicitly test quadrant boundaries
        let bounds: [f32; 4] = [0., PI / 2., PI, 3. * PI / 2.];
        for theta in bounds.iter() {
            let res = sin(*theta);
            let act = theta.sin();
            assert!((res - act).abs() < tol, "theta: {}, res: {}, act: {}", theta, res, act);
        }
    }

    #[test]
    fn cos_tol_1en6() {
        let n = 100_000;
        let delta = 2. * std::f64::consts::PI / n as f64;
        let tol: f32 = 1e-6;

        for i in 0..n {
            let theta = (i as f64 * delta) as f32;
            let res = cos(theta);
            let act = theta.cos();
            assert!((res - act).abs() < tol, "theta: {}, res: {}, act: {}", theta, res, act);
        }

        let bounds: [f32; 4] = [0., PI / 2., PI, 3. * PI / 2.];
        for theta in bounds.iter() {
            let res = cos(*theta);
            let act = theta.cos();
            assert!((res - act).abs() < tol, "theta: {}, res: {}, act: {}", theta, res, act);
        }
    }

    #[test]
    fn atan2_tol_1en6_range_100() {
        const N: usize = 1_000;
        let range: f64 = 100.;
        let delta = range / N as f64;
        let val_start = -range / 2.;
        let tol: f32 = 1e-6;

        let mut vals = [0f32; N];
        for i in 0..N {
            vals[i] = (val_start + i as f64 * delta) as f32;
        }

        for x in vals.iter() {
            for y in vals.iter() {
                let res = atan2(*y, *x);
                let act = y.atan2(*x);
                assert!((res - act).abs() < tol, "x: {}, y: {}, res: {}, act: {}", *x, *y, res, act);
            }
        }
    }

    #[test]
    fn atan2_tol_1en6_range_1() {
        const N: usize = 1_000;
        let range: f64 = 1.;
        let delta = range / N as f64;
        let val_start = -range / 2.;
        let tol: f32 = 1e-6;

        let mut vals = [0f32; N];
        for i in 0..N {
            vals[i] = (val_start + i as f64 * delta) as f32;
        }

        for x in vals.iter() {
            for y in vals.iter() {
                let res = atan2(*y, *x);
                let act = y.atan2(*x);
                assert!((res - act).abs() < tol, "x: {}, y: {}, res: {}, act: {}", *x, *y, res, act);
            }
        }
    }
}
