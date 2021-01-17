#![allow(dead_code)]
use super::Complex;

/// Maximum acceptable error between a computed and actual value given fixed and relative
/// tolerances.
///
/// # Args
/// * `a` - First input.
/// * `b` - Second input. The relative tolerance is computed with respect to the maximum of the
/// absolute values of the first and second inputs.
/// * `rtol` - Relative tolerance.
/// * `atol` - Fixed tolerance.
///
/// # Returns
/// Maximum acceptable error.
pub fn max_error(a: f64, b: f64, rtol: f64, atol: f64) -> f64 {
    rtol * a.abs().max(b.abs()) + atol
}

pub fn isclose(a: f64, b: f64, rtol: f64, atol: f64) -> bool {
    (a - b).abs() <= a.abs().max(b.abs()) * rtol + atol
}

pub fn isclosef(a: f32, b: f32, rtol: f32, atol: f32) -> bool {
    (a - b).abs() <= a.abs().max(b.abs()) * rtol + atol
}

pub fn complex_isclose(
    a: Complex<f32>,
    b: Complex<f32>,
    rtol: f32,
    atol: f32,
) -> bool {
    isclosef(a.0, b.0, rtol, atol) && isclosef(a.1, b.1, rtol, atol)
}

pub fn complex_allclose(
    a: &[Complex<f32>],
    b: &[Complex<f32>],
    rtol: f32,
    atol: f32,
) -> bool {
    a.iter()
        .zip(b)
        .all(|(&i, &j)| complex_isclose(i, j, rtol, atol))
}
