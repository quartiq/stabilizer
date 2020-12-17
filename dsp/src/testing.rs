use super::Complex;

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
    let mut result: bool = true;
    a.iter().zip(b.iter()).for_each(|(i, j)| {
        result &= complex_isclose(*i, *j, rtol, atol);
    });
    result
}
