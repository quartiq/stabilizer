use super::Complex;

pub fn f32_is_close(a: f32, b: f32) -> bool {
    (a - b).abs() <= a.abs().max(b.abs()) * f32::EPSILON
}

pub fn complex_is_close(a: Complex<f32>, b: Complex<f32>) -> bool {
    f32_is_close(a.0, b.0) && f32_is_close(a.1, b.1)
}

pub fn complex_array_is_close(a: &[Complex<f32>], b: &[Complex<f32>]) -> bool {
    let mut result: bool = true;
    a.iter().zip(b.iter()).for_each(|(i, j)| {
        result &= complex_is_close(*i, *j);
    });
    result
}

pub fn within_tolerance(
    a: f32,
    b: f32,
    relative_tolerance: f32,
    fixed_tolerance: f32,
) -> bool {
    (a - b).abs() <= a.abs().max(b.abs()) * relative_tolerance + fixed_tolerance
}

pub fn complex_within_tolerance(
    a: Complex<f32>,
    b: Complex<f32>,
    relative_tolerance: f32,
    fixed_tolerance: f32,
) -> bool {
    within_tolerance(a.0, b.0, relative_tolerance, fixed_tolerance)
        && within_tolerance(a.1, b.1, relative_tolerance, fixed_tolerance)
}

pub fn complex_array_within_tolerance(
    a: &[Complex<f32>],
    b: &[Complex<f32>],
    relative_tolerance: f32,
    fixed_tolerance: f32,
) -> bool {
    let mut result: bool = true;
    a.iter().zip(b.iter()).for_each(|(i, j)| {
        result &= complex_within_tolerance(
            *i,
            *j,
            relative_tolerance,
            fixed_tolerance,
        );
    });
    result
}
