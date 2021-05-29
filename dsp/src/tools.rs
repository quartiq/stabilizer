use core::ops::{Add, Mul, Neg};

pub fn abs<T>(x: T) -> T
where
    T: PartialOrd + Default + Neg<Output = T>,
{
    if x >= T::default() {
        x
    } else {
        -x
    }
}

// These are implemented here because core::f32 doesn't have them (yet).
// They are naive and don't handle inf/nan.
// `compiler-intrinsics`/llvm should have better (robust, universal, and
// faster) implementations.

pub fn copysign<T>(x: T, y: T) -> T
where
    T: PartialOrd + Default + Neg<Output = T>,
{
    if (x >= T::default() && y >= T::default())
        || (x <= T::default() && y <= T::default())
    {
        x
    } else {
        -x
    }
}

#[cfg(not(feature = "nightly"))]
pub fn max<T>(x: T, y: T) -> T
where
    T: PartialOrd,
{
    if x > y {
        x
    } else {
        y
    }
}

#[cfg(not(feature = "nightly"))]
pub fn min<T>(x: T, y: T) -> T
where
    T: PartialOrd,
{
    if x < y {
        x
    } else {
        y
    }
}

#[cfg(feature = "nightly")]
pub fn max(x: f32, y: f32) -> f32 {
    core::intrinsics::maxnumf32(x, y)
}

#[cfg(feature = "nightly")]
pub fn min(x: f32, y: f32) -> f32 {
    core::intrinsics::minnumf32(x, y)
}

// Multiply-accumulate vectors `x` and `a`.
//
// A.k.a. dot product.
// Rust/LLVM optimize this nicely.
pub fn macc<T>(y0: T, x: &[T], a: &[T]) -> T
where
    T: Add<Output = T> + Mul<Output = T> + Copy,
{
    x.iter()
        .zip(a)
        .map(|(x, a)| *x * *a)
        .fold(y0, |y, xa| y + xa)
}

pub fn macc_i32(y0: i32, x: &[i32], a: &[i32], shift: u32) -> i32 {
    // Rounding bias, half up
    let y0 = ((y0 as i64) << shift) + (1 << (shift - 1));
    let y = x
        .iter()
        .zip(a)
        .map(|(x, a)| *x as i64 * *a as i64)
        .fold(y0, |y, xa| y + xa);
    (y >> shift) as i32
}

/// Combine high and low i32 into a single downscaled i32, saturating the type.
pub fn saturating_scale(lo: i32, hi: i32, shift: u32) -> i32 {
    debug_assert!(shift & 31 == shift);
    let scale = -1 << shift;
    if hi <= scale {
        -i32::MAX
    } else if -hi <= scale {
        i32::MAX
    } else {
        (lo >> shift) + (hi << (31 - shift))
    }
}
