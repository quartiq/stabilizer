#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "nightly", feature(asm, core_intrinsics))]

use core::ops::{Add, Mul, Neg};

pub type Complex<T> = (T, T);

/// Bit shift, round up half.
///
/// # Arguments
///
/// `x` - Value to shift and round.
/// `shift` - Number of bits to right shift `x`.
///
/// # Returns
///
/// Shifted and rounded value.
#[inline(always)]
pub fn shift_round(x: i32, shift: usize) -> i32 {
    (x + (1 << (shift - 1))) >> shift
}

/// Integer division, round up half.
///
/// # Arguments
///
/// `dividend` - Value to divide.
/// `divisor` - Value that divides the
/// dividend. `dividend`+`divisor`-1 must be inside [i64::MIN,
/// i64::MAX].
///
/// # Returns
///
/// Divided and rounded value.
#[inline(always)]
pub fn divide_round(dividend: i64, divisor: i64) -> i64 {
    (dividend + (divisor - 1)) / divisor
}

fn abs<T>(x: T) -> T
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

fn copysign<T>(x: T, y: T) -> T
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
fn max<T>(x: T, y: T) -> T
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
fn min<T>(x: T, y: T) -> T
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
fn max(x: f32, y: f32) -> f32 {
    core::intrinsics::maxnumf32(x, y)
}

#[cfg(feature = "nightly")]
fn min(x: f32, y: f32) -> f32 {
    core::intrinsics::minnumf32(x, y)
}

// Multiply-accumulate vectors `x` and `a`.
//
// A.k.a. dot product.
// Rust/LLVM optimize this nicely.
fn macc<T>(y0: T, x: &[T], a: &[T]) -> T
where
    T: Add<Output = T> + Mul<Output = T> + Copy,
{
    x.iter()
        .zip(a)
        .map(|(x, a)| *x * *a)
        .fold(y0, |y, xa| y + xa)
}

pub mod iir;
pub mod iir_int;
pub mod lockin;
pub mod pll;
pub mod trig;
pub mod unwrap;

#[cfg(test)]
mod testing;
