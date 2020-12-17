#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "nightly", feature(asm, core_intrinsics))]

use core::ops::Neg;

pub type Complex<T> = (T, T);

/// Round up half.
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

pub mod iir;
pub mod lockin;
pub mod pll;
pub mod trig;
pub mod unwrap;

#[cfg(test)]
mod testing;
