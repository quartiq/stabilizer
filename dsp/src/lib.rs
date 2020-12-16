#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "nightly", feature(asm, core_intrinsics))]

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

pub mod atan2;
pub mod cossin;
pub mod iir;
pub mod lockin;
pub mod pll;
pub mod unwrap;

#[cfg(test)]
mod testing;
