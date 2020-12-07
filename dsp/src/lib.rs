#![cfg_attr(not(test), no_std)]
#![cfg_attr(feature = "nightly", feature(asm, core_intrinsics))]

pub type Complex<T> = (T, T);
pub mod iir;
pub mod lockin;
pub mod pll;
pub mod unwrap;

#[cfg(test)]
mod testing;
