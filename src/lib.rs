#![no_std]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

pub mod design_parameters;

#[cfg(target_os = "none")]
pub mod hardware;

pub mod telemetry;

pub mod convert;

pub mod statistics;
