#![no_std]

#[cfg(target_arch = "arm")]
mod dfu;
#[cfg(target_arch = "arm")]
pub use dfu::*;

#[cfg(target_arch = "arm")]
mod flash;
#[cfg(target_arch = "arm")]
pub use flash::*;

#[cfg(target_arch = "arm")]
mod settings;
#[cfg(target_arch = "arm")]
pub use settings::*;

mod metadata;
pub use metadata::*;

mod mqtt_app;
pub use mqtt_app::*;

mod telemetry;
pub use telemetry::*;
