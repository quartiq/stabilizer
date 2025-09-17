use core::fmt;
use serde::Serialize;

#[derive(Serialize)]
pub struct ApplicationMetadata {
    pub firmware_version: &'static str,
    pub rust_version: &'static str,
    pub profile: &'static str,
    pub git_dirty: bool,
    pub features: &'static str,
    pub panic_info: &'static str,
    pub hardware_version: &'static str,
}

impl fmt::Display for ApplicationMetadata {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_fmt(format_args!(
            "{:<20}: {} [{}]",
            "Version", self.firmware_version, self.profile,
        ))?;
        f.write_fmt(format_args!(
            "{:<20}: {}",
            "Hardware Revision", self.hardware_version
        ))?;
        f.write_fmt(format_args!(
            "{:<20}: {}",
            "Rustc Version", self.rust_version
        ))?;
        f.write_fmt(format_args!("{:<20}: {}", "Features", self.features))?;
        f.write_fmt(format_args!("{:<20}: {}", "Panic Info", self.panic_info))?;
        Ok(())
    }
}
