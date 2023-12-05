use crate::hardware::HardwareVersion;
use serde::Serialize;

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[derive(Serialize)]
pub struct ApplicationMetadata {
    pub firmware_version: &'static str,
    pub rust_version: &'static str,
    pub profile: &'static str,
    pub git_dirty: bool,
    pub features: &'static str,
    pub panic_info: &'static str,
    pub hardware_version: HardwareVersion,
}

impl ApplicationMetadata {
    /// Construct the global metadata.
    ///
    /// # Note
    /// This may only be called once.
    ///
    /// # Args
    /// * `hardware_version` - The hardware version detected.
    /// * `phy` - The identifier of the detected ethernet PHY.
    ///
    /// # Returns
    /// A reference to the global metadata.
    pub fn new(version: HardwareVersion) -> &'static ApplicationMetadata {
        cortex_m::singleton!(: ApplicationMetadata = ApplicationMetadata {
            firmware_version: build_info::GIT_VERSION.unwrap_or("Unspecified"),
            rust_version: build_info::RUSTC_VERSION,
            profile: build_info::PROFILE,
            git_dirty: build_info::GIT_DIRTY.unwrap_or(false),
            features: build_info::FEATURES_STR,
            hardware_version: version,
            panic_info: panic_persist::get_panic_message_utf8().unwrap_or("None"),
        }).unwrap()
    }
}
