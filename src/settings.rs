//! Stabilizer Settings Management
//!
//! # Design
//! Stabilizer supports two types of settings:
//! 1. Static Device Configuration
//! 2. Dynamic Run-time Settings
//!
//! Static device configuration settings are loaded and used only at device power-up. These include
//! things like the MQTT broker address and the MQTT identifier. Conversely, the dynamic run-time
//! settings can be changed and take effect immediately during device operation.
//!
//! This settings management interface is currently targeted at the static device configuration
//! settings. Settings are persisted into the unused 1MB flash bank of Stabilizer for future
//! recall. They can be modified via the USB interface to facilitate device configuration.
//!
//! Settings are stored in flash using a key-value pair mapping, where the `key` is the name of the
//! entry in the settings structure. This has a number of benefits:
//! 1. The `Settings` structure can have new entries added to it in the future without losing old
//!    settings values, as each entry of the `Settings` struct is stored separately as its own
//!    key-value pair.
//! 2. The `Settings` can be used among multiple Stabilizer firmware versions that need the same
//!    settings values
//! 3. Unknown/unneeded settings values in flash can be actively ignored, facilitating simple flash
//!    storage sharing.
use crate::hardware::{flash::Flash, metadata::ApplicationMetadata, platform};
use core::fmt::Write;
use embassy_futures::block_on;
use embedded_io::Write as EioWrite;
use heapless::{String, Vec};
use miniconf::{JsonCoreSlash, Path, Postcard, Tree};
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item, SerializationError},
};
use serial_settings::{BestEffortInterface, Platform, Settings};
use smoltcp_nal::smoltcp::wire::EthernetAddress;
use stm32h7xx_hal::flash::LockedFlashBank;

/// Settings that are used for configuring the network interface to Stabilizer.
#[derive(Clone, Debug, Tree)]
pub struct NetSettings {
    /// The broker domain name (or IP address) to use for MQTT connections.
    pub broker: String<255>,

    /// The MQTT ID to use upon connection with a broker.
    pub id: String<23>,

    /// An optional static IP address to use. An unspecified IP address (or malformed address) will
    /// use DHCP.
    pub ip: String<15>,
    #[tree(skip)]
    /// The MAC address of Stabilizer, which is used to reinitialize the ID to default settings.
    pub mac: EthernetAddress,
}

impl NetSettings {
    pub fn new(mac: EthernetAddress) -> Self {
        let mut id = String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".try_into().unwrap(),
            ip: "0.0.0.0".try_into().unwrap(),
            id,
            mac,
        }
    }
}

pub trait AppSettings {
    /// Construct the settings given known network settings.
    fn new(net: NetSettings) -> Self;

    /// Get the network settings from the application settings.
    fn net(&self) -> &NetSettings;
}

#[derive(
    Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq,
)]
pub struct SettingsKey(Vec<u8, 128>);

impl sequential_storage::map::Key for SettingsKey {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, SerializationError> {
        Ok(postcard::to_slice(self, buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?
            .len())
    }

    fn deserialize_from(
        buffer: &[u8],
    ) -> Result<(Self, usize), SerializationError> {
        let original_length = buffer.len();
        let (result, remainder) = postcard::take_from_bytes(buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?;
        Ok((result, original_length - remainder.len()))
    }
}

pub struct SerialSettingsPlatform<C, const Y: usize> {
    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: BestEffortInterface<crate::hardware::SerialPort>,

    pub _settings_marker: core::marker::PhantomData<C>,

    /// The storage mechanism used to persist settings to between boots.
    pub storage: Flash,

    /// Metadata associated with the application
    pub metadata: &'static ApplicationMetadata,
}

impl<C, const Y: usize> SerialSettingsPlatform<C, Y>
where
    C: for<'d> JsonCoreSlash<'d, Y>,
{
    pub fn load(structure: &mut C, storage: &mut Flash) {
        // Loop over flash and read settings
        let mut buffer = [0u8; 512];
        for path in C::nodes::<Path<String<128>, '/'>>() {
            let (path, _node) = path.unwrap();

            // Try to fetch the setting from flash.
            let value: &[u8] = match block_on(fetch_item(
                storage,
                storage.range(),
                &mut NoCache::new(),
                &mut buffer,
                &SettingsKey(Vec::from_slice(path.as_bytes()).unwrap()),
            )) {
                Err(e) => {
                    log::warn!(
                        "Failed to fetch `{}` from flash: {e:?}",
                        path.as_str()
                    );
                    continue;
                }
                Ok(Some(value)) => value,
                Ok(None) => continue,
            };

            // An empty vector may be saved to flash to "erase" a setting, since the H7 doesn't support
            // multi-write NOR flash. If we see an empty vector, ignore this entry.
            if value.is_empty() {
                continue;
            }

            log::info!("Loading initial `{}` from flash", path.as_str());

            let flavor = postcard::de_flavors::Slice::new(value);
            if let Err(e) = structure.set_postcard_by_key(&path, flavor) {
                log::warn!(
                    "Failed to deserialize `{}` from flash: {e:?}",
                    path.as_str()
                );
            }
        }
    }
}

impl<C, const Y: usize> Platform<Y> for SerialSettingsPlatform<C, Y>
where
    C: Settings<Y>,
{
    type Interface = BestEffortInterface<crate::hardware::SerialPort>;
    type Settings = C;
    type Error = sequential_storage::Error<
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error,
    >;

    fn fetch<'a>(
        &mut self,
        buf: &'a mut [u8],
        key: &[u8],
    ) -> Result<Option<&'a [u8]>, Self::Error> {
        let range = self.storage.range();
        block_on(fetch_item(
            &mut self.storage,
            range,
            &mut NoCache::new(),
            buf,
            &SettingsKey(Vec::try_from(key).unwrap()),
        ))
        .map(|v| v.filter(|v: &&[u8]| !v.is_empty()))
    }

    fn store(
        &mut self,
        buf: &mut [u8],
        key: &[u8],
        value: &[u8],
    ) -> Result<(), Self::Error> {
        let range = self.storage.range();
        block_on(store_item(
            &mut self.storage,
            range,
            &mut NoCache::new(),
            buf,
            &SettingsKey(Vec::try_from(key).unwrap()),
            &value,
        ))
    }

    fn clear(&mut self, buf: &mut [u8], key: &[u8]) -> Result<(), Self::Error> {
        self.store(buf, key, b"")
    }

    fn cmd(&mut self, cmd: &str) {
        match cmd {
            "reboot" => cortex_m::peripheral::SCB::sys_reset(),
            "dfu" => platform::start_dfu_reboot(),
            "service" => {
                writeln!(
                    &mut self.interface,
                    "{:<20}: {} [{}]",
                    "Version",
                    self.metadata.firmware_version,
                    self.metadata.profile,
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Hardware Revision", self.metadata.hardware_version
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Rustc Version", self.metadata.rust_version
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Features", self.metadata.features
                )
                .unwrap();
                writeln!(
                    &mut self.interface,
                    "{:<20}: {}",
                    "Panic Info", self.metadata.panic_info
                )
                .unwrap();
            }
            _ => {
                writeln!(
                    self.interface_mut(),
                    "Invalid platform command: `{cmd}` not in [`dfu`, `reboot`, `service`]"
                )
                .ok();
            }
        }
    }

    fn interface_mut(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
