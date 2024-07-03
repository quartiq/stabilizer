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
            broker: "mqtt".into(),
            ip: "0.0.0.0".into(),
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

pub fn load_from_flash<T: for<'d> JsonCoreSlash<'d, Y>, const Y: usize>(
    structure: &mut T,
    storage: &mut Flash,
) {
    // Loop over flash and read settings
    let mut buffer = [0u8; 512];
    for path in T::nodes::<Path<String<64>, '/'>>() {
        let (path, _node) = path.unwrap();

        // Try to fetch the setting from flash.
        let item: SettingsItem = match block_on(fetch_item(
            storage,
            storage.range(),
            &mut NoCache::new(),
            &mut buffer,
            SettingsKey(path.clone()),
        )) {
            Err(e) => {
                log::warn!(
                    "Failed to fetch `{}` from flash: {e:?}",
                    path.as_str()
                );
                continue;
            }
            Ok(Some(item)) => item,
            _ => continue,
        };

        // An empty vector may be saved to flash to "erase" a setting, since the H7 doesn't support
        // multi-write NOR flash. If we see an empty vector, ignore this entry.
        if item.0.is_empty() {
            continue;
        }

        log::info!("Loading initial `{}` from flash", path.as_str());

        let flavor = postcard::de_flavors::Slice::new(&item.0);
        if let Err(e) = structure.set_postcard_by_key(&path, flavor) {
            log::warn!(
                "Failed to deserialize `{}` from flash: {e:?}",
                path.as_str()
            );
        }
    }
}

#[derive(
    Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq,
)]
pub struct SettingsKey(Path<String<64>, '/'>);

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

#[derive(
    Default, serde::Serialize, serde::Deserialize, Clone, PartialEq, Eq,
)]
pub struct SettingsItem(Vec<u8, 256>);

impl<'a> sequential_storage::map::Value<'a> for SettingsItem {
    fn serialize_into(
        &self,
        buffer: &mut [u8],
    ) -> Result<usize, SerializationError> {
        if buffer.len() < self.0.len() {
            return Err(SerializationError::BufferTooSmall);
        }

        buffer[..self.0.len()].copy_from_slice(&self.0);
        Ok(self.0.len())
    }

    fn deserialize_from(buffer: &'a [u8]) -> Result<Self, SerializationError> {
        let vec = Vec::from_slice(buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?;
        Ok(Self(vec))
    }
}

#[derive(Debug)]
pub enum Error<F> {
    Postcard(postcard::Error),
    Flash(F),
}

impl<F> From<postcard::Error> for Error<F> {
    fn from(e: postcard::Error) -> Self {
        Self::Postcard(e)
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

impl<C, const Y: usize> Platform<Y> for SerialSettingsPlatform<C, Y>
where
    C: Settings<Y>,
{
    type Interface = BestEffortInterface<crate::hardware::SerialPort>;
    type Settings = C;
    type Error = Error<
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error,
    >;

    fn save(
        &mut self,
        buf: &mut [u8],
        key: Option<&str>,
        settings: &Self::Settings,
    ) -> Result<(), Self::Error> {
        let mut save_setting = |path| -> Result<(), Self::Error> {
            let path = SettingsKey(path);

            let mut data = Vec::new();
            data.resize(data.capacity(), 0).unwrap();
            let flavor = postcard::ser_flavors::Slice::new(&mut data);

            let len = match settings.get_postcard_by_key(&path.0, flavor) {
                Err(e) => {
                    log::warn!(
                        "Failed to save `{}` to flash: {e:?}",
                        path.0.as_str()
                    );
                    return Ok(());
                }
                Ok(slice) => slice.len(),
            };
            data.truncate(len);

            let range = self.storage.range();

            // Check if the settings has changed from what's currently in flash (or if it doesn't
            // yet exist).
            if block_on(fetch_item(
                &mut self.storage,
                range.clone(),
                &mut NoCache::new(),
                buf,
                path.clone(),
            ))
            .unwrap()
            .map(|old: SettingsItem| old.0 != data)
            .unwrap_or(true)
            {
                log::info!("Storing `{}` to flash", path.0.as_str());
                block_on(store_item(
                    &mut self.storage,
                    range,
                    &mut NoCache::new(),
                    buf,
                    path,
                    &SettingsItem(data),
                ))
                .unwrap();
            }

            Ok(())
        };

        if let Some(key) = key {
            save_setting(String::from(key).into())?;
        } else {
            for path in Self::Settings::nodes() {
                save_setting(path.unwrap().0)?;
            }
        }

        Ok(())
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

    fn clear(&mut self, buf: &mut [u8], key: Option<&str>) {
        let mut erase_setting = |path| -> Result<(), Self::Error> {
            let path = SettingsKey(path);
            let range = self.storage.range();

            // Check if there's an entry for this item in our flash map. The item might be a
            // sentinel value indicating "erased". Because we can't write flash memory twice, we
            // instead append a sentry "erased" value to the map where the serialized value is
            // empty.
            let maybe_item: Option<SettingsItem> = block_on(fetch_item(
                &mut self.storage,
                range.clone(),
                &mut NoCache::new(),
                buf,
                path.clone(),
            ))
            .unwrap();

            // An entry may exist in the map with no data as a sentinel that this path was
            // previously erased. If we find this, there's no need to store a duplicate "item is
            // erased" sentinel in flash. We only need to logically erase the path from the map if
            // it existed there in the first place.
            if matches!(maybe_item, Some(item) if !item.0.is_empty()) {
                block_on(store_item(
                    &mut self.storage,
                    range,
                    &mut NoCache::new(),
                    buf,
                    path,
                    &SettingsItem(Vec::new()),
                ))
                .unwrap();
            }

            Ok(())
        };

        if let Some(key) = key {
            erase_setting(String::from(key).into()).unwrap();
        } else {
            for path in Self::Settings::nodes() {
                erase_setting(path.unwrap().0).unwrap();
            }
        }
    }
}
