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
use crate::hardware::{dfu, metadata::ApplicationMetadata};
use core::fmt::Write;
use embassy_futures::block_on;
use embedded_io::Write as EioWrite;
use embedded_storage_async::nor_flash::NorFlash;
use heapless::{String, Vec};
use miniconf::{
    postcard, Path, Tree, TreeDeserializeOwned, TreeSchema, TreeSerialize,
};
use sequential_storage::{
    cache::NoCache,
    map::{fetch_item, store_item, SerializationError},
};
use serial_settings::{BestEffortInterface, Platform, Settings};
use smoltcp_nal::smoltcp::wire::EthernetAddress;

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
            broker: String::try_from("mqtt").unwrap(),
            ip: String::try_from("0.0.0.0").unwrap(),
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
        Ok(::postcard::to_slice(self, buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?
            .len())
    }

    fn deserialize_from(
        buffer: &[u8],
    ) -> Result<(Self, usize), SerializationError> {
        let original_length = buffer.len();
        let (result, remainder) = ::postcard::take_from_bytes(buffer)
            .map_err(|_| SerializationError::BufferTooSmall)?;
        Ok((result, original_length - remainder.len()))
    }
}

pub struct SerialSettingsPlatform<C, F> {
    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: BestEffortInterface<crate::hardware::SerialPort>,

    pub _settings_marker: core::marker::PhantomData<C>,

    /// The storage mechanism used to persist settings to between boots.
    pub storage: F,

    /// Metadata associated with the application
    pub metadata: &'static ApplicationMetadata,
}

impl<C, F> SerialSettingsPlatform<C, F>
where
    C: TreeDeserializeOwned + TreeSerialize + TreeSchema,
    F: NorFlash,
{
    pub fn load(structure: &mut C, storage: &mut F) {
        // Loop over flash and read settings
        let mut buffer = [0u8; 512];
        for path in C::SCHEMA.nodes::<Path<String<128>, '/'>, 8>() {
            let path = path.unwrap();

            // Try to fetch the setting from flash.
            let value: &[u8] = match block_on(fetch_item(
                storage,
                0..storage.capacity() as _,
                &mut NoCache::new(),
                &mut buffer,
                &SettingsKey(path.clone().into_inner().into_bytes()),
            )) {
                Err(e) => {
                    log::warn!(
                        "Failed to fetch `{}` from flash: {e:?}",
                        path.0.as_str()
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

            log::info!("Loading initial `{}` from flash", path.0.as_str());

            let flavor = ::postcard::de_flavors::Slice::new(value);
            if let Err(e) = postcard::set_by_key(structure, &path, flavor) {
                log::warn!(
                    "Failed to deserialize `{}` from flash: {e:?}",
                    path.0.as_str()
                );
            }
        }
    }
}

impl<C, F> Platform for SerialSettingsPlatform<C, F>
where
    C: Settings,
    F: NorFlash,
{
    type Interface = BestEffortInterface<crate::hardware::SerialPort>;
    type Settings = C;
    type Error = sequential_storage::Error<F::Error>;

    fn fetch<'a>(
        &mut self,
        buf: &'a mut [u8],
        key: &[u8],
    ) -> Result<Option<&'a [u8]>, Self::Error> {
        let range = 0..self.storage.capacity() as _;
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
        let range = 0..self.storage.capacity() as _;
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
            "dfu" => dfu::start_dfu_reboot(),
            "service" => {
                write!(&mut self.interface, "{}", &self.metadata).unwrap();
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
