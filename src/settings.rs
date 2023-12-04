//! Stabilizer Settings Management
//!
//! # Design
//! Stabilizer supports two types of settings:
//! 1. Static Device Configuration
//! 2. Dynamic Run-time Settings
//!
//! Static device configuration settings are loaded and used only at device power-up. These include
//! things like the MQTT broker address and the MQTT identified. Conversely, the dynamic run-time
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
use crate::hardware::flash::Flash;
use crate::hardware::platform;
use core::fmt::Write;
use miniconf::{TreeDeserialize, TreeKey, TreeSerialize};
use postcard::ser_flavors::Flavor;
use stm32h7xx_hal::flash::LockedFlashBank;

#[derive(Clone, miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self::new(self.mac)
    }
}

impl Settings {
    pub fn new(mac: smoltcp_nal::smoltcp::wire::EthernetAddress) -> Self {
        let mut id = heapless::String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".into(),
            id,
            mac,
        }
    }

    pub fn reload(&mut self, storage: &mut Flash) {
        // Loop over flash and read settings
        let mut buffer = [0u8; 512];
        for path in Settings::iter_paths::<heapless::String<32>>("/") {
            let path = path.unwrap();

            // Try to fetch the setting from flash.
            let Some(item) =
                sequential_storage::map::fetch_item::<SettingsItem, _>(
                    storage,
                    storage.range(),
                    &mut buffer,
                    path.clone(),
                )
                .unwrap()
            else {
                continue;
            };

            log::info!("Found `{path}` in flash settings");

            let mut deserializer = postcard::Deserializer::from_flavor(
                postcard::de_flavors::Slice::new(&item.data),
            );
            if let Err(e) = self
                .deserialize_by_key(path.split('/').skip(1), &mut deserializer)
            {
                log::warn!("Failed to load {path} from flash settings: {e:?}");
            }
        }
    }
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct SettingsItem {
    // We only make these owned vec/string to get around lifetime limitations.
    pub path: heapless::String<32>,
    pub data: heapless::Vec<u8, 256>,
}

impl sequential_storage::map::StorageItem for SettingsItem {
    type Key = heapless::String<32>;
    type Error = postcard::Error;

    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, Self::Error> {
        Ok(postcard::to_slice(self, buffer)?.len())
    }

    fn deserialize_from(buffer: &[u8]) -> Result<Self, Self::Error> {
        postcard::from_bytes(buffer)
    }

    fn key(&self) -> Self::Key {
        self.path.clone()
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

pub struct SerialSettingsPlatform {
    /// The interface to read/write data to/from serially (via text) to the user.
    pub interface: serial_settings::BestEffortInterface<
        usbd_serial::SerialPort<'static, crate::hardware::UsbBus>,
    >,
    /// The Settings structure.
    pub settings: Settings,
    /// The storage mechanism used to persist settings to between boots.
    pub storage: Flash,
}

impl serial_settings::Platform for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<
        usbd_serial::SerialPort<'static, crate::hardware::UsbBus>,
    >;
    type Settings = Settings;
    type Error = Error<
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error,
    >;

    fn save(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        for path in Settings::iter_paths::<heapless::String<32>>("/") {
            let path = path.unwrap();
            let range = self.storage.range();

            let (buf1, buf2) = buf.split_at_mut(buf.len() / 2);
            let mut serializer = postcard::Serializer {
                output: postcard::ser_flavors::Slice::new(buf1),
            };
            if let Err(e) = self
                .settings
                .serialize_by_key(path.split('/').skip(1), &mut serializer)
            {
                log::warn!("Failed to save {path} to flash: {e:?}");
                continue;
            }

            let serialized_setting = heapless::Vec::from_slice(
                serializer.output.finalize().unwrap(),
            )
            .unwrap();

            // Check if the settings has changed from what's currently in flash (or if it doesn't
            // yet exist).
            let update = if let Some(item) =
                sequential_storage::map::fetch_item::<SettingsItem, _>(
                    &mut self.storage,
                    range.clone(),
                    buf2,
                    path.clone(),
                )
                .unwrap()
            {
                let changed = item.data != serialized_setting;
                if changed {
                    log::info!(
                        "{path} yet exists in flash, but has changed. Updating"
                    );
                }

                changed
            } else {
                log::info!("{path} does not yet exist in flash. Setting it");
                true
            };

            // If the value needs to be rewritten to flash, update it now.
            if update {
                let item = SettingsItem {
                    data: serialized_setting,
                    path,
                };

                sequential_storage::map::store_item(
                    &mut self.storage,
                    range,
                    buf2,
                    item,
                )
                .unwrap();
            }
        }

        Ok(())
    }

    fn cmd(&mut self, cmd: &str) {
        match cmd {
            "reboot" => cortex_m::peripheral::SCB::sys_reset(),
            "dfu" => platform::start_dfu_reboot(),
            _ => {
                writeln!(
                    self.interface_mut(),
                    "Invalid platform command: `{cmd}` not in [`dfu`, `reboot`]"
                )
                .ok();
            }
        }
    }

    fn settings(&self) -> &Self::Settings {
        &self.settings
    }

    fn settings_mut(&mut self) -> &mut Self::Settings {
        &mut self.settings
    }

    fn interface_mut(&mut self) -> &mut Self::Interface {
        &mut self.interface
    }
}
