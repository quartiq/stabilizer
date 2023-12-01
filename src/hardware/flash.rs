use crate::hardware::platform;
use core::fmt::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use stm32h7xx_hal::flash::LockedFlashBank;

#[derive(Clone, serde::Serialize, serde::Deserialize, miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
    #[serde(skip)]
    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self::new(self.mac)
    }
}

impl Settings {
    pub fn reload(&mut self, storage: &mut Flash) {
        let mut buffer = [0u8; 512];
        storage.read(0, &mut buffer).unwrap();
        let Ok(mut settings) = postcard::from_bytes::<Self>(&buffer) else {
            return;
        };

        settings.mac = self.mac;
        *self = settings;
    }

    pub fn new(mac: smoltcp_nal::smoltcp::wire::EthernetAddress) -> Self {
        let mut id = heapless::String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".into(),
            id,
            mac,
        }
    }
}

pub struct Flash(pub LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error =
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl embedded_storage::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::WRITE_SIZE;
    const ERASE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.write(offset, bytes)
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
        usbd_serial::SerialPort<'static, super::UsbBus>,
    >,
    /// The Settings structure.
    pub settings: Settings,
    /// The storage mechanism used to persist settings to between boots.
    pub storage: Flash,
}

impl serial_settings::Platform for SerialSettingsPlatform {
    type Interface = serial_settings::BestEffortInterface<
        usbd_serial::SerialPort<'static, super::UsbBus>,
    >;
    type Settings = Settings;
    type Error =
        Error<<Flash as embedded_storage::nor_flash::ErrorType>::Error>;

    fn save(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        let serialized = postcard::to_slice(self.settings(), buf)?;
        self.storage
            .erase(0, serialized.len() as u32)
            .map_err(Self::Error::Flash)?;
        self.storage
            .write(0, serialized)
            .map_err(Self::Error::Flash)?;
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
