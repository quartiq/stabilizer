use core::fmt::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use stm32h7xx_hal::flash::{LockedFlashBank, UnlockedFlashBank};

#[derive(Clone, serde::Serialize, serde::Deserialize, miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
    #[serde(skip)]
    #[tree(skip)]
    pub mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
}

impl serial_settings::SerialSettings for Settings {
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
}

pub struct Flash(pub LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error = <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl embedded_storage::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize = stm32h7xx_hal::flash::UnlockedFlashBank::WRITE_SIZE;
    const ERASE_SIZE: usize = stm32h7xx_hal::flash::UnlockedFlashBank::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.erase(from, to)
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        let mut bank = self.0.unlocked();
        bank.write(offset, bytes)
    }
}

pub struct FlashSettings {
    flash: LockedFlashBank,
    pub settings: Settings,
}

impl FlashSettings {
    pub fn new(
        mut flash: LockedFlashBank,
        mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
    ) -> Self {
        let mut buffer = [0u8; 512];
        flash.read(0, &mut buffer[..]).unwrap();

        let settings = match postcard::from_bytes::<Settings>(&buffer) {
            Ok(mut settings) => {
                settings.mac = mac;
                settings
            }
            Err(_) => {
                log::warn!(
                    "Failed to load settings from flash. Using defaults"
                );
                Settings::new(mac)
            }
        };

        Self { flash, settings }
    }

    pub fn save(&mut self) {
        let mut bank = self.flash.unlocked();

        let mut data = [0; 512];
        let serialized = postcard::to_slice(&self.settings, &mut data).unwrap();
        bank.erase(0, UnlockedFlashBank::ERASE_SIZE as u32).unwrap();
        bank.write(0, serialized).unwrap();
    }
}
