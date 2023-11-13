use core::fmt::Write;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use stm32h7xx_hal::flash::{LockedFlashBank, UnlockedFlashBank};

#[derive(miniconf::Tree)]
pub struct Settings {
    pub broker: heapless::String<255>,
    pub id: heapless::String<23>,
}

impl Settings {
    fn new(mac: smoltcp_nal::smoltcp::wire::EthernetAddress) -> Self {
        let mut id = heapless::String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: "mqtt".into(),
            id,
        }
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
        let mut settings = Settings::new(mac);
        let mut buffer = [0u8; 256];
        let mut offset: usize = 0;

        // We iteratively read the settings from flash to allow for easy expansion of the settings
        // without losing data in the future when new fields are added.
        flash.read(offset as u32, &mut buffer[..]).unwrap();
        let len =
            buffer.iter().skip(1).position(|x| x == &b'"').unwrap_or(0) + 2;
        settings.broker = {
            match serde_json_core::from_slice(&buffer[..len]) {
                Ok((item, size)) => {
                    offset += size;
                    item
                }
                Err(e) => {
                    log::warn!("Failed to decode broker from flash settings memory - using default: {e:?}");
                    settings.broker
                }
            }
        };

        flash.read(offset as u32, &mut buffer[..]).unwrap();
        let len =
            buffer.iter().skip(1).position(|x| x == &b'"').unwrap_or(0) + 2;
        settings.id = {
            match serde_json_core::from_slice(&buffer[..len]) {
                Ok((item, size)) => {
                    item
                }
                Err(e) => {
                    log::warn!("Failed to MQTT ID from flash settings memory - using default: {e:?}");
                    settings.id
                }
            }
        };

        Self { flash, settings }
    }

    pub fn save(&mut self) {
        let mut bank = self.flash.unlocked();
        let mut data = [0; 512];
        let mut offset: usize = 0;
        offset += serde_json_core::to_slice(
            &self.settings.broker,
            &mut data[offset..],
        )
        .unwrap();
        offset +=
            serde_json_core::to_slice(&self.settings.id, &mut data[offset..])
                .unwrap();

        bank.erase(0, UnlockedFlashBank::ERASE_SIZE as u32).unwrap();
        bank.write(0, &data[..offset]).unwrap();
    }
}
