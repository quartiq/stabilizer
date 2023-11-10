use embedded_storage::nor_flash::NorFlash;
use stm32h7xx_hal::flash::{LockedFlashBank, UnlockedFlashBank};
#[derive(Debug)]
pub enum StorageError {
    JsonDe(serde_json_core::de::Error),
    JsonSer(serde_json_core::ser::Error),
}

impl From<serde_json_core::de::Error> for StorageError {
    fn from(e: serde_json_core::de::Error) -> Self {
        Self::JsonDe(e)
    }
}

impl From<serde_json_core::ser::Error> for StorageError {
    fn from(e: serde_json_core::ser::Error) -> Self {
        Self::JsonSer(e)
    }
}

impl sequential_storage::map::StorageItemError for StorageError {
    fn is_buffer_too_small(&self) -> bool {
        match self {
            Self::JsonSer(serde_json_core::ser::Error::BufferFull) => true,
            Self::JsonDe(serde_json_core::de::Error::EofWhileParsingString) => {
                true
            }
            Self::JsonDe(serde_json_core::de::Error::EofWhileParsingList) => {
                true
            }
            Self::JsonDe(serde_json_core::de::Error::EofWhileParsingObject) => {
                true
            }
            Self::JsonDe(serde_json_core::de::Error::EofWhileParsingNumber) => {
                true
            }
            Self::JsonDe(serde_json_core::de::Error::EofWhileParsingValue) => {
                true
            }
            _ => false,
        }
    }
}

macro_rules! storage_item {
    ($len:literal, $key:literal, $name:ident) => {
        pub struct $name(pub heapless::String<$len>);

        impl sequential_storage::map::StorageItem for $name {
            type Key = &'static str;
            type Error = StorageError;

            fn serialize_into(
                &self,
                buffer: &mut [u8],
            ) -> Result<usize, Self::Error> {
                Ok(serde_json_core::to_slice(&self.0, buffer)?)
            }

            fn deserialize_from(
                buffer: &[u8],
            ) -> Result<(Self, usize), Self::Error> {
                Ok(serde_json_core::from_slice(buffer)
                    .map(|(item, size)| (Self(item), size))?)
            }

            fn key(&self) -> Self::Key {
                $key
            }
        }
    };
}

storage_item!(23, "id", MqttIdentifier);
storage_item!(255, "broker", BrokerAddress);

pub struct FlashSettings {
    flash: LockedFlashBank,
}

impl FlashSettings {
    pub fn new(flash: LockedFlashBank) -> Self {
        Self { flash }
    }

    pub fn store_item(
        &mut self,
        item: impl sequential_storage::map::StorageItem,
    ) {
        let mut bank = self.flash.unlocked();
        let range =
            (bank.address() as u32)..((bank.address() + bank.len()) as u32);
        sequential_storage::map::store_item::<
            _,
            _,
            { <UnlockedFlashBank as NorFlash>::ERASE_SIZE },
        >(&mut bank, range, item)
        .unwrap();
    }

    pub fn fetch_item<
        I: sequential_storage::map::StorageItem<Key = &'static str>,
    >(
        &mut self,
        key: &'static str,
    ) -> Option<I> {
        let mut bank = self.flash.unlocked();
        let range =
            (bank.address() as u32)..((bank.address() + bank.len()) as u32);
        sequential_storage::map::fetch_item(&mut bank, range, key).unwrap()
    }
}
