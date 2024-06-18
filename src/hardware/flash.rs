use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use stm32h7xx_hal::flash::LockedFlashBank;

use stm32h7xx_hal::flash::Error as FlashError;

pub struct Flash(pub LockedFlashBank);

impl Flash {
    pub fn range(&self) -> core::ops::Range<u32> {
        0..(self.0.len() as u32)
    }
}

impl ErrorType for Flash {
    type Error = FlashError;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), FlashError> {
        self.0.read(offset, bytes)
    }

    fn capacity(&self) -> usize {
        self.0.capacity()
    }
}

impl embedded_storage_async::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    async fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), FlashError> {
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

impl embedded_storage_async::nor_flash::NorFlash for Flash {
    const WRITE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::WRITE_SIZE;
    const ERASE_SIZE: usize =
        stm32h7xx_hal::flash::UnlockedFlashBank::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), FlashError> {
        let mut bank = self.0.unlocked();
        bank.erase(from, to)
    }

    async fn write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), FlashError> {
        let mut bank = self.0.unlocked();
        bank.write(offset, bytes)
    }
}
