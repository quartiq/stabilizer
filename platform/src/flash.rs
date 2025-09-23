use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};

pub struct AsyncFlash<T>(pub T);

pub trait UnlockFlash: ReadNorFlash {
    type Unlocked<'a>: NorFlash<Error = Self::Error>
    where
        Self: 'a;
    fn unlock(&mut self) -> Self::Unlocked<'_>;
}

impl<T: ReadNorFlash> ErrorType for AsyncFlash<T> {
    type Error = T::Error;
}

impl<T: ReadNorFlash> embedded_storage_async::nor_flash::ReadNorFlash
    for AsyncFlash<T>
{
    const READ_SIZE: usize = T::READ_SIZE;

    async fn read(
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

impl<T: UnlockFlash> embedded_storage_async::nor_flash::NorFlash
    for AsyncFlash<T>
{
    const WRITE_SIZE: usize = T::Unlocked::WRITE_SIZE;
    const ERASE_SIZE: usize = T::Unlocked::ERASE_SIZE;

    async fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.0.unlock().erase(from, to)
    }

    async fn write(
        &mut self,
        offset: u32,
        bytes: &[u8],
    ) -> Result<(), Self::Error> {
        self.0.unlock().write(offset, bytes)
    }
}
