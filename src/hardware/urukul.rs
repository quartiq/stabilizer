use log;
use stm32h7xx_hal::{self as hal, prelude::*};

pub struct Urukul {
    pub spi: hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled, u8>,
    pub cs: [hal::gpio::ErasedPin<hal::gpio::Output>; 3],
    pub io_update: hal::gpio::ErasedPin<hal::gpio::Output>,
    pub sync: hal::gpio::ErasedPin<hal::gpio::Output>,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum Error {}

#[repr(u8)]
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd)]
pub enum Select {
    None = 0,
    Cfg = 1,
    Att = 2,
    Multi = 3,
    Dds0 = 4,
    Dds1 = 5,
    Dds2 = 6,
    Dds3 = 7,
}

impl Urukul {
    fn select(&mut self, s: Select) {
        for (i, cs) in self.cs.iter_mut().enumerate() {
            cs.set_state(((s as u8 >> i) & 1 == 1).into());
        }
    }

    fn selected<F: FnOnce(&mut Self) -> Result<O, E>, O, E>(
        &mut self,
        select: Select,
        func: F,
    ) -> Result<O, E> {
        self.select(select);
        let ret = func(self);
        self.select(Select::None);
        ret
    }

    fn transfer<'a>(
        &mut self,
        select: Select,
        write: &'a mut [u8],
    ) -> Result<&'a [u8], hal::spi::Error> {
        self.selected(select, |dev| dev.spi.transfer(write))
    }

    pub fn init(&mut self) -> Result<(), Error> {
        let mut cfg = [0; 3];
        let sta = self.transfer(Select::Cfg, &mut cfg).unwrap();
        log::info!("{sta:?}");
        let mut att = [0; 4];
        let attr = self.transfer(Select::Att, &mut att).unwrap();
        log::info!("{attr:?}");
        Ok(())
    }
}
