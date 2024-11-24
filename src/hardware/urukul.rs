use log;
use stm32h7xx_hal::{self as hal, prelude::*};

use arbitrary_int::{u2, u24, u3, u4, u7};
use bitbybit::{bitenum, bitfield};
use core::fmt::Debug;

pub struct Urukul {
    pub spi: hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled, u8>,
    pub cs: [hal::gpio::ErasedPin<hal::gpio::Output>; 3],
    pub io_update: hal::gpio::ErasedPin<hal::gpio::Output>,
    pub sync: hal::gpio::ErasedPin<hal::gpio::Output>,
}

#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum Error {
    InvalidProtoRev,
}

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

#[bitfield(u24, default = 0x000000)]
#[derive(Debug, PartialEq)]
pub struct Cfg {
    #[bits(0..=3, rw)]
    pub rf_sw: u4,
    #[bits(4..=7, rw)]
    pub led: u4,
    #[bits(8..=10, rw)]
    pub profile: u3,
    // 11: dummy
    #[bit(12, rw)]
    pub io_update: bool,
    #[bits(13..=16, rw)]
    pub mask_nu: u4,
    #[bits([17, 20], rw)]
    clk_sel: u2,
    #[bit(18, rw)]
    sync_sel: bool,
    #[bit(19, rw)]
    rst: bool,
    #[bits(23..=24, rw)]
    div: u2,
}

#[bitfield(u24)]
#[derive(Debug, PartialEq)]
pub struct Status {
    #[bits(0..=3, r)]
    pub rf_sw: u4,
    #[bits(4..=7, r)]
    pub smp_err: u4,
    #[bits(8..=11, r)]
    pub pll_lock: u4,
    #[bits(12..=15, r)]
    pub ifc_mode: u4,
    #[bits(16..=22, r)]
    pub proto_rev: u7,
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
        let cfg = Cfg::default();
        let mut cfg = cfg.raw_value().to_be_bytes();
        self.transfer(Select::Cfg, &mut cfg).unwrap();
        let sta = Status::new_with_raw_value(u24::from_be_bytes(cfg));
        if sta.proto_rev() != u7::new(0x8) {
            return Err(Error::InvalidProtoRev);
        }
        log::info!(
            "rf_sw={:4b} smp_err={:4b} pll_lock={:4b} ifc_mode={:4b}",
            sta.rf_sw(),
            sta.smp_err(),
            sta.pll_lock(),
            sta.ifc_mode()
        );
        let mut att = [0; 4];
        let attr = self.transfer(Select::Att, &mut att).unwrap();
        log::info!("{attr:?}");
        log::info!("Urukul CPLD initialization complete.");
        Ok(())
    }
}
