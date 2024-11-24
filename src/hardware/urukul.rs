use log;
use stm32h7xx_hal::{self as hal, prelude::*};

use arbitrary_int::{u2, u24, u3, u4, u7};
use bitbybit::bitfield;
use core::fmt::Debug;

pub struct Urukul {
    spi: hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled, u8>,
    cs: [hal::gpio::ErasedPin<hal::gpio::Output>; 3],
    io_update: hal::gpio::ErasedPin<hal::gpio::Output>,
    sync: hal::gpio::ErasedPin<hal::gpio::Output>,
    cfg: Cfg,
    att: [u8; 4],
}

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("Invalid PROTO_REV {0}")]
    InvalidProtoRev(u7),
    #[error("RF_SW is driven {0}")]
    DrivenRfSw(u4),
    #[error("Invalid IFC_MODE {0}")]
    InvalidIfcMode(u4),
    #[error("SPI Error {0:?}")]
    Spi(hal::spi::Error),
}

impl From<hal::spi::Error> for Error {
    fn from(value: hal::spi::Error) -> Self {
        Self::Spi(value)
    }
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

#[bitfield(u24, default = 0x000700)]
#[derive(Debug, PartialEq)]
pub struct Cfg {
    #[bits(0..=3, rw)]
    rf_sw: u4,
    #[bits(4..=7, rw)]
    led: u4,
    #[bits(8..=10, rw)]
    profile: u3,
    #[bit(12, rw)]
    io_update: bool,
    #[bits(13..=16, rw)]
    mask_nu: u4,
    #[bits([17, 21], rw)]
    clk_sel: u2,
    #[bit(18, rw)]
    sync_sel: bool,
    #[bit(19, rw)]
    rst: bool,
    #[bit(20, rw)]
    io_rst: bool,
    #[bits(22..=23, rw)]
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
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled, u8>,
        cs: [hal::gpio::ErasedPin<hal::gpio::Output>; 3],
        io_update: hal::gpio::ErasedPin<hal::gpio::Output>,
        sync: hal::gpio::ErasedPin<hal::gpio::Output>,
    ) -> Result<Self, Error> {
        let mut dev = Self {
            spi,
            cs,
            io_update,
            sync,
            cfg: Cfg::default(),
            att: [0; 4],
        };
        dev.init()?;
        Ok(dev)
    }

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

    pub fn cfg(&self) -> Cfg {
        self.cfg
    }

    pub fn set_cfg(&mut self, cfg: Cfg) -> Result<Status, Error> {
        let mut cfg = cfg.raw_value().to_be_bytes();
        self.transfer(Select::Cfg, &mut cfg)?;
        Ok(Status::new_with_raw_value(u24::from_be_bytes(cfg)))
    }

    pub fn att(&self, ch: u2) -> u8 {
        self.att[ch.value() as usize]
    }

    pub fn set_att(&mut self, ch: u2, att: u8) -> Result<(), Error> {
        self.att[ch.value() as usize] = att;
        self.transfer(Select::Att, &mut self.att.clone())?;
        Ok(())
    }

    pub fn init(&mut self) -> Result<(), Error> {
        let sta = self.set_cfg(self.cfg())?;
        if sta.proto_rev() != u7::new(0x8) {
            return Err(Error::InvalidProtoRev(sta.proto_rev()));
        }
        if sta.rf_sw() != u4::new(0) {
            return Err(Error::DrivenRfSw(sta.rf_sw()));
        }
        if sta.ifc_mode() != u4::new(0) {
            return Err(Error::InvalidIfcMode(sta.ifc_mode()));
        }
        log::info!(
            "smp_err={:4b} pll_lock={:4b}",
            sta.smp_err(),
            sta.pll_lock(),
        );
        // Idempotent read of attenuators
        self.att = self.selected(Select::Att, |dev| {
            let mut att = [0; 4];
            dev.spi.transfer(&mut att)?; // Write and read, but not update
            dev.spi.write(&att)?; // Write previously read and do a NOP update
            Ok::<_, Error>(att)
        })?;
        log::info!("att: {:?}", self.att);
        log::info!("Urukul CPLD initialization complete.");
        Ok(())
    }
}
