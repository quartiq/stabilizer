use core::cell::RefCell;

use crate::hardware::ad9912;
use arbitrary_int::{u2, u24, u3, u4, u7};
use bitbybit::{bitenum, bitfield};
use embedded_hal_1::digital::OutputPin;
use embedded_hal_1::spi::{self, SpiBus, SpiDevice};
use embedded_hal_bus::spi::{DeviceError, NoDelay, RefCellDevice};
use log;

use super::ad9912::Ad9912;
use super::decoded_cs::DecodedCs;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("Invalid PROTO_REV {0}")]
    InvalidProtoRev(u7),
    #[error("RF_SW is driven {0}")]
    DrivenRfSw(u4),
    #[error("Invalid IFC_MODE {0}")]
    InvalidIfcMode(u4),
    #[error("SPI Error {0}")]
    Spi(spi::ErrorKind),
    #[error("DDS")]
    Dds(#[from] ad9912::Error),
}

impl<E: spi::Error> From<E> for Error {
    fn from(value: E) -> Self {
        Self::Spi(value.kind())
    }
}

#[bitenum(u3, exhaustive = true)]
#[derive(Debug, PartialEq, PartialOrd)]
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

pub struct Urukul<'a, B, P> {
    att_spi: RefCellDevice<'a, B, DecodedCs<'a, P, 3>, NoDelay>,
    cfg_spi: RefCellDevice<'a, B, DecodedCs<'a, P, 3>, NoDelay>,
    io_update: P,
    sync: P,
    cfg: Cfg,
    att: [u8; 4],
    dds: [Ad9912<RefCellDevice<'a, B, DecodedCs<'a, P, 3>, NoDelay>>; 4],
}

impl<'a, B: SpiBus<u8>, P: OutputPin> Urukul<'a, B, P> {
    pub fn new(
        spi: &'a RefCell<B>,
        cs: &'a RefCell<[P; 3]>,
        io_update: P,
        sync: P,
    ) -> Result<Self, Error> {
        let sel = |sel| {
            RefCellDevice::new(spi, DecodedCs::new(cs, sel), NoDelay).unwrap()
        };
        let cfg_spi = sel(1);
        let att_spi = sel(2);
        let mut dev = Self {
            cfg_spi,
            att_spi,
            io_update,
            sync,
            cfg: Cfg::default(),
            att: [0; 4],
            dds: [
                Ad9912::new(sel(4)),
                Ad9912::new(sel(5)),
                Ad9912::new(sel(6)),
                Ad9912::new(sel(7)),
            ],
        };
        dev.init()?;
        Ok(dev)
    }

    pub fn cfg(&self) -> Cfg {
        self.cfg
    }

    pub fn set_cfg(
        &mut self,
        cfg: Cfg,
    ) -> Result<Status, DeviceError<B::Error, P::Error>> {
        let cfg = cfg.raw_value().to_be_bytes();
        self.cfg_spi.write(&cfg)?;
        Ok(Status::new_with_raw_value(u24::from_be_bytes(cfg)))
    }

    pub fn att(&self, ch: u2) -> u8 {
        self.att[ch.value() as usize]
    }

    pub fn set_att(
        &mut self,
        ch: u2,
        att: u8,
    ) -> Result<(), DeviceError<B::Error, P::Error>> {
        self.att[ch.value() as usize] = att;
        self.att_spi.write(&self.att)
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

        let att = [0; 4];
        // self.att_spi.read(&mut att)?; // This is destructive and clears attenuation
        self.att_spi.write(&att)?;

        for dds in self.dds.iter_mut() {
            dds.init()?;
        }

        log::info!("Urukul CPLD initialization complete.");
        Ok(())
    }
}
