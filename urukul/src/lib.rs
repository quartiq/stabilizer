#![no_std]

use arbitrary_int::{u2, u24, u3, u4, u7};
use bitbybit::{bitenum, bitfield};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{self, SpiBus, SpiDevice};
use embedded_hal_bus::spi::{DeviceError, NoDelay, RefCellDevice};
use num_traits::Float;
use serde::{Deserialize, Serialize};

use ad9912::Ad9912;
use encoded_pin::EncodedPin;

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("Initialization: {0}: {1}")]
    Initialization(&'static str, u32),
    #[error("SPI Error {0}")]
    Spi(spi::ErrorKind),
    #[error("DDS")]
    Dds(#[source] ad9912::Error),
}

impl<E: spi::Error> From<E> for Error {
    fn from(value: E) -> Self {
        Self::Spi(value.kind())
    }
}

#[bitfield(u24, default = 0x000700)]
#[derive(Debug, PartialEq, Serialize, Deserialize)]
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
    clk_sel: ClkSel,
    #[bit(18, rw)]
    sync_sel: bool,
    #[bit(19, rw)]
    rst: bool,
    #[bit(20, rw)]
    io_rst: bool,
    #[bits(22..=23, rw)]
    div_sel: DivSel,
}

#[bitfield(u24)]
#[derive(Debug, PartialEq, Serialize, Deserialize)]
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

#[bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum ClkSel {
    Osc = 0,
    Sma = 1,
    Mmcx = 2,
    _Sma = 3,
}

#[bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq, Serialize, Deserialize)]
pub enum DivSel {
    One = 0,
    _One = 1,
    Two = 2,
    Four = 3,
}

impl DivSel {
    pub fn divider(&self) -> u32 {
        match self {
            Self::One => 1,
            Self::Two => 2,
            Self::Four => 4,
            Self::_One => 1,
        }
    }
}

pub fn att_to_mu(att: f32) -> u8 {
    255 - (att * 8.0).round() as u8
}

pub struct Urukul<'a, B, P> {
    att_spi: RefCellDevice<'a, B, EncodedPin<'a, P, 3>, NoDelay>,
    cfg_spi: RefCellDevice<'a, B, EncodedPin<'a, P, 3>, NoDelay>,
    io_update: P,
    _sync: P,
    cfg: Cfg,
    att: [u8; 4],
    dds: [Ad9912<RefCellDevice<'a, B, EncodedPin<'a, P, 3>, NoDelay>>; 4],
}

impl<'a, B: SpiBus<u8>, P: OutputPin> Urukul<'a, B, P> {
    pub fn new(
        spi: &'a core::cell::RefCell<B>,
        cs: &'a core::cell::RefCell<[P; 3]>,
        io_update: P,
        sync: P,
    ) -> Result<Self, Error> {
        let sel = |sel| {
            RefCellDevice::new(spi, EncodedPin::new(cs, u3::new(sel)), NoDelay)
                .unwrap()
        };
        let cfg_spi = sel(1);
        let att_spi = sel(2);
        let mut dev = Self {
            cfg_spi,
            att_spi,
            io_update,
            _sync: sync,
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
        let mut bits = [0; 3];
        let w = cfg.raw_value().to_be_bytes();
        self.cfg_spi.transfer(&mut bits, &w)?;
        self.cfg = cfg;
        Ok(Status::new_with_raw_value(u24::from_be_bytes(bits)))
    }

    pub fn att(&self, ch: u2) -> u8 {
        self.att[(u2::new(3) - ch).value() as usize]
    }

    pub fn set_att(
        &mut self,
        ch: u2,
        att: u8,
    ) -> Result<(), DeviceError<B::Error, P::Error>> {
        self.att[(u2::new(3) - ch).value() as usize] = att;
        self.att_spi.write(&self.att)
    }

    pub fn init(&mut self) -> Result<(), Error> {
        let sta = self.set_cfg(self.cfg())?;
        if sta.proto_rev().value() != 0x8 {
            return Err(Error::Initialization(
                "Invalid PROTO_REV",
                sta.proto_rev().value() as _,
            ));
        }
        if sta.rf_sw() != u4::new(0) {
            return Err(Error::Initialization(
                "RF_SW driven",
                sta.rf_sw().value() as _,
            ));
        }
        if sta.ifc_mode() != u4::new(0) {
            return Err(Error::Initialization(
                "invalid IFC_MODE",
                sta.ifc_mode().value() as _,
            ));
        }

        let cfg = self.cfg();
        self.set_cfg(cfg.with_io_rst(true).with_rst(true))?;
        self.set_cfg(cfg)?;

        for want in [[0; 4], [0xff; 4], [0x5a; 4], [0xa5; 4]].iter() {
            self.att_spi.write(want)?;
            let mut have = [0; 4];
            self.att_spi.read(&mut have)?;
            if want != &have {
                return Err(Error::Initialization(
                    "Attenuator mismatch",
                    u32::from_be_bytes(have),
                ));
            }
        }

        // This is destructive and clears attenuation
        // https://github.com/rust-embedded/embedded-hal/issues/642
        self.att_spi.write(&self.att)?;

        for dds in self.dds.iter_mut() {
            dds.init().map_err(Error::Dds)?;
        }

        log::info!("Urukul initialized");
        Ok(())
    }

    pub fn io_update(&mut self) -> Result<(), P::Error> {
        self.io_update.set_high()?;
        self.io_update.set_low()
    }

    pub fn set_rf_sw(
        &mut self,
        ch: u2,
        state: bool,
    ) -> Result<(), DeviceError<B::Error, P::Error>> {
        let mut v = self.cfg.rf_sw();
        v &= !u4::new(1u8 << ch.value());
        v |= u4::new((state as u8) << ch.value());
        self.set_cfg(self.cfg.with_rf_sw(v))?;
        Ok(())
    }

    pub fn set_led(
        &mut self,
        ch: u2,
        state: bool,
    ) -> Result<(), DeviceError<B::Error, P::Error>> {
        let mut v = self.cfg.led();
        v &= !u4::new(1u8 << ch.value());
        v |= u4::new((state as u8) << ch.value());
        self.set_cfg(self.cfg.with_led(v))?;
        Ok(())
    }

    pub fn dds(
        &mut self,
        ch: u2,
    ) -> &mut Ad9912<RefCellDevice<'a, B, EncodedPin<'a, P, 3>, NoDelay>> {
        &mut self.dds[ch.value() as usize]
    }
}
