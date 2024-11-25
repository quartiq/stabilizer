use crate::hardware::ad9912;
use arbitrary_int::{u2, u24, u3, u4, u7};
use bitbybit::{bitenum, bitfield};
use embedded_hal::blocking::spi::{Transfer, Write};
use embedded_hal::digital::v2::OutputPin;
use log;
use shared_bus::{BusManager, NullMutex, SpiProxy};

#[derive(Debug, Clone, PartialEq, thiserror::Error)]
pub enum Error<E> {
    #[error("Invalid PROTO_REV {0}")]
    InvalidProtoRev(u7),
    #[error("RF_SW is driven {0}")]
    DrivenRfSw(u4),
    #[error("Invalid IFC_MODE {0}")]
    InvalidIfcMode(u4),
    #[error("SPI Error {0:?}")]
    Spi(E),
    #[error("DDS")]
    Dds(#[from] ad9912::Error<E>),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Spi(value)
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

pub enum Dds<B> {
    None,
    Bus(B),
    Ad9912(ad9912::Ad9912<B>),
}

impl<B: Transfer<u8> + Write<u8, Error = <B as Transfer<u8>>::Error>> Dds<B> {
    pub fn detect(&mut self) -> Result<(), Error<<B as Transfer<u8>>::Error>> {
        if let Self::Bus(bus) = core::mem::replace(self, Self::None) {
            *self = ad9912::Ad9912::new(bus).map(Self::Ad9912)?;
        }
        Ok(())
    }
}

pub struct Urukul<'a, B, P> {
    spi: SpiProxy<'a, NullMutex<B>>,
    cs: [P; 3],
    io_update: P,
    sync: P,
    cfg: Cfg,
    att: [u8; 4],
    dds: [Dds<SpiProxy<'a, NullMutex<B>>>; 4],
}

impl<'a, B, P> Urukul<'a, B, P>
where
    B: Transfer<u8> + Write<u8, Error = <B as Transfer<u8>>::Error>,
    P: OutputPin,
    P::Error: core::fmt::Debug,
{
    pub fn new(
        bus: &'a BusManager<NullMutex<B>>,
        cs: [P; 3],
        io_update: P,
        sync: P,
    ) -> Result<Self, Error<<B as Transfer<u8>>::Error>> {
        let mut dev = Self {
            spi: bus.acquire_spi(),
            cs,
            io_update,
            sync,
            cfg: Cfg::default(),
            att: [0; 4],
            dds: [
                Dds::Bus(bus.acquire_spi()),
                Dds::Bus(bus.acquire_spi()),
                Dds::Bus(bus.acquire_spi()),
                Dds::Bus(bus.acquire_spi()),
            ],
        };
        dev.init()?;
        Ok(dev)
    }

    fn select(&mut self, s: Select) {
        for (i, cs) in self.cs.iter_mut().enumerate() {
            cs.set_state(((s as u8 >> i) & 1 == 1).into()).unwrap();
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

    fn transfer<'b>(
        &mut self,
        select: Select,
        write: &'b mut [u8],
    ) -> Result<&'b [u8], <B as Transfer<u8>>::Error> {
        self.selected(select, |dev| dev.spi.transfer(write))
    }

    pub fn cfg(&self) -> Cfg {
        self.cfg
    }

    pub fn set_cfg(
        &mut self,
        cfg: Cfg,
    ) -> Result<Status, <B as Transfer<u8>>::Error> {
        let mut cfg = cfg.raw_value().to_be_bytes();
        self.transfer(Select::Cfg, &mut cfg)?;
        Ok(Status::new_with_raw_value(u24::from_be_bytes(cfg)))
    }

    pub fn att(&self, ch: u2) -> u8 {
        self.att[ch.value() as usize]
    }

    pub fn set_att(
        &mut self,
        ch: u2,
        att: u8,
    ) -> Result<(), <B as Transfer<u8>>::Error> {
        self.att[ch.value() as usize] = att;
        self.transfer(Select::Att, &mut self.att.clone())?;
        Ok(())
    }

    pub fn init(&mut self) -> Result<(), Error<<B as Transfer<u8>>::Error>> {
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
            Ok::<_, Error<_>>(att)
        })?;
        log::info!("att: {:?}", self.att);

        for i in 0..4 {
            self.selected(
                Select::new_with_raw_value(
                    Select::Dds0.raw_value() + u3::new(i as _),
                ),
                |dev| dev.dds[i].detect(),
            )?;
        }

        log::info!("Urukul CPLD initialization complete.");
        Ok(())
    }
}
