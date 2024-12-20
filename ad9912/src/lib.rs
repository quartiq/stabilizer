#![no_std]

use arbitrary_int::{u10, u14, u48, u5, Number};
use bitbybit::{bitenum, bitfield};
use embedded_hal::spi::{self, Operation, SpiDevice};
use num_traits::float::FloatCore;

#[bitenum(u13)]
#[derive(PartialEq, Debug)]
pub enum Addr {
    Serial = 0x0000,
    PartId = 0x0003,
    Buffer = 0x0004,
    Update = 0x0005,
    Power = 0x0010,
    DdsReset = 0x0012,
    Reset = 0x0013,
    NDiv = 0x0020,
    Pll = 0x0022,
    SDiv = 0x0106,
    Ftw0 = 0x01ab,
    Phase = 0x01ad,
    Hstl = 0x0200,
    Cmos = 0x0201,
    Fsc = 0x040c,
    SpurA = 0x0504,
    SpurB = 0x0509,
}

#[bitenum(u2, exhaustive = true)]
#[derive(PartialEq, Debug, Default)]
pub enum Size {
    #[default]
    One = 0,
    Two = 1,
    Three = 2,
    Stream = 3,
}

impl From<usize> for Size {
    fn from(value: usize) -> Self {
        match value {
            0 => unimplemented!(),
            1 => Self::One,
            2 => Self::Two,
            3 => Self::Three,
            _ => Self::Stream,
        }
    }
}

#[bitfield(u16)]
#[derive(Debug, PartialEq)]
pub struct Instruction {
    #[bits(0..=12, rw)]
    addr: Option<Addr>,
    #[bits(13..=14, rw)]
    size: Size,
    #[bit(15, rw)]
    read: bool,
}

#[bitfield(u8, default = 0x18)]
#[derive(Debug, PartialEq)]
pub struct Serial {
    #[bit(0, rw)]
    sdo_active: bool,
    #[bit(1, rw)]
    lsb_first: bool,
    #[bit(2, rw)]
    soft_reset: bool,
    #[bit(3, rw)]
    long_insn: bool,
}

impl Serial {
    pub fn mirror(self) -> Self {
        let v = self.raw_value();
        Self::new_with_raw_value(
            v & 0x0f
                | ((v & 1) << 7)
                | ((v & 2) << 5)
                | ((v & 4) << 3)
                | ((v & 8) << 1),
        )
    }
}

#[bitfield(u8, default = 0xc0)]
#[derive(Debug, PartialEq)]
pub struct Power {
    #[bit(0, rw)]
    digital_pd: bool,
    #[bit(1, rw)]
    full_pd: bool,
    #[bit(4, rw)]
    pll_pd: bool,
    #[bit(5, rw)]
    output_doubler_en: bool,
    #[bit(6, rw)]
    cmos_en: bool,
    #[bit(7, rw)]
    hstl_pd: bool,
}

#[bitfield(u8, default = 0x00)]
#[derive(Debug, PartialEq)]
pub struct Reset {
    #[bit(1, rw)]
    sdiv: bool,
    #[bit(3, rw)]
    sdiv2: bool,
    #[bit(7, rw)]
    fund_dds_pd: bool,
}

#[bitenum(u2, exhaustive = true)]
#[derive(Debug, PartialEq)]
pub enum ChargePump {
    Ua250 = 0,
    Ua375 = 1,
    Off = 2,
    Ua125 = 3,
}

#[bitfield(u8, default = 0x04)]
#[derive(Debug, PartialEq)]
pub struct Pll {
    #[bits(0..=1, rw)]
    charge_pump: ChargePump,
    #[bit(2, rw)]
    vco_range_high: bool,
    #[bit(3, rw)]
    ref_doubler: bool,
    #[bit(7, rw)]
    vco_auto_range: bool,
}

impl Pll {
    pub fn set_refclk(&mut self, ndiv: u5, refclk: f64) -> f64 {
        let sysclk = refclk
            * ((self.ref_doubler() as u8 + 1) * 2 * (ndiv.value() + 2)) as f64;
        *self = if sysclk > 900e6 {
            self.with_vco_auto_range(false).with_vco_range_high(true)
        } else if sysclk > 810e6 {
            self.with_vco_auto_range(true)
        } else {
            self.with_vco_auto_range(false).with_vco_range_high(false)
        };
        sysclk
    }
}

#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum Error {
    #[error("Invalid Part ID {0}")]
    Id(u16),
    #[error("SPI")]
    Bus(spi::ErrorKind),
}

impl<E: spi::Error> From<E> for Error {
    fn from(value: E) -> Self {
        value.kind().into()
    }
}

#[derive(Clone, Debug)]
pub struct Ad9912<B> {
    bus: B,
}

pub fn frequency_to_ftw(frequency: f64, sysclk: f64) -> u48 {
    let lsb = sysclk.recip() * (1u64 << 48) as f64;
    // Alias into Nyquist
    u48::new(((frequency * lsb).round() as i64 as u64) & u48::MASK)
}

pub fn phase_to_pow(phase: f32) -> u14 {
    // Alias into Nyquist
    u14::new(((phase * (1u32 << 14) as f32).round() as i32 as u16) & u14::MASK)
}

pub fn dac_fs_to_fsc(dac_fs: f32, r_dac_ref: f32) -> u10 {
    let lsb = r_dac_ref * (1024.0 / 192.0 / 1.2);
    let fsc = dac_fs * lsb - (1024.0 / 192.0 * 72.0);
    // Clamp
    u10::new(
        fsc.round()
            .clamp(u10::MIN.value() as _, u10::MAX.value() as _) as _,
    )
}

impl<B: SpiDevice<u8>> Ad9912<B> {
    pub fn new(bus: B) -> Self {
        Self { bus }
    }

    fn write(&mut self, addr: Addr, data: &[u8]) -> Result<(), Error> {
        Ok(self.bus.transaction(&mut [
            Operation::Write(
                &Instruction::builder()
                    .with_addr(addr)
                    .with_size(data.len().into())
                    .with_read(false)
                    .build()
                    .raw_value()
                    .to_be_bytes(),
            ),
            Operation::Write(data),
        ])?)
    }

    fn read(&mut self, addr: Addr, data: &mut [u8]) -> Result<(), Error> {
        Ok(self.bus.transaction(&mut [
            Operation::Write(
                &Instruction::builder()
                    .with_addr(addr)
                    .with_size(data.len().into())
                    .with_read(true)
                    .build()
                    .raw_value()
                    .to_be_bytes(),
            ),
            Operation::Read(data),
        ])?)
    }

    pub fn init(&mut self) -> Result<(), Error> {
        self.write(
            Addr::Serial,
            &Serial::builder()
                .with_sdo_active(true)
                .with_lsb_first(false)
                .with_soft_reset(false)
                .with_long_insn(true)
                .build()
                .mirror()
                .raw_value()
                .to_be_bytes(),
        )?;
        let mut id = [0; 2];
        self.read(Addr::PartId, &mut id)?;
        let id = u16::from_be_bytes(id);
        if id != 0x1982 {
            return Err(Error::Id(id));
        }
        Ok(())
    }

    pub fn set_power(&mut self, power: Power) -> Result<(), Error> {
        self.write(Addr::Power, &power.raw_value().to_be_bytes())
    }

    /// Non-clearing, needs init()
    pub fn soft_reset(&mut self) -> Result<(), Error> {
        self.write(
            Addr::Serial,
            &Serial::builder()
                .with_sdo_active(true)
                .with_lsb_first(false)
                .with_soft_reset(true)
                .with_long_insn(true)
                .build()
                .mirror()
                .raw_value()
                .to_be_bytes(),
        )
    }

    /// Needs io-update
    pub fn dds_reset(&mut self) -> Result<(), Error> {
        self.write(Addr::DdsReset, &0x01u8.to_be_bytes())
    }

    pub fn set_ndiv(&mut self, ndiv: u5) -> Result<(), Error> {
        self.write(Addr::NDiv, &ndiv.value().to_be_bytes())
    }

    pub fn set_pll(&mut self, pll: Pll) -> Result<(), Error> {
        self.write(Addr::Pll, &pll.raw_value().to_be_bytes())
    }

    pub fn set_ftw(&mut self, ftw: u48) -> Result<(), Error> {
        self.write(Addr::Ftw0, &ftw.to_be_bytes())
    }

    pub fn ftw(&mut self) -> Result<u48, Error> {
        let mut r = u48::default().to_be_bytes();
        self.read(Addr::Ftw0, &mut r)?;
        Ok(u48::from_be_bytes(r))
    }

    pub fn set_frequency(
        &mut self,
        frequency: f64,
        sysclk: f64,
    ) -> Result<u48, Error> {
        let ftw = frequency_to_ftw(frequency, sysclk);
        self.set_ftw(ftw)?;
        Ok(ftw)
    }

    pub fn set_pow(&mut self, pow: u14) -> Result<(), Error> {
        self.write(Addr::Phase, &pow.value().to_be_bytes())
    }

    pub fn set_phase(&mut self, phase: f32) -> Result<u14, Error> {
        let pow = phase_to_pow(phase);
        self.set_pow(pow)?;
        Ok(pow)
    }

    pub fn set_fsc(&mut self, fsc: u10) -> Result<(), Error> {
        self.write(Addr::Fsc, &fsc.value().to_be_bytes())
    }

    pub fn set_full_scale_current(
        &mut self,
        dac_fs: f32,
        r_dac_ref: f32,
    ) -> Result<u10, Error> {
        let fsc = dac_fs_to_fsc(dac_fs, r_dac_ref);
        self.set_fsc(fsc)?;
        Ok(fsc)
    }
}
