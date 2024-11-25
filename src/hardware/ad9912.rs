use arbitrary_int::{u10, u2, u5};
use bitbybit::{bitenum, bitfield};
use embedded_hal::blocking::spi::{Transfer, Write};

#[bitenum(u13)]
#[derive(PartialEq, Debug)]
pub enum Addr {
    Serial = 0x0000,
    PartId = 0x0003,
    Buffer = 0x0004,
    Update = 0x0005,
    Power = 0x0010,
    DdsReset = 0x0013,
    Reset = 0x0014,
    NDiv = 0x0020,
    Pll = 0x0022,
    SDiv = 0x0106,
    Ftw0 = 0x01ab,
    Phase = 0x01ad,
    Hstl = 0x0200,
    Cmos = 0x0201,
    Fsc = 0x040c,
    SpurA = 0x0500,
    SpurB = 0x0505,
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
pub struct DdsReset {
    #[bit(0, rw)]
    dds: bool,
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

#[bitfield(u8, default = 0x12)]
#[derive(Debug, PartialEq)]
pub struct NDiv {
    #[bits(0..=4, rw)]
    ndiv: u5,
}

#[bitfield(u8, default = 0x04)]
#[derive(Debug, PartialEq)]
pub struct Pll {
    #[bits(0..=1, rw)]
    current: u2,
    #[bit(2, rw)]
    vco_range: bool,
    #[bit(3, rw)]
    ref_doubler: bool,
    #[bit(7, rw)]
    vco_auto_range: bool,
}

#[bitfield(u16, default = 0x01ff)]
#[derive(Debug, PartialEq)]
pub struct Fsc {
    #[bits(0..=9, rw)]
    fsc: u10,
}

#[derive(Debug, Clone, Copy, PartialEq, thiserror::Error)]
pub enum Error<E> {
    #[error("Invalid Part ID {0}")]
    Id(u16),
    #[error("SPI")]
    Bus(E),
}

impl<E> From<E> for Error<E> {
    fn from(value: E) -> Self {
        Self::Bus(value)
    }
}

#[derive(Clone, Debug)]
pub struct Ad9912<B> {
    bus: B,
}

impl<B: Transfer<u8> + Write<u8, Error = <B as Transfer<u8>>::Error>>
    Ad9912<B>
{
    pub fn new(bus: B) -> Result<Self, Error<<B as Transfer<u8>>::Error>> {
        let mut dev = Self { bus };
        dev.init()?;
        Ok(dev)
    }

    pub fn init(&mut self) -> Result<(), Error<<B as Transfer<u8>>::Error>> {
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
            Err(Error::Id(id))
        } else {
            Ok(())
        }
    }

    pub fn write(
        &mut self,
        addr: Addr,
        data: &[u8],
    ) -> Result<(), Error<<B as Transfer<u8>>::Error>> {
        self.bus.write(
            &Instruction::builder()
                .with_addr(addr)
                .with_size(data.len().into())
                .with_read(false)
                .build()
                .raw_value()
                .to_be_bytes(),
        )?;
        Ok(self.bus.write(data)?)
    }

    pub fn read<'a>(
        &mut self,
        addr: Addr,
        data: &'a mut [u8],
    ) -> Result<&'a [u8], Error<<B as Transfer<u8>>::Error>> {
        self.bus.write(
            &Instruction::builder()
                .with_addr(addr)
                .with_size(data.len().into())
                .with_read(true)
                .build()
                .raw_value()
                .to_be_bytes(),
        )?;
        Ok(self.bus.transfer(data)?)
    }
}
