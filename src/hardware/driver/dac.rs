///! Driver DAC11001 driver
use core::fmt::Debug;
use embedded_hal::blocking::spi::{Transfer, Write};
use stm32h7xx_hal::gpio;

use super::ChannelVariant;
use embedded_hal::blocking::delay::DelayUs;

// DAC11001 register addresses
#[allow(unused, non_camel_case_types)]
pub enum DAC_ADDR {
    NOP = 0x00 << 24,
    DAC_DATA = 0x01 << 24,
    CONFIG1 = 0x02 << 24,
    DAC_CLEAR_DATA = 0x03 << 24,
    TRIGGER = 0x04 << 24,
    STATUS = 0x05 << 24,
    CONFIG2 = 0x06 << 24,
}

#[allow(non_snake_case)]
pub mod CONFIG1 {
    pub mod EN_TMP_CAL {
        pub const DISABLED: u32 = 0b0 << 23;
        pub const ENABLED: u32 = 0b1 << 23;
    }
    pub mod TNH_MASK {
        pub const JUMP14: u32 = 0b00 << 18;
        pub const JUMP15: u32 = 0b01 << 18;
        pub const JUMP13: u32 = 0b10 << 18;
        pub const JUMP12: u32 = 0b11 << 18;
    }
    pub mod LDACMODE {
        pub const ASYNC: u32 = 0b0 << 14;
        pub const SYNC: u32 = 0b1 << 14;
    }
    pub mod FSDO {
        pub const DISABLED: u32 = 0b0 << 13;
        pub const ENABLED: u32 = 0b1 << 13;
    }
    pub mod ENALMP {
        pub const DISABLED: u32 = 0b0 << 12;
        pub const ENABLED: u32 = 0b1 << 12;
    }
    pub mod DSDO {
        pub const DISABLED: u32 = 0b0 << 11;
        pub const ENABLED: u32 = 0b1 << 11;
    }
    pub mod FSET {
        pub const DISABLED: u32 = 0b0 << 10;
        pub const ENABLED: u32 = 0b1 << 10;
    }
    pub mod VREFVAL {
        pub const SPAN_5V: u32 = 0b0010 << 6;
        pub const SPAN_7_5V: u32 = 0b0011 << 6;
        pub const SPAN_10V: u32 = 0b0100 << 6;
        pub const SPAN_12_5V: u32 = 0b0101 << 6;
        pub const SPAN_15V: u32 = 0b0110 << 6;
        pub const SPAN_17_5V: u32 = 0b0111 << 6;
        pub const SPAN_20V: u32 = 0b1000 << 6;
        pub const SPAN_22_5V: u32 = 0b1001 << 6;
        pub const SPAN_25V: u32 = 0b1010 << 6;
        pub const SPAN_27_5V: u32 = 0b1011 << 6;
        pub const SPAN_30V: u32 = 0b1100 << 6;
    }
    pub mod PDN {
        pub const POWERUP: u32 = 0b0 << 4;
        pub const POWERDOWN: u32 = 0b1 << 4;
    }
}

#[allow(non_snake_case)]
pub mod TRIGGER {
    pub mod RCLTMP {
        pub const RECAL: u32 = 0b1 << 8;
    }
    pub mod SRST {
        pub const RESET: u32 = 0b1 << 6;
    }
    pub mod SCLR {
        pub const CLEAR: u32 = 0b1 << 5;
    }
}

#[allow(non_snake_case)]
pub mod STATUS {
    pub mod ALM {
        pub const RECALIBRATED: u32 = 0b1 << 12;
    }
}

#[allow(non_snake_case)]
pub mod CONFIG2 {
    pub mod DIS_TNH {
        pub const DISABLED: u32 = 0b0 << 7;
        pub const ENABLED: u32 = 0b1 << 7;
    }
    pub mod UP_RATE {
        pub const UPDATE_RATE_1_MHZ_38_MHZ_SCLK: u32 = 0b000 << 4;
        pub const UPDATE_RATE_0_9_MHZ_34_MHZ_SCLK: u32 = 0b001 << 4;
        pub const UPDATE_RATE_0_8_MHZ_31_MHZ_SCLK: u32 = 0b010 << 4;
        pub const UPDATE_RATE_1_2_MHZ_45_MHZ_SCLK: u32 = 0b011 << 4;
        pub const UPDATE_RATE_0_5_MHZ_21_MHZ_SCLK: u32 = 0b100 << 4;
        pub const UPDATE_RATE_0_45_MHZ_18_MHZ_SCLK: u32 = 0b101 << 4;
        pub const UPDATE_RATE_0_4_MHZ_16_MHZ_SCLK: u32 = 0b110 << 4;
        pub const UPDATE_RATE_0_6_MHZ_24_MHZ_SCLK: u32 = 0b111 << 4;
    }
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds(f32, i32),
}

/// A type representing a DAC sample.
#[derive(Copy, Clone, Debug)]
pub struct DacCode(u32);
impl DacCode {
    // DAC constants
    const MAX_DAC_WORD: i32 = 1 << 24; // maximum DAC dataword plus 4 bits to avoid shift later (exclusive)
    const VREF_DAC: f32 = 10.0; // Difference between positive and negaitiv reference pin
}

impl TryFrom<(f32, ChannelVariant)> for DacCode {
    type Error = Error;
    /// Convert an f32 representing a current into the corresponding DAC output code for the respective channel.
    fn try_from(current_channel: (f32, ChannelVariant)) -> Result<Self, Error> {
        let (current, channel) = current_channel;
        let scale = channel.transimpedance()
            * (DacCode::MAX_DAC_WORD as f32 / DacCode::VREF_DAC);
        let mut code = (current * scale) as i32;
        if scale < 0. {
            // Flip the sign and overflow bits
            code ^= !(DacCode::MAX_DAC_WORD - 1);
        }
        if !(0..DacCode::MAX_DAC_WORD).contains(&code) {
            Err(Error::Bounds(current, code))
        } else {
            Ok(Self((code & (DacCode::MAX_DAC_WORD - 1)) as u32))
        }
    }
}

impl From<DacCode> for u32 {
    fn from(code: DacCode) -> u32 {
        code.0
    }
}

pub struct Dac<SPI: Transfer<u8> + Write<u8>> {
    spi: SPI,
    sync_n: gpio::ErasedPin<gpio::Output>,
    channel: ChannelVariant,
}

impl<SPI, E> Dac<SPI>
where
    SPI: Transfer<u8, Error = E> + Write<u8, Error = E>,
    E: Debug,
{
    /// Construct a new [Dac] in the following state:
    /// - reset
    /// - 10 V plusminus 1.25 V refernece span configured
    /// - temperature drift calibration performed
    /// - 0.5-MHz max DAC update rate by default for enhanced THD performance
    /// - TNH masked for code jump > 2^14 (default)
    pub fn new(
        spi: SPI,
        sync_n: gpio::ErasedPin<gpio::Output>,
        channel: ChannelVariant,
        #[allow(unused)] delay: &mut impl DelayUs<u8>,
    ) -> Self {
        let mut dac = Dac {
            spi,
            sync_n,
            channel,
        };

        // reset DAC
        dac.write(DAC_ADDR::TRIGGER, TRIGGER::SRST::RESET);
        // set to 10 V plusminus 1.25 V referenece span, retain defaults
        dac.write(
            DAC_ADDR::CONFIG1,
            CONFIG1::VREFVAL::SPAN_10V
                | CONFIG1::DSDO::ENABLED // set to retain default
                | CONFIG1::FSET::ENABLED // set to retain default
                | CONFIG1::LDACMODE::ASYNC, // set to use SPI SYNC
        );
        log::info!("val: {:b}", 
            CONFIG1::VREFVAL::SPAN_10V
            | CONFIG1::DSDO::ENABLED // set to retain default
            | CONFIG1::FSET::ENABLED // set to retain default
            | CONFIG1::LDACMODE::ASYNC, // set to use SPI SYNC
        );
        // perform calibration
        // don't try to calibrate without driver because it will wait forever
        // dac.calibrate(delay);

        dac
    }

    /// Perform temperature drift calibration (waits until calibration is done).
    pub fn calibrate(&mut self, delay: &mut impl DelayUs<u8>) {
        self.write(DAC_ADDR::CONFIG1, CONFIG1::EN_TMP_CAL::ENABLED);
        self.write(DAC_ADDR::TRIGGER, TRIGGER::RCLTMP::RECAL);
        // continuously read recalibration done bit until it is set
        while (self.read(DAC_ADDR::STATUS, delay) & STATUS::ALM::RECALIBRATED)
            == 0
        {
            delay.delay_us(1)
        }
    }

    /// Set the DAC to produce a voltage corresponding to `current`.
    pub fn set(&mut self, current: f32) -> Result<(), Error> {
        let dac_code = DacCode::try_from((current, self.channel))?;
        self.write(DAC_ADDR::DAC_DATA, dac_code.0);
        Ok(())
    }

    /// Write 24 bit [data] to the DAC at [addr].
    pub fn write(&mut self, addr: DAC_ADDR, data: u32) {
        let bytes = (addr as u32 | (data & 0xffffff)).to_be_bytes();
        self.sync_n.set_low();
        self.spi.write(&bytes).unwrap();
        self.sync_n.set_high();
    }

    /// Perform a read of a DAC register. First writes a word and then reads one.
    pub fn read(
        &mut self,
        addr: DAC_ADDR,
        delay: &mut impl DelayUs<u8>,
    ) -> u32 {
        let mut bytes = (1u32 << 31 | addr as u32).to_be_bytes();
        self.sync_n.set_low();
        self.spi.write(&bytes).unwrap();
        self.sync_n.set_high();
        delay.delay_us(1); // has to be high for at least 100 ns
        self.sync_n.set_low();
        self.spi.transfer(&mut bytes).unwrap();
        self.sync_n.set_high();
        u32::from_be_bytes(bytes)
    }
}
