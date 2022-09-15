///! Driver DAC11001 driver
use core::fmt::Debug;
use embedded_hal::blocking::spi::{Transfer, Write};
use stm32h7xx_hal::gpio;

use super::ChannelVariant;

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
        pub const DISABLED: u32 = 0 << 23;
        pub const ENABLED: u32 = 1 << 23;
    }
    pub mod TNH_MASK {
        pub const JUMP14: u32 = 00 << 18;
        pub const JUMP15: u32 = 01 << 18;
        pub const JUMP13: u32 = 10 << 18;
        pub const JUMP12: u32 = 11 << 18;
    }
    pub mod LDACMODE {
        pub const SYNC: u32 = 0 << 14;
        pub const ASYNC: u32 = 1 << 14;
    }
    pub mod FSDO {
        pub const DISABLED: u32 = 0 << 13;
        pub const ENABLED: u32 = 1 << 13;
    }
    pub mod ENALMP {
        pub const DISABLED: u32 = 0 << 12;
        pub const ENABLED: u32 = 1 << 12;
    }
    pub mod DSOO {
        pub const DISABLED: u32 = 0 << 11;
        pub const ENABLED: u32 = 1 << 11;
    }
    pub mod FSET {
        pub const DISABLED: u32 = 0 << 10;
        pub const ENABLED: u32 = 1 << 10;
    }
    pub mod VREFVAL {
        pub const SPAN_5V_1_25V: u32 = 0010 << 6;
        pub const SPAN_7_5V_1_25V: u32 = 0011 << 6;
        pub const SPAN_10V_1_25V: u32 = 0100 << 6;
        pub const SPAN_12_5V_1_25V: u32 = 0101 << 6;
        pub const SPAN_15V_1_25V: u32 = 0110 << 6;
        pub const SPAN_17_5V_1_25V: u32 = 0111 << 6;
        pub const SPAN_20V_1_25V: u32 = 1000 << 6;
        pub const SPAN_22_5V_1_25V: u32 = 1001 << 6;
        pub const SPAN_25V_1_25V: u32 = 1010 << 6;
        pub const SPAN_27_5V_1_25V: u32 = 1011 << 6;
        pub const SPAN_30V_1_25V: u32 = 1100 << 6;
    }
    pub mod PDN {
        pub const POWERUP: u32 = 0 << 4;
        pub const POWERDOWN: u32 = 1 << 4;
    }
}

#[allow(non_snake_case)]
pub mod TRIGGER {
    pub mod RCLTMP {
        pub const RECAL: u32 = 1 << 8;
    }
    pub mod SRST {
        pub const RESET: u32 = 1 << 6;
    }
    pub mod SCLR {
        pub const CLEAR: u32 = 1 << 5;
    }
}

#[allow(non_snake_case)]
pub mod STATUS {
    pub mod ALM {
        pub const RECALIBRATED: u32 = 1 << 12;
    }
}

pub enum Polarity {
    AnodeGrounded,
    CathodeGrounded,
}

#[allow(non_snake_case)]
pub mod CONFIG2 {
    pub mod DIS_TNH {
        pub const DISABLED: u32 = 0 << 7;
        pub const ENABLED: u32 = 1 << 7;
    }
    pub mod UP_RATE {
        pub const UPDATE_RATE_1_MHZ_38_MHZ_SCLK: u32 = 000 << 4;
        pub const UPDATE_RATE_0_9_MHZ_34_MHZ_SCLK: u32 = 001 << 4;
        pub const UPDATE_RATE_0_8_MHZ_31_MHZ_SCLK: u32 = 010 << 4;
        pub const UPDATE_RATE_1_2_MHZ_45_MHZ_SCLK: u32 = 011 << 4;
        pub const UPDATE_RATE_0_5_MHZ_21_MHZ_SCLK: u32 = 100 << 4;
        pub const UPDATE_RATE_0_45_MHZ_18_MHZ_SCLK: u32 = 101 << 4;
        pub const UPDATE_RATE_0_4_MHZ_16_MHZ_SCLK: u32 = 110 << 4;
        pub const UPDATE_RATE_0_6_MHZ_24_MHZ_SCLK: u32 = 111 << 4;
    }
}

/// DAC value out of bounds error.
#[derive(Debug)]
pub enum Error {
    Bounds,
}

/// A type representing a DAC sample.
#[derive(Copy, Clone, Debug)]
pub struct DacCode(u32);
impl DacCode {
    // DAC constants
    const MAX_DAC_WORD: i32 = 1 << 20; // maximum DAC dataword (exclusive)
    const VREF_DAC: f32 = 10.0; // Difference between positive and negaitiv reference pin
    const R_OUT_LN: f32 = 40.0; // Low noise side output resistor
    const R_OUT_HP: f32 = 0.68; // High power side output resistor
}

impl TryFrom<(f32, ChannelVariant)> for DacCode {
    type Error = Error;
    /// Convert an f32 representing a current int the corresponding DAC output code for the respective channel.
    fn try_from(current_channel: (f32, ChannelVariant)) -> Result<Self, Error> {
        let (current, channel) = current_channel;
        let (r_out, is_inverted) = match channel {
            ChannelVariant::LowNoiseAnodeGrounded => (Self::R_OUT_LN, true),
            ChannelVariant::LowNoiseCathodeGrounded => (Self::R_OUT_LN, false),
            ChannelVariant::HighPowerAnodeGrounded => (Self::R_OUT_HP, true),
            ChannelVariant::HighPowerCathodeGrounded => (Self::R_OUT_HP, false),
        };
        let mut dac_code = (r_out
            * current
            * (DacCode::MAX_DAC_WORD as f32 / DacCode::VREF_DAC))
            as i32;
        log::info!("dac_code before: {:?}", dac_code);

        if is_inverted {
            // Convert to inverted dac output for anode grounded Driver channel variants.
            // These variants need (VREF_DAC - V_CURR) to produce the current CURR.
            // No bitinversion and masking so we can still detect bounds errors.
            dac_code = DacCode::MAX_DAC_WORD - dac_code + 1;
        }
        log::info!("dac_code after: {:?}", dac_code);
        if !(0..DacCode::MAX_DAC_WORD).contains(&dac_code) {
            return Err(Error::Bounds);
        };

        Ok(Self(dac_code as u32))
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
    /// - deglitch circuit is enabled by default (fast-settling off)
    /// - asyncronous output update (when sync goes high) by default
    /// - 0.5-MHz DAC update rate by default for enhanced THD performance
    /// - temperature drift calibration performed
    pub fn new(
        spi: SPI,
        sync_n: gpio::ErasedPin<gpio::Output>,
        channel: ChannelVariant,
    ) -> Self {
        let mut dac = Dac {
            spi,
            sync_n,
            channel,
        };

        // reset DAC
        dac.write(DAC_ADDR::TRIGGER as u32 | TRIGGER::SRST::RESET);
        dac.calibrate();

        dac
    }

    /// Perform temperature drift calibration (waits until calibration is done).
    pub fn calibrate(&mut self) {
        self.write(DAC_ADDR::CONFIG1 as u32 | CONFIG1::EN_TMP_CAL::ENABLED);
        self.write(DAC_ADDR::TRIGGER as u32 | TRIGGER::RCLTMP::RECAL);
        // continuously read recalibration done bit until it is set
        while (self.read(DAC_ADDR::STATUS) & STATUS::ALM::RECALIBRATED) == 0 {}
    }

    /// Set the DAC to produce a voltage corresponding to `current`.
    pub fn set(&mut self, current: f32) -> Result<(), Error> {
        let dac_code = DacCode::try_from((current, self.channel))?;
        let word = DAC_ADDR::DAC_DATA as u32 | (dac_code.0 << 4);
        self.write(word);
        Ok(())
    }

    /// Write a 32 bit word to the DAC.
    pub fn write(&mut self, word: u32) {
        let bytes = word.to_be_bytes();
        self.sync_n.set_low();
        self.spi.write(&bytes).unwrap();
        self.sync_n.set_high();
    }

    /// Perform a read of a DAC register. First writes a word and then reads one.
    pub fn read(&mut self, addr: DAC_ADDR) -> u32 {
        let mut buf = [0u8; 8];
        buf[0] = 1u8 << 7 | addr as u8;
        self.sync_n.set_low();
        self.spi.transfer(&mut buf).unwrap();
        self.sync_n.set_high();
        u64::from_be_bytes(buf) as u32 // just cut off the upper word where no data was received
    }
}
