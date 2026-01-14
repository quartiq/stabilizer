use bitbybit::bitenum;
use core::convert::TryFrom;
use serde::{Deserialize, Serialize};

#[derive(Debug, Serialize, Deserialize, Default)]
#[bitenum(u2, exhaustive = true)]
pub enum Gain {
    #[default]
    G1 = 0b00,
    G2 = 0b01,
    G5 = 0b10,
    G10 = 0b11,
}

impl Gain {
    /// Get the AFE gain as a numerical value.
    pub const fn gain(self) -> f32 {
        match self {
            Gain::G1 => 1.0,
            Gain::G2 => 2.0,
            Gain::G5 => 5.0,
            Gain::G10 => 10.0,
        }
    }
}

/// A type representing an ADC sample.
#[derive(Copy, Clone, Default)]
pub struct AdcCode(pub u16);

impl AdcCode {
    // The ADC has a differential input with a range of +/- 4.096 V and 16-bit resolution.
    // The gain into the two inputs is 1/5.
    pub const FULL_SCALE: f32 = 5.0 / 2.0 * 4.096;
    pub const VOLT_PER_LSB: f32 = -Self::FULL_SCALE / i16::MIN as f32;
    pub const LSB_PER_VOLT: f32 = 1. / Self::VOLT_PER_LSB;
}

impl From<u16> for AdcCode {
    /// Construct an ADC code from a provided binary (ADC-formatted) code.
    fn from(value: u16) -> Self {
        Self(value)
    }
}

impl From<i16> for AdcCode {
    /// Construct an ADC code from the stabilizer-defined code (i16 full range).
    fn from(value: i16) -> Self {
        Self(value as u16)
    }
}

impl From<AdcCode> for i16 {
    /// Get a stabilizer-defined code from the ADC code.
    fn from(code: AdcCode) -> i16 {
        code.0 as i16
    }
}

impl From<AdcCode> for u16 {
    /// Get an ADC-frmatted binary value from the code.
    fn from(code: AdcCode) -> u16 {
        code.0
    }
}

impl From<AdcCode> for f32 {
    /// Convert raw ADC codes to/from voltage levels.
    ///
    /// # Note
    /// This does not account for the programmable gain amplifier at the signal input.
    fn from(code: AdcCode) -> f32 {
        i16::from(code) as f32 * AdcCode::VOLT_PER_LSB
    }
}

impl TryFrom<f32> for AdcCode {
    type Error = ();

    fn try_from(voltage: f32) -> Result<AdcCode, ()> {
        let code = voltage * Self::LSB_PER_VOLT;
        if !(i16::MIN as f32..=i16::MAX as f32).contains(&code) {
            Err(())
        } else {
            Ok(AdcCode::from(code as i16))
        }
    }
}

/// Custom type for referencing DAC output codes.
/// The internal integer is the raw code written to the DAC output register.
#[derive(Copy, Clone, Default)]
pub struct DacCode(pub u16);
impl DacCode {
    // The DAC output range in bipolar mode (including the external output op-amp) is +/- 4.096
    // V with 16-bit resolution. The anti-aliasing filter has an additional gain of 2.5.
    pub const FULL_SCALE: f32 = 4.096 * 2.5;
    pub const VOLT_PER_LSB: f32 = -Self::FULL_SCALE / i16::MIN as f32;
    pub const LSB_PER_VOLT: f32 = 1. / Self::VOLT_PER_LSB;
}

impl TryFrom<f32> for DacCode {
    type Error = ();

    fn try_from(voltage: f32) -> Result<DacCode, ()> {
        let code = voltage * Self::LSB_PER_VOLT;
        if !(i16::MIN as f32..=i16::MAX as f32).contains(&code) {
            Err(())
        } else {
            Ok(DacCode::from(code as i16))
        }
    }
}

impl From<DacCode> for f32 {
    fn from(code: DacCode) -> f32 {
        i16::from(code) as f32 * DacCode::VOLT_PER_LSB
    }
}

impl From<DacCode> for i16 {
    fn from(code: DacCode) -> i16 {
        (code.0 as i16).wrapping_sub(i16::MIN)
    }
}

impl From<i16> for DacCode {
    /// Encode signed 16-bit values into DAC offset binary for a bipolar output configuration.
    fn from(value: i16) -> Self {
        Self(value.wrapping_add(i16::MIN) as u16)
    }
}

impl From<u16> for DacCode {
    /// Create a dac code from the provided DAC output code.
    fn from(value: u16) -> Self {
        Self(value)
    }
}

pub fn att_is_valid(attenuation: f32) -> bool {
    (0.0..=31.5).contains(&attenuation)
}
