use super::{Error, GpioPin};
use crate::hardware::I2c1Proxy;

pub enum IoExpander {
    Mcp23017(mcp230xx::Mcp230xx<I2c1Proxy, mcp230xx::Mcp23017>),
    Pca9539(tca9539::Pca9539<I2c1Proxy>),
}

impl From<GpioPin> for mcp230xx::Mcp23017 {
    fn from(x: GpioPin) -> Self {
        match x {
            GpioPin::Led4Green => Self::A0,
            GpioPin::Led5Red => Self::A1,
            GpioPin::Led6Green => Self::A2,
            GpioPin::Led7Red => Self::A3,
            GpioPin::Led8Green => Self::A4,
            GpioPin::Led9Red => Self::A5,
            GpioPin::AttLe0 => Self::B0,
            GpioPin::AttLe1 => Self::B1,
            GpioPin::AttLe2 => Self::B2,
            GpioPin::AttLe3 => Self::B3,
            GpioPin::AttRstN => Self::B5,
            GpioPin::OscEnN => Self::B6,
            GpioPin::ExtClkSel => Self::B7,
        }
    }
}

impl From<GpioPin> for tca9539::Pin {
    fn from(x: GpioPin) -> Self {
        match x {
            GpioPin::Led4Green => Self::P00,
            GpioPin::Led5Red => Self::P01,
            GpioPin::Led6Green => Self::P02,
            GpioPin::Led7Red => Self::P03,
            GpioPin::Led8Green => Self::P04,
            GpioPin::Led9Red => Self::P05,
            GpioPin::AttLe0 => Self::P10,
            GpioPin::AttLe1 => Self::P11,
            GpioPin::AttLe2 => Self::P12,
            GpioPin::AttLe3 => Self::P13,
            GpioPin::AttRstN => Self::P15,
            GpioPin::OscEnN => Self::P16,
            GpioPin::ExtClkSel => Self::P17,
        }
    }
}

#[derive(Copy, Clone, Debug)]
pub enum Level {
    High,
    Low,
}

impl Into<tca9539::Level> for Level {
    fn into(self) -> tca9539::Level {
        match self {
            Level::Low => tca9539::Level::Low,
            Level::High => tca9539::Level::High,
        }
    }
}

impl Into<mcp230xx::Level> for Level {
    fn into(self) -> mcp230xx::Level {
        match self {
            Level::Low => mcp230xx::Level::Low,
            Level::High => mcp230xx::Level::High,
        }
    }
}

pub enum Direction {
    Output,
    Input,
}

impl Into<tca9539::Direction> for Direction {
    fn into(self) -> tca9539::Direction {
        match self {
            Direction::Output => tca9539::Direction::Output,
            Direction::Input => tca9539::Direction::Input,
        }
    }
}

impl Into<mcp230xx::Direction> for Direction {
    fn into(self) -> mcp230xx::Direction {
        match self {
            Direction::Output => mcp230xx::Direction::Output,
            Direction::Input => mcp230xx::Direction::Input,
        }
    }
}

impl IoExpander {
    pub fn set_direction(
        &mut self,
        pin: GpioPin,
        direction: Direction,
    ) -> Result<(), Error> {
        match self {
            Self::Mcp23017(expander) => expander
                .set_direction(pin.into(), direction.into())
                .map_err(|_| Error::I2c),
            Self::Pca9539(expander) => expander
                .set_direction(pin.into(), direction.into())
                .map_err(|_| Error::I2c),
        }
    }

    pub fn set_level(
        &mut self,
        pin: GpioPin,
        level: Level,
    ) -> Result<(), Error> {
        match self {
            Self::Mcp23017(expander) => expander
                .set_gpio(pin.into(), level.into())
                .map_err(|_| Error::I2c),
            Self::Pca9539(expander) => expander
                .set_level(pin.into(), level.into())
                .map_err(|_| Error::I2c),
        }
    }
}
