use embedded_hal;
use serde::{Deserialize, Serialize};

use core::convert::TryFrom;
use enum_iterator::IntoEnumIterator;

#[derive(Copy, Clone, Debug, Serialize, Deserialize, IntoEnumIterator)]
pub enum Gain {
    G1 = 0b00,
    G2 = 0b01,
    G5 = 0b10,
    G10 = 0b11,
}

/// A programmable gain amplifier that allows for setting the gain via GPIO.
pub struct ProgrammableGainAmplifier<A0, A1> {
    a0: A0,
    a1: A1,
}

impl TryFrom<u8> for Gain {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        for gain in Gain::into_enum_iter() {
            if value == gain as u8 {
                return Ok(gain);
            }
        }

        Err(())
    }
}

impl<A0, A1> ProgrammableGainAmplifier<A0, A1>
where
    A0: embedded_hal::digital::v2::StatefulOutputPin,
    A0::Error: core::fmt::Debug,
    A1: embedded_hal::digital::v2::StatefulOutputPin,
    A1::Error: core::fmt::Debug,
{
    /// Construct a new programmable gain driver.
    ///
    /// Args:
    /// * `a0` - An output connected to the A0 input of the amplifier.
    /// * `a1` - An output connected to the A1 input of the amplifier.
    pub fn new(a0: A0, a1: A1) -> Self {
        let mut afe = Self { a0: a0, a1: a1 };

        afe.set_gain(Gain::G1);

        afe
    }

    /// Set the gain of the front-end.
    pub fn set_gain(&mut self, gain: Gain) {
        if (gain as u8 & 0b01) != 0 {
            self.a0.set_high().unwrap();
        } else {
            self.a0.set_low().unwrap();
        }

        if (gain as u8 & 0b10) != 0 {
            self.a1.set_high().unwrap()
        } else {
            self.a1.set_low().unwrap();
        }
    }

    /// Get the programmed gain of the analog front-end.
    pub fn get_gain(&self) -> Result<Gain, ()> {
        let mut code: u8 = 0;
        if self.a0.is_set_high().unwrap() {
            code |= 0b1;
        }
        if self.a1.is_set_high().unwrap() {
            code |= 0b10;
        }

        Gain::try_from(code)
    }
}
