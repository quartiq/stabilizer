use serde::{Deserialize, Serialize};

use core::convert::TryFrom;
use num_enum::TryFromPrimitive;

#[derive(
    Copy, Clone, Debug, Serialize, Deserialize, TryFromPrimitive, Default,
)]
#[repr(u8)]
pub enum Gain {
    #[default]
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

impl Gain {
    /// Get the AFE gain as a numerical value.
    pub fn as_multiplier(self) -> f32 {
        match self {
            Gain::G1 => 1.0,
            Gain::G2 => 2.0,
            Gain::G5 => 5.0,
            Gain::G10 => 10.0,
        }
    }
}

impl<A0, A1> ProgrammableGainAmplifier<A0, A1>
where
    A0: embedded_hal_02::digital::v2::StatefulOutputPin,
    A0::Error: core::fmt::Debug,
    A1: embedded_hal_02::digital::v2::StatefulOutputPin,
    A1::Error: core::fmt::Debug,
{
    /// Construct a new programmable gain driver.
    ///
    /// Args:
    /// * `a0` - An output connected to the A0 input of the amplifier.
    /// * `a1` - An output connected to the A1 input of the amplifier.
    pub fn new(a0: A0, a1: A1) -> Self {
        let mut afe = Self { a0, a1 };

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
    pub fn get_gain(&self) -> Gain {
        let mut code: u8 = 0;
        if self.a0.is_set_high().unwrap() {
            code |= 0b1;
        }
        if self.a1.is_set_high().unwrap() {
            code |= 0b10;
        }

        // NOTE(unwrap): All possibilities covered.
        Gain::try_from(code).unwrap()
    }
}
