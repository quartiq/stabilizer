use bitbybit::bitenum;
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

/// A programmable gain amplifier that allows for setting the gain via GPIO.
pub struct ProgrammableGainAmplifier<P> {
    a: [P; 2],
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

impl<P> ProgrammableGainAmplifier<P>
where
    P: embedded_hal_1::digital::OutputPin,
{
    /// Construct a new programmable gain driver.
    ///
    /// Args:
    /// * `a0` - An output connected to the A0 input of the amplifier.
    /// * `a1` - An output connected to the A1 input of the amplifier.
    pub fn new(a: [P; 2]) -> Self {
        let mut afe = Self { a };
        afe.set_gain(Gain::default());
        afe
    }

    /// Set the gain of the front-end.
    pub fn set_gain(&mut self, gain: Gain) {
        let mut gain = gain as u8;
        for a in self.a.iter_mut() {
            a.set_state(((gain & 1) != 0).into()).unwrap();
            gain >>= 1;
        }
    }
}
