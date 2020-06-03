use embedded_hal;
use serde::{Serialize, Deserialize};

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub enum Gain {
    G1 = 0b00,
    G2 = 0b01,
    G5 = 0b10,
    G10 = 0b11
}

pub struct ProgrammableGainAmplifier<A0, A1> {
    a0: A0,
    a1: A1
}

impl<A0, A1> ProgrammableGainAmplifier<A0, A1>
where
    A0: embedded_hal::digital::v2::StatefulOutputPin,
    A0::Error: core::fmt::Debug,
    A1: embedded_hal::digital::v2::StatefulOutputPin,
    A1::Error: core::fmt::Debug,
{
    pub fn new(a0: A0, a1: A1) -> Self
    {
        let mut afe = Self { a0: a0, a1: a1};

        afe.set_gain(Gain::G1);

        afe
    }

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

    pub fn get_gain(&self) -> Gain {
        let lsb_set = self.a0.is_set_high().unwrap();
        let msb_set = self.a1.is_set_high().unwrap();

        if msb_set {
            if lsb_set {
               Gain::G10
            } else {
                Gain::G5
            }
        } else {
            if lsb_set {
                Gain::G2
            } else {
                Gain::G1
            }
        }
    }
}
