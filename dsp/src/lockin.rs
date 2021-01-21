use super::{
    iir_int,
    trig::{atan2, cossin},
    Complex,
};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Lockin {
    iir: iir_int::IIR,
    iir_state: [iir_int::IIRState; 2],
}

impl Lockin {
    pub fn new(_corner: u8) -> Self {
        Lockin {
            iir: iir_int::IIR::default(), // TODO: lowpass coefficients from corner
            iir_state: [iir_int::IIRState::default(); 2],
        }
    }

    pub fn update(&mut self, signal: i32, phase: i32) -> Complex<i32> {
        // Get the LO signal for demodulation.
        let m = cossin(phase.wrapping_neg());

        // Mix with the LO signal, filter with the IIR lowpass,
        // return IQ (in-phase and quadrature) data.
        // Note: 32x32 -> 64 bit multiplications are pretty much free.
        Complex(
            self.iir.update(
                &mut self.iir_state[0],
                ((signal as i64 * m.0 as i64) >> 32) as _,
            ),
            self.iir.update(
                &mut self.iir_state[1],
                ((signal as i64 * m.1 as i64) >> 32) as _,
            ),
        )
    }

    pub fn iq(&self) -> Complex<i32> {
        Complex(self.iir_state[0].get_y(0), self.iir_state[1].get_y(0))
    }

    pub fn power(&self) -> i32 {
        let iq = self.iq();
        (((iq.0 as i64) * (iq.0 as i64) + (iq.1 as i64) * (iq.1 as i64)) >> 32)
            as i32
    }

    pub fn phase(&self) -> i32 {
        let iq = self.iq();
        atan2(iq.1, iq.0)
    }
}
