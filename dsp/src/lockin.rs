use super::{cossin, iir_int, Complex};
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
}
