use super::{
    iir_int::{Vec5, IIR},
    Complex,
};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Lockin {
    iir: IIR,
    state: [Vec5; 2],
}

impl Lockin {
    /// Create a new Lockin with given IIR coefficients.
    pub fn new(ba: Vec5) -> Self {
        Self {
            iir: IIR {
                ba,
                ..Default::default()
            },
            state: [Vec5::default(); 2],
        }
    }

    /// Update the lockin with a sample taken at a given phase.
    pub fn update(&mut self, sample: i32, phase: i32) -> Complex<i32> {
        // Get the LO signal for demodulation.
        let lo = Complex::from_angle(phase);

        // Mix with the LO signal, filter with the IIR lowpass,
        // return IQ (in-phase and quadrature) data.
        // Note: 32x32 -> 64 bit multiplications are pretty much free.
        Complex(
            self.iir.update(
                &mut self.state[0],
                ((sample as i64 * lo.0 as i64) >> 32) as _,
            ),
            self.iir.update(
                &mut self.state[1],
                ((sample as i64 * lo.1 as i64) >> 32) as _,
            ),
        )
    }
}
