use super::{lowpass::Lowpass, Complex};
use generic_array::typenum::U3;

#[derive(Clone, Default)]
pub struct Lockin {
    state: [Lowpass<U3>; 2],
    k: u8,
}

impl Lockin {
    /// Create a new Lockin with given IIR coefficients.
    pub fn new(k: u8) -> Self {
        let lp = Lowpass::default();
        Self {
            state: [lp.clone(), lp],
            k,
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
            self.state[0]
                .update(((sample as i64 * lo.0 as i64) >> 32) as _, self.k),
            self.state[1]
                .update(((sample as i64 * lo.1 as i64) >> 32) as _, self.k),
        )
    }
}
