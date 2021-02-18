use super::{Complex, FastInt, Lowpass, MulScaled};
use generic_array::typenum::U2;

#[derive(Clone, Default)]
pub struct Lockin {
    state: [Lowpass<U2>; 2],
}

impl Lockin {
    /// Update the lockin with a sample taken at a given phase.
    pub fn update(&mut self, sample: i32, phase: i32, k: u8) -> Complex<i32> {
        // Get the LO signal for demodulation and mix the sample;
        let mix = Complex::from_angle(phase).mul_scaled(sample);

        // Filter with the IIR lowpass,
        // return IQ (in-phase and quadrature) data.
        Complex {
            re: self.state[0].update(mix.re, k),
            im: self.state[1].update(mix.im, k),
        }
    }
}
