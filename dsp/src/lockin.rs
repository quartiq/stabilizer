use super::{Complex, ComplexExt, Lowpass, MulScaled};
use generic_array::ArrayLength;

#[derive(Clone, Default)]
pub struct Lockin<N: ArrayLength<i32>> {
    state: [Lowpass<N>; 2],
}

impl<N: ArrayLength<i32>> Lockin<N> {
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
