use super::{lowpass::Lowpass, Complex};
use generic_array::typenum::U2;

#[derive(Clone, Default)]
pub struct Lockin {
    state: [Lowpass<U2>; 2],
}

impl Lockin {
    /// Create a new Lockin with given IIR coefficients.
    pub fn new() -> Self {
        let lp = Lowpass::default();
        Self {
            state: [lp.clone(), lp],
        }
    }

    /// Update the lockin with a sample taken at a given phase.
    /// The lowpass has a gain of `1 << k`.
    pub fn update(&mut self, sample: i16, phase: i32, k: u8) -> Complex<i32> {
        // Get the LO signal for demodulation.
        let lo = Complex::from_angle(phase);

        // Mix with the LO signal
        let mix = lo * sample;

        // Filter with the IIR lowpass,
        // return IQ (in-phase and quadrature) data.
        Complex(
            self.state[0].update(mix.0, k),
            self.state[1].update(mix.1, k),
        )
    }
}
