use super::{iir_int, Complex};
use serde::{Deserialize, Serialize};

#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Lockin {
    iir: iir_int::IIR,
    iir_state: [iir_int::IIRState; 2],
}

impl Lockin {
    /// Create a new Lockin with given IIR coefficients.
    pub fn new(ba: &iir_int::IIRState) -> Self {
        let mut iir = iir_int::IIR::default();
        iir.ba.0.copy_from_slice(&ba.0);
        Lockin {
            iir,
            iir_state: [iir_int::IIRState::default(); 2],
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
                &mut self.iir_state[0],
                ((sample as i64 * lo.0 as i64) >> 32) as _,
            ),
            self.iir.update(
                &mut self.iir_state[1],
                ((sample as i64 * lo.1 as i64) >> 32) as _,
            ),
        )
    }

    /// Feed an iterator into the Lockin and return the latest I/Q data.
    /// Initial stample phase and frequency (phase increment between samples)
    /// are supplied.
    pub fn feed<I: IntoIterator<Item = i32>>(
        &mut self,
        signal: I,
        phase: i32,
        frequency: i32,
    ) -> Option<Complex<i32>> {
        let mut phase = phase;

        signal
            .into_iter()
            .map(|sample| {
                phase = phase.wrapping_add(frequency);
                self.update(sample, phase)
            })
            .last()
    }
}
