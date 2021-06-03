use serde::{Deserialize, Serialize};

/// Type-II, sampled phase, discrete time PLL
///
/// This PLL tracks the frequency and phase of an input signal with respect to the sampling clock.
/// The transfer function is I^2,I from input phase to output phase and P,I from input phase to
/// output frequency.
///
/// The PLL locks to any frequency (i.e. it locks to the alias in the first Nyquist zone) and is
/// stable for any gain (1 <= shift <= 30). It has a single parameter that determines the loop
/// bandwidth in octave steps. The gain can be changed freely between updates.
///
/// The frequency and phase settling time constants for a frequency/phase jump are `1 << shift`
/// update cycles. The loop bandwidth is about `1/(2*pi*(1 << shift))` in units of the sample rate.
/// While the phase is being settled within one turn, there is a typically very small frequency
/// overshoot.
///
/// All math is naturally wrapping 32 bit integer. Phase and frequency are understood modulo that
/// overflow in the first Nyquist zone. Expressing the IIR equations in other ways (e.g. single
/// (T)-DF-{I,II} biquad/IIR) would break on overflow (i.e. every cycle).
///
/// There are no floating point rounding errors here. But there is integer quantization/truncation
/// error of the `shift` lowest bits leading to a phase offset for very low gains. Truncation
/// bias is applied. Rounding is "half up". The phase truncation error can be removed very
/// efficiently by dithering.
///
/// This PLL does not unwrap phase slips during lock acquisition. This can and should be
/// implemented elsewhere by unwrapping and scaling the input phase and un-scaling
/// and wrapping output phase and frequency. This affects dynamic range, gain, and noise accordingly.
///
/// The extension to I^3,I^2,I behavior to track chirps phase-accurately or to i64 data to
/// increase resolution for extremely narrowband applications is obvious.
#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct PLL {
    // last input phase
    x: i32,
    // filtered frequency
    f: i32,
    // filtered output phase
    y: i32,
}

impl PLL {
    /// Update the PLL with a new phase sample. This needs to be called (sampled) periodically.
    /// The signal's phase/frequency is reconstructed relative to the sampling period.
    ///
    /// Args:
    /// * `x`: New input phase sample.
    /// * `shift_frequency`: Frequency error scaling. The frequency gain per update is
    ///   `1/(1 << shift_frequency)`.
    /// * `shift_phase`: Phase error scaling. The phase gain is `1/(1 << shift_phase)`
    ///   per update. A good value is typically `shift_frequency - 1`.
    ///
    /// Returns:
    /// A tuple of instantaneous phase and frequency (the current phase increment).
    pub fn update(
        &mut self,
        x: i32,
        shift_frequency: u8,
        shift_phase: u8,
    ) -> (i32, i32) {
        debug_assert!((1..=30).contains(&shift_frequency));
        debug_assert!((1..=30).contains(&shift_phase));
        let e = x.wrapping_sub(self.f);
        self.f = self.f.wrapping_add(
            (1i32 << (shift_frequency - 1))
                .wrapping_add(e)
                .wrapping_sub(self.x)
                >> shift_frequency,
        );
        self.x = x;
        let f = self.f.wrapping_add(
            (1i32 << (shift_phase - 1))
                .wrapping_add(e)
                .wrapping_sub(self.y)
                >> shift_phase,
        );
        self.y = self.y.wrapping_add(f);
        (self.y, f)
    }

    /// Advance the PLL without providing a new timestamp.
    pub fn advance(&mut self) -> (i32, i32) {
        self.x = self.x.wrapping_add(self.f);
        self.y = self.y.wrapping_add(self.f);
        (self.y, self.f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mini() {
        let mut p = PLL::default();
        let (y, f) = p.update(0x10000, 8, 4);
        assert_eq!(y, 0x1100);
        assert_eq!(f, y);
    }

    #[test]
    fn converge() {
        let mut p = PLL::default();
        let f0 = 0x71f63049_i32;
        let shift = (10, 9);
        let n = 31 << shift.0 + 2;
        let mut x = 0i32;
        for i in 0..n {
            x = x.wrapping_add(f0);
            let (y, f) = p.update(x, shift.0, shift.1);
            if i > n / 4 {
                assert_eq!(f.wrapping_sub(f0).abs() <= 1, true);
            }
            if i > n / 2 {
                // The remaining error would be removed by dithering.
                assert_eq!(y.wrapping_sub(x).abs() < 1 << 18, true);
            }
        }
    }
}
