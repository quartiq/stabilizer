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
/// The frequency and phase settling time constants for an (any) frequency jump are `1 << shift`
/// update cycles. The loop bandwidth is about `1/(2*pi*(1 << shift))` in units of the sample rate.
///
/// All math is naturally wrapping 32 bit integer. Phase and frequency are understood modulo that
/// overflow in the first Nyquist zone. Expressing the IIR equations in other ways (e.g. single
/// (T)-DF-{I,II} biquad/IIR) would break on overflow.
///
/// There are no floating point rounding errors here. But there is integer quantization/truncation
/// error of the `shift` lowest bits leading to a phase offset for very low gains. Truncation
/// bias is applied. Rounding is "half up". The phase truncation error can be removed very
/// efficiently by dithering.
///
/// This PLL does not unwrap phase slips during lock acquisition. This can and should be
/// implemented elsewhere by (down) scaling and then unwrapping the input phase and (up) scaling
/// and wrapping output phase and frequency. This affects dynamic range accordingly.
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
    /// Update the PLL with a new phase sample.
    ///
    /// Args:
    /// * `input`: New input phase sample.
    /// * `shift`: Error scaling. The frequency gain per update is `1/(1 << shift)`. The phase gain
    ///   is always twice the frequency gain.
    ///
    /// Returns:
    /// A tuple of instantaneous phase and frequency (the current phase increment).
    pub fn update(&mut self, x: i32, shift: u8) -> (i32, i32) {
        debug_assert!((1..=30).contains(&shift));
        let bias = 1i32 << shift;
        let e = x.wrapping_sub(self.f);
        self.f = self.f.wrapping_add(
            (bias >> 1).wrapping_add(e).wrapping_sub(self.x) >> shift,
        );
        self.x = x;
        let f = self.f.wrapping_add(
            bias.wrapping_add(e).wrapping_sub(self.y) >> (shift - 1),
        );
        self.y = self.y.wrapping_add(f);
        (self.y, f)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mini() {
        let mut p = PLL::default();
        let (y, f) = p.update(0x10000, 10);
        assert_eq!(y, 0xc2);
        assert_eq!(f, y);
    }

    #[test]
    fn converge() {
        let mut p = PLL::default();
        let f0 = 0x71f63049_i32;
        let shift = 10;
        let n = 31 << shift + 2;
        let mut x = 0i32;
        for i in 0..n {
            x = x.wrapping_add(f0);
            let (y, f) = p.update(x, shift);
            if i > n / 4 {
                assert_eq!(f.wrapping_sub(f0).abs() <= 1, true);
            }
            if i > n / 2 {
                // The remaining error is removed by dithering.
                assert_eq!(y.wrapping_sub(x).abs() < 1 << 18, true);
            }
        }
    }
}
