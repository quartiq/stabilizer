use core::f64::consts::PI;
use miniconf::StringSet;
use serde::Deserialize;

/// Generic vector for integer IIR filter.
/// This struct is used to hold the x/y input/output data vector or the b/a coefficient
/// vector.
pub type Vec5 = [i32; 5];

trait Coeff {
    /// Lowpass biquad filter using cutoff and sampling frequencies.  Taken from:
    /// https://webaudio.github.io/Audio-EQ-Cookbook/audio-eq-cookbook.html
    ///
    /// # Args
    /// * `f` - Corner frequency, or 3dB cutoff frequency (in units of sample rate).
    ///         This is only accurate for low corner frequencies less than ~0.01.
    /// * `q` - Quality factor (1/sqrt(2) for critical).
    /// * `k` - DC gain.
    ///
    /// # Returns
    /// 2nd-order IIR filter coefficients in the form [b0,b1,b2,a1,a2]. a0 is set to -1.
    fn lowpass(f: f64, q: f64, k: f64) -> Self;
}

impl Coeff for Vec5 {
    fn lowpass(f: f64, q: f64, k: f64) -> Self {
        // 3rd order Taylor approximation of sin and cos.
        let f = f * 2. * PI;
        let f2 = f * f * 0.5;
        let fcos = 1. - f2;
        let fsin = f * (1. - f2 / 3.);
        let alpha = fsin / (2. * q);
        // IIR uses Q2.30 fixed point
        let a0 = (1. + alpha) / (1 << IIR::SHIFT) as f64;
        let b0 = (k / 2. * (1. - fcos) / a0 + 0.5) as _;
        let a1 = (2. * fcos / a0 + 0.5) as _;
        let a2 = ((alpha - 1.) / a0 + 0.5) as _;

        [b0, 2 * b0, b0, a1, a2]
    }
}

fn macc(y0: i32, x: &[i32], a: &[i32], shift: u32) -> i32 {
    // Rounding bias, half up
    let y0 = ((y0 as i64) << shift) + (1 << (shift - 1));
    let y = x
        .iter()
        .zip(a)
        .map(|(x, a)| *x as i64 * *a as i64)
        .fold(y0, |y, xa| y + xa);
    (y >> shift) as i32
}

/// Integer biquad IIR
///
/// See `dsp::iir::IIR` for general implementation details.
/// Offset and limiting disabled to suit lowpass applications.
/// Coefficient scaling fixed and optimized.
#[derive(Copy, Clone, Default, Debug, StringSet, Deserialize)]
pub struct IIR {
    pub ba: Vec5,
    // pub y_offset: i32,
    // pub y_min: i32,
    // pub y_max: i32,
}

impl IIR {
    /// Coefficient fixed point format: signed Q2.30.
    /// Tailored to low-passes, PI, II etc.
    pub const SHIFT: u32 = 30;

    /// Feed a new input value into the filter, update the filter state, and
    /// return the new output. Only the state `xy` is modified.
    ///
    /// # Arguments
    /// * `xy` - Current filter state.
    /// * `x0` - New input.
    pub fn update(&self, xy: &mut Vec5, x0: i32) -> i32 {
        let n = self.ba.len();
        debug_assert!(xy.len() == n);
        // `xy` contains       x0 x1 y0 y1 y2
        // Increment time      x1 x2 y1 y2 y3
        // Shift               x1 x1 x2 y1 y2
        // This unrolls better than xy.rotate_right(1)
        xy.copy_within(0..n - 1, 1);
        // Store x0            x0 x1 x2 y1 y2
        xy[0] = x0;
        // Compute y0 by multiply-accumulate
        let y0 = macc(0, xy, &self.ba, IIR::SHIFT);
        // Limit y0
        // let y0 = y0.max(self.y_min).min(self.y_max);
        // Store y0            x0 x1 y0 y1 y2
        xy[n / 2] = y0;
        y0
    }
}

#[cfg(test)]
mod test {
    use super::{Coeff, Vec5};

    #[test]
    fn lowpass_gen() {
        let ba = Vec5::lowpass(1e-5, 1. / 2f64.sqrt(), 2.);
        println!("{:?}", ba);
    }
}
