use serde::{Deserialize, Serialize};

pub type IIRState = [i32; 5];

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
#[derive(Copy, Clone, Deserialize, Serialize)]
pub struct IIR {
    pub ba: IIRState,
    // pub y_offset: i32,
    // pub y_min: i32,
    // pub y_max: i32,
}

impl IIR {
    /// Coefficient fixed point: signed Q2.30.
    /// Tailored to low-passes PI, II etc.
    const SHIFT: u32 = 30;

    /// Feed a new input value into the filter, update the filter state, and
    /// return the new output. Only the state `xy` is modified.
    ///
    /// # Arguments
    /// * `xy` - Current filter state.
    /// * `x0` - New input.
    pub fn update(&self, xy: &mut IIRState, x0: i32) -> i32 {
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
