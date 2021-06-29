use serde::{Deserialize, Serialize};

/// Subtract `y - x` with signed overflow.
///
/// This is very similar to `i32::overflowing_sub(y, x)` except that the
/// overflow indicator is not a boolean but the signum of the overflow.
/// Additionally it's typically faster.
///
/// Returns:
/// A tuple containg the (wrapped) difference `y - x` and the signum of the
/// overflow.
#[inline(always)]
pub fn overflowing_sub(y: i32, x: i32) -> (i32, i8) {
    let delta = y.wrapping_sub(x);
    let wrap = (delta >= 0) as i8 - (y >= x) as i8;
    (delta, wrap)
}

/// Combine high and low i32 into a single downscaled i32, saturating monotonically.
///
/// Args:
/// `lo`: LSB i32 to scale down by `shift` and range-extend with `hi`
/// `hi`: MSB i32 to scale up and extend `lo` with. Output will be clipped if
///     `hi` exceeds the output i32 range.
/// `shift`: Downscale `lo` by that many bits. Values from 1 to 32 inclusive
///     are valid.
pub fn saturating_scale(lo: i32, hi: i32, shift: u32) -> i32 {
    debug_assert!(shift > 0);
    debug_assert!(shift <= 32);
    let hi_range = -1 << (shift - 1);
    if hi <= hi_range {
        i32::MIN - hi_range
    } else if -hi <= hi_range {
        hi_range - i32::MIN
    } else {
        (lo >> shift) + (hi << (32 - shift))
    }
}

/// Overflow unwrapper.
///
/// This is unwrapping as in the phase and overflow unwrapping context, not
/// unwrapping as in the `Result`/`Option` context.
#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Unwrapper {
    // last input
    x: i32,
    // last wraps
    w: i32,
}

impl Unwrapper {
    /// Unwrap a new sample from a sequence and update the unwrapper state.
    ///
    /// Args:
    /// * `x`: New sample
    ///
    /// Returns:
    /// A tuple containing the (wrapped) difference `x - x_old` and the
    /// signed number of wraps accumulated by the new sample.
    pub fn update(&mut self, x: i32) -> (i32, i32) {
        let (dx, dw) = overflowing_sub(x, self.x);
        self.x = x;
        self.w = self.w.wrapping_add(dw as i32);
        (dx, self.w)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn overflowing_sub_correctness() {
        for (x0, x1, v) in [
            (0i32, 0i32, 0i8),
            (0, 1, 0),
            (0, -1, 0),
            (1, 0, 0),
            (-1, 0, 0),
            (0, 0x7fff_ffff, 0),
            (-1, 0x7fff_ffff, -1),
            (-2, 0x7fff_ffff, -1),
            (-1, -0x8000_0000, 0),
            (0, -0x8000_0000, 0),
            (1, -0x8000_0000, 1),
            (-0x6000_0000, 0x6000_0000, -1),
            (0x6000_0000, -0x6000_0000, 1),
            (-0x4000_0000, 0x3fff_ffff, 0),
            (-0x4000_0000, 0x4000_0000, -1),
            (-0x4000_0000, 0x4000_0001, -1),
            (0x4000_0000, -0x3fff_ffff, 0),
            (0x4000_0000, -0x4000_0000, 0),
            (0x4000_0000, -0x4000_0001, 1),
        ]
        .iter()
        {
            let (dx, w) = overflowing_sub(*x1, *x0);
            assert_eq!(*v, w, " = overflowing_sub({:#x}, {:#x})", *x0, *x1);
            let (dx0, w0) = x1.overflowing_sub(*x0);
            assert_eq!(w0, w != 0);
            assert_eq!(dx, dx0);
        }
    }

    #[test]
    fn saturating_scale_correctness() {
        let shift = 8;
        for (lo, hi, res) in [
            (0i32, 0i32, 0i32),
            (0, 1, 0x0100_0000),
            (0, -1, -0x0100_0000),
            (0x100, 0, 1),
            (-1 << 31, 0, -1 << 23),
            (0x7fffffff, 0, 0x007f_ffff),
            (0x7fffffff, 1, 0x0017f_ffff),
            (-0x7fffffff, -1, -0x0180_0000),
            (0x1234_5600, 0x7f, 0x7f12_3456),
            (0x1234_5600, -0x7f, -0x7f00_0000 + 0x12_3456),
            (0, 0x7f, 0x7f00_0000),
            (0, 0x80, 0x7fff_ff80),
            (0, -0x7f, -0x7f00_0000),
            (0, -0x80, -0x7fff_ff80),
            (0x7fff_ffff, 0x7f, 0x7f7f_ffff),
            (-0x8000_0000, 0x7f, 0x7e80_0000),
            (-0x8000_0000, -0x7f, -0x7f80_0000),
            (0x7fff_ffff, -0x7f, -0x7e80_0001),
            (0x100, 0x7f, 0x7f00_0001),
            (0, -0x80, -0x7fff_ff80),
            (-1 << 31, 0x80, 0x7fff_ff80),
            (-1 << 31, -0x80, -0x7fff_ff80),
        ]
        .iter()
        {
            let s = saturating_scale(*lo, *hi, shift);
            assert_eq!(
                *res, s,
                "{:#x} != {:#x} = saturating_scale({:#x}, {:#x}, {:#x})",
                *res, s, *lo, *hi, shift
            );
        }
    }
}
