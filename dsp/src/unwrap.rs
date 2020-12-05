use serde::{Deserialize, Serialize};

/// Get phase wrap from x to y.
///
/// Phases are modulo integer overflow.
///
/// Args:
/// * `x`: Old phase sample
/// * `y`: New phase sample
///
/// Returns:
/// A tuple containg the (wrapped) phase difference and
/// one times the direction of the wrap.
pub fn get_wrap(x: i32, y: i32) -> (i32, i8) {
    let delta = y.wrapping_sub(x);
    let wrap = (delta >= 0) as i8 - (y >= x) as i8;
    (delta, wrap)
}

/// Phase unwrapper.
#[derive(Copy, Clone, Default, Deserialize, Serialize)]
pub struct Unwrapper {
    // last input
    x: i32,
    // last wraps
    v: i32,
}

impl Unwrapper {
    /// Unwrap a new sample from a phase sequence and update the
    /// unwrapper state.
    ///
    /// Args:
    /// * `x`: New phase sample
    ///
    /// Returns:
    /// A tuple containing the (wrapped) phase difference
    /// and the signed number of phase wraps corresponding to the new sample.
    pub fn update(&mut self, x: i32) -> (i32, i32) {
        let (dx, v) = get_wrap(self.x, x);
        self.x = x;
        self.v = self.v.wrapping_add(v as i32);
        (dx, self.v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn mini() {
        for (x0, x1, v) in [
            (0i32, 0i32, 0i8),
            (1, 1, 0),
            (-1, -1, 0),
            (1, -1, 0),
            (-1, 1, 0),
            (0, 0x7fff_ffff, 0),
            (-1, 0x7fff_ffff, -1),
            (0, -0x8000_0000, 0),
            (1, -0x8000_0000, 1),
            (-0x6000_0000, 0x6000_0000, -1),
            (0x6000_0000, -0x6000_0000, 1),
            (0x6000_0000, -0x6000_0000, 1),
            (0x6000_0000, -0x6000_0000, 1),
        ]
        .iter()
        {
            let (_dx, w) = get_wrap(*x0, *x1);
            assert_eq!(*v, w, " = get_wrap({:#x}, {:#x})", *x0, *x1);
        }
    }
}
