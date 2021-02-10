use generic_array::{ArrayLength, GenericArray};

/// Arbitrary order, high dynamic range, wide coefficient range,
/// lowpass filter implementation.
///
/// Type argument N is the filter order + 1.
#[derive(Clone, Default)]
pub struct Lowpass<N: ArrayLength<i32>> {
    // IIR state storage
    xy: GenericArray<i32, N>,
}

impl<N: ArrayLength<i32>> Lowpass<N> {
    /// Update the filter with a new sample.
    ///
    /// # Args
    /// * `x`: Input data
    /// * `k`: Log2 time constant
    ///
    /// # Return
    /// Filtered output y
    pub fn update(&mut self, x: i32, k: u8) -> i32 {
        // This is an unrolled and optimized first-order IIR loop
        // that works for all possible time constants.
        // Note the zero(s) at Nyquist and the benign overflow (DF-I).
        let mut x0 = x;
        let mut x1 = self.xy[0];
        self.xy[0] = x;
        for y1 in self.xy[1..].iter_mut() {
            x0 = *y1 + (((x0 >> 1) + (x1 >> 1) - *y1 + (1 << (k - 1))) >> k);
            x1 = *y1;
            *y1 = x0;
        }
        x0
    }
}
