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
    /// * `k`: Cutoff, `u32::MAX` being Nyquist
    ///
    /// # Returns
    /// Filtered output y
    pub fn update(&mut self, x: i32, k: u32) -> i32 {
        let mut x1 = self.xy[0];
        self.xy[0] = x;
        let mut x0 = x;

        // This is an unrolled and optimized first-order IIR loop
        // that works for all possible time constants.
        for y1 in self.xy[1..].iter_mut() {
            // Optimized first order lowpass expression
            // Note the zero at Nyquist
            let mut y0 =
                ((x0 >> 1) as i64 + (x1 >> 1) as i64 - *y1 as i64) * k as i64;
            y0 += (*y1 as i64) << 32;
            y0 += 1i64 << 31; // Half-up rounding bias

            // Store and advance
            x0 = (y0 >> 32) as i32;
            x1 = *y1;
            *y1 = x0;
        }
        x0
    }
}
