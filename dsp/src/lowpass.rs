use generic_array::{ArrayLength, GenericArray};

/// Arbitrary order, high dynamic range, wide coefficient range,
/// lowpass filter implementation. DC gain is 1.
///
/// Type argument N is the filter order.
#[derive(Clone, Default)]
pub struct Lowpass<N: ArrayLength<i32>> {
    // IIR state storage
    y: GenericArray<i32, N>,
}

impl<N: ArrayLength<i32>> Lowpass<N> {
    /// Update the filter with a new sample.
    ///
    /// # Args
    /// * `x`: Input data. Needs 1 bit headroom but will saturate cleanly beyond that.
    /// * `k`: Log2 time constant, 1..=31.
    ///
    /// # Return
    /// Filtered output y.
    pub fn update(&mut self, x: i32, k: u8) -> i32 {
        debug_assert!(k & 31 == k);
        debug_assert!((k - 1) & 31 == k - 1);
        // This is an unrolled and optimized first-order IIR loop
        // that works for all possible time constants.
        // Note T-DF-I and the zeros at Nyquist.
        let mut x = x;
        for y in self.y.iter_mut() {
            let dy = x.saturating_sub(*y) >> k;
            *y += dy;
            x = *y - (dy >> 1);
        }
        x.saturating_add((self.y.len() as i32) << (k - 1))
    }
}
