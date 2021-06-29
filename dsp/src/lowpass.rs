/// Arbitrary order, high dynamic range, wide coefficient range,
/// lowpass filter implementation. DC gain is 1.
///
/// Type argument N is the filter order.
#[derive(Clone)]
pub struct Lowpass<const N: usize> {
    // IIR state storage
    y: [i32; N],
}

impl<const N: usize> Default for Lowpass<N> {
    fn default() -> Self {
        Lowpass { y: [0i32; N] }
    }
}

impl<const N: usize> Lowpass<N> {
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
        // This is an unrolled and optimized first-order IIR loop
        // that works for all possible time constants.
        // Note T-DF-I and the zeros at Nyquist.
        let mut x = x;
        for y in self.y.iter_mut() {
            let dy = x.saturating_sub(*y) >> k;
            *y += dy;
            x = *y - (dy >> 1);
        }
        x.saturating_add((self.y.len() as i32) << (k - 1).max(0))
    }
}
