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
    /// * `x`: Input data, needs `k` bits headroom.
    /// * `k`: Log2 time constant, 0..31.
    ///
    /// # Return
    /// Filtered output y, with gain of `1 << k`.
    pub fn update(&mut self, x: i32, k: u8) -> i32 {
        debug_assert!(k & 31 == k);
        // This is an unrolled and optimized first-order IIR loop
        // that works for all possible time constants.
        // Note DF-II and the zeros at Nyquist.
        let mut x = x << k;
        for y in self.y.iter_mut() {
            let dy = (x - *y + (1 << (k - 1))) >> k;
            *y += dy;
            x = *y - (dy >> 1);
        }
        x
    }
}
