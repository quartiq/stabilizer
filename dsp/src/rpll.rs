/// Reciprocal PLL.
///
/// Consumes noisy, quantized timestamps of a reference signal and reconstructs
/// the phase and frequency of the update() invocations with respect to (and in units of
/// 1 << 32 of) that reference.
#[derive(Copy, Clone, Default)]
pub struct RPLL {
    dt2: u8, // 1 << dt2 is the counter rate to update() rate ratio
    t: i32,  // current counter time
    x: i32,  // previous timestamp
    ff: i32, // current frequency estimate from the frequency loop
    f: i32,  // current frequency estimate from both frequency and phase loop
    y: i32,  // current phase estimate
}

impl RPLL {
    /// Create a new RPLL instance.
    ///
    /// Args:
    /// * dt2: inverse update() rate. 1 << dt2 is the counter rate to update() rate ratio.
    /// * t: Counter time. Counter value at the first update() call. Typically 0.
    ///
    /// Returns:
    /// Initialized RPLL instance.
    pub fn new(dt2: u8, t: i32) -> RPLL {
        let mut pll = RPLL::default();
        pll.dt2 = dt2;
        pll.t = t;
        pll
    }

    /// Advance the RPLL and optionally supply a new timestamp.
    ///
    /// Args:
    /// * input: Optional new timestamp (wrapping around at the i32 boundary).
    /// * shift_frequency: Frequency lock settling time. 1 << shift_frequency is
    ///   frequency lock settling time in counter periods. The settling time must be larger
    ///   than the signal period to lock to.
    /// * shift_phase: Phase lock settling time. Usually the same as
    ///   `shift_frequency` (see there).
    ///
    /// Returns:
    /// A tuple containing the current phase (wrapping at the i32 boundary, pi) and
    /// frequency (wrapping at the i32 boundary, Nyquist) estimate.
    pub fn update(
        &mut self,
        input: Option<i32>,
        shift_frequency: u8,
        shift_phase: u8,
    ) -> (i32, i32) {
        debug_assert!(shift_frequency > 0);
        debug_assert!(shift_phase > 0);
        debug_assert!(32 + self.dt2 >= shift_frequency);
        self.y = self.y.wrapping_add(self.f as i32);
        if let Some(x) = input {
            self.ff = self.ff.wrapping_add((1i32 << 32 + self.dt2 - shift_frequency)
                - ((self.ff as i64).wrapping_mul(x.wrapping_sub(self.x) as i64)
                    + (1i64 << shift_frequency - 1) // half-up rounding bias
                    >> shift_frequency) as i32);
            self.f = self.ff.wrapping_add(
                ((self.f as i64).wrapping_mul(self.t.wrapping_sub(x) as i64)
                    .wrapping_sub((self.y as i64) << self.dt2)
                    // + (1i64 << shift_phase - 1)
                    >> shift_phase) as i32,
            );
            self.x = x;
        }
        self.t = self.t.wrapping_add(1 << self.dt2);
        (self.y, self.f)
    }
}
