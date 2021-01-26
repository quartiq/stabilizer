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
    ff: i32, // current frequency estimate from frequency loop
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
        RPLL {
            dt2,
            t,
            ..Default::default()
        }
    }

    /// Advance the RPLL and optionally supply a new timestamp.
    ///
    /// Args:
    /// * input: Optional new timestamp (wrapping around at the i32 boundary).
    ///   There can be at most one timestamp per `update()` cycle (1 << dt2 counter cycles).
    /// * shift_frequency: Frequency lock settling time. 1 << shift_frequency is
    ///   frequency lock settling time in counter periods. The settling time must be larger
    ///   than the signal period to lock to.
    /// * shift_phase: Phase lock settling time. Usually one less than
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
        debug_assert!(shift_frequency > self.dt2);
        debug_assert!(shift_phase > self.dt2);
        // Advance phase
        self.y = self.y.wrapping_add(self.f);
        if let Some(x) = input {
            // Reference period in counter cycles
            let dx = x.wrapping_sub(self.x);
            // Store timestamp for next time.
            self.x = x;
            // Phase using the current frequency estimate
            let p_sig_long = (self.ff as i64).wrapping_mul(dx as i64);
            // Add half-up rounding bias and apply gain/attenuation
            let p_sig = (p_sig_long.wrapping_add(1i64 << (shift_frequency - 1))
                >> shift_frequency) as i32;
            // Reference phase (1 << dt2 full turns) with gain/attenuation applied
            let p_ref = 1i32 << (32 + self.dt2 - shift_frequency);
            // Update frequency lock
            self.ff = self.ff.wrapping_add(p_ref.wrapping_sub(p_sig));
            // Time in counter cycles between timestamp and "now"
            let dt = self.t.wrapping_sub(x);
            // Reference phase estimate "now"
            let y_ref = (self.f >> self.dt2).wrapping_mul(dt);
            // Phase error
            let dy = y_ref.wrapping_sub(self.y);
            // Current frequency estimate from frequency lock and phase error
            self.f = self.ff.wrapping_add(dy >> (shift_phase - self.dt2));
        }
        // Advance time
        self.t = self.t.wrapping_add(1 << self.dt2);
        (self.y, self.f)
    }
}
