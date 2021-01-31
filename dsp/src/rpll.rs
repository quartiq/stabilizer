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
    ff: u32, // current frequency estimate from frequency loop
    f: u32,  // current frequency estimate from both frequency and phase loop
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
    ) -> (i32, u32) {
        debug_assert!(shift_frequency > self.dt2);
        debug_assert!(shift_phase >= self.dt2);
        // Advance phase
        self.y = self.y.wrapping_add(self.f as i32);
        if let Some(x) = input {
            // Reference period in counter cycles
            let dx = x.wrapping_sub(self.x);
            // Store timestamp for next time.
            self.x = x;
            // Phase using the current frequency estimate
            let p_sig_64 = (self.ff as u64).wrapping_mul(dx as u64);
            // Add half-up rounding bias and apply gain/attenuation
            let p_sig = (p_sig_64.wrapping_add(1u64 << (shift_frequency - 1))
                >> shift_frequency) as i32;
            // Reference phase (1 << dt2 full turns) with gain/attenuation applied
            let p_ref = 1i32 << (32 + self.dt2 - shift_frequency);
            // Update frequency lock
            self.ff = self.ff.wrapping_add(p_ref.wrapping_sub(p_sig) as u32);
            // Time in counter cycles between timestamp and "now"
            let dt = self.t.wrapping_sub(x);
            // Reference phase estimate "now"
            let y_ref = ((self.f >> self.dt2) as i32).wrapping_mul(dt);
            // Phase error
            let dy = y_ref.wrapping_sub(self.y);
            // Current frequency estimate from frequency lock and phase error
            self.f = self
                .ff
                .wrapping_add((dy >> (shift_phase - self.dt2)) as u32);
        }
        // Advance time
        self.t = self.t.wrapping_add(1 << self.dt2);
        (self.y, self.f)
    }
}

#[cfg(test)]
mod test {
    use super::RPLL;
    use ndarray::prelude::*;
    use ndarray_stats::QuantileExt;
    use rand::{prelude::*, rngs::StdRng};
    use std::vec::Vec;

    #[test]
    fn make() {
        let _ = RPLL::new(8, 0);
    }

    struct Harness {
        rpll: RPLL,
        dt2: u8,
        shift_frequency: u8,
        shift_phase: u8,
        noise: i32,
        period: i32,
        next: i32,
        time: i32,
        rng: StdRng,
    }

    impl Harness {
        fn default() -> Self {
            Harness {
                rpll: RPLL::new(8, 0),
                dt2: 8,
                shift_frequency: 9,
                shift_phase: 8,
                noise: 0,
                period: 333,
                next: 111,
                time: 0,
                rng: StdRng::seed_from_u64(42),
            }
        }

        fn run(&mut self, n: usize) -> (Vec<f32>, Vec<f32>) {
            let mut y = Vec::<f32>::new();
            let mut f = Vec::<f32>::new();
            for _ in 0..n {
                let timestamp = if self.time - self.next >= 0 {
                    let p_noise = self.rng.gen_range(-self.noise..=self.noise);
                    let timestamp = self.next.wrapping_add(p_noise);
                    self.next = self.next.wrapping_add(self.period);
                    Some(timestamp)
                } else {
                    None
                };
                let (yi, fi) = self.rpll.update(
                    timestamp,
                    self.shift_frequency,
                    self.shift_phase,
                );
                let y_ref = (self.time.wrapping_sub(self.next) as i64
                    * (1i64 << 32)
                    / self.period as i64) as i32;
                // phase error
                y.push(yi.wrapping_sub(y_ref) as f32 / 2f32.powi(32));
                let p_ref = 1 << 32 + self.dt2;
                let p_sig = fi as i64 * self.period as i64;
                // relative frequency error
                f.push(p_sig.wrapping_sub(p_ref) as f32 / 2f32.powi(32));
                // advance time
                self.time = self.time.wrapping_add(1 << self.dt2);
            }
            (y, f)
        }

        fn measure(&mut self, n: usize) -> (f32, f32, f32, f32) {
            let t_settle = (1 << self.shift_frequency - self.dt2 + 4)
                + (1 << self.shift_phase - self.dt2 + 4);
            self.run(t_settle);

            let (y, f) = self.run(n);
            let y = Array::from(y);
            let f = Array::from(f);

            let fm = f.mean().unwrap();
            let fs = f.std_axis(Axis(0), 0.).into_scalar();
            let ym = y.mean().unwrap();
            let ys = y.std_axis(Axis(0), 0.).into_scalar();

            println!("f: {:.2e}±{:.2e}; y: {:.2e}±{:.2e}", fm, fs, ym, ys);
            (fm, fs, ym, ys)
        }
    }

    #[test]
    fn default() {
        let mut h = Harness::default();

        let (fm, fs, ym, ys) = h.measure(1 << 16);
        assert!(fm.abs() < 1e-9);
        assert!(fs.abs() < 8e-6);
        assert!(ym.abs() < 2e-8);
        assert!(ys.abs() < 2e-8);
    }

    #[test]
    fn noisy() {
        let mut h = Harness::default();
        h.noise = 10;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        let (fm, fs, ym, ys) = h.measure(1 << 16);
        assert!(fm.abs() < 1e-6);
        assert!(fs.abs() < 6e-4);
        assert!(ym.abs() < 2e-4);
        assert!(ys.abs() < 2e-4);
    }

    #[test]
    fn narrow_fast() {
        let mut h = Harness::default();
        h.period = 990;
        h.noise = 5;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        let (fm, fs, ym, ys) = h.measure(1 << 16);
        assert!(fm.abs() < 7e-6);
        assert!(fs.abs() < 6e-4);
        assert!(ym.abs() < 1e-3);
        assert!(ys.abs() < 1e-4);
    }

    /*#[test]
    fn narrow_slow() {
        let mut h = Harness::default();
        h.period = 1818181;
        h.noise = 1800;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        let (fm, fs, ym, ys) = h.measure(1 << 16);
        assert!(fm.abs() < 1e-8);
        assert!(fs.abs() < 6e-4);
        assert!(ym.abs() < 2e-4);
        assert!(ys.abs() < 2e-4);
    }
    */
}
