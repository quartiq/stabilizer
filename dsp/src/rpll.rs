/// Reciprocal PLL.
///
/// Consumes noisy, quantized timestamps of a reference signal and reconstructs
/// the phase and frequency of the update() invocations with respect to (and in units of
/// 1 << 32 of) that reference.
/// In other words, `update()` rate ralative to reference frequency,
/// `u32::MAX` corresponding to both being equal.
#[derive(Copy, Clone, Default)]
pub struct RPLL {
    dt2: u8, // 1 << dt2 is the counter rate to update() rate ratio
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
    ///
    /// Returns:
    /// Initialized RPLL instance.
    pub fn new(dt2: u8) -> RPLL {
        RPLL {
            dt2,
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
    /// frequency.
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
            let p_sig_64 = self.ff as u64 * dx as u64;
            // Add half-up rounding bias and apply gain/attenuation
            let p_sig = ((p_sig_64 + (1u32 << (shift_frequency - 1)) as u64)
                >> shift_frequency) as u32;
            // Reference phase (1 << dt2 full turns) with gain/attenuation applied
            let p_ref = 1u32 << (32 + self.dt2 - shift_frequency);
            // Update frequency lock
            self.ff = self.ff.wrapping_add(p_ref.wrapping_sub(p_sig) as u32);
            // Time in counter cycles between timestamp and "now"
            let dt = (x.wrapping_neg() & ((1 << self.dt2) - 1)) as u32;
            // Reference phase estimate "now"
            let y_ref = (self.f >> self.dt2).wrapping_mul(dt) as i32;
            // Phase error with gain
            let dy = y_ref.wrapping_sub(self.y) >> (shift_phase - self.dt2);
            // Current frequency estimate from frequency lock and phase error
            self.f = self.ff.wrapping_add(dy as u32);
        }
        (self.y, self.f)
    }
}

#[cfg(test)]
mod test {
    use super::RPLL;
    use ndarray::prelude::*;
    use rand::{prelude::*, rngs::StdRng};
    use std::vec::Vec;

    #[test]
    fn make() {
        let _ = RPLL::new(8);
    }

    struct Harness {
        rpll: RPLL,
        dt2: u8,
        shift_frequency: u8,
        shift_phase: u8,
        noise: i32,
        period: i32,
        next: i32,
        next_noisy: i32,
        time: i32,
        rng: StdRng,
    }

    impl Harness {
        fn default() -> Self {
            Harness {
                rpll: RPLL::new(8),
                dt2: 8,
                shift_frequency: 9,
                shift_phase: 8,
                noise: 0,
                period: 333,
                next: 111,
                next_noisy: 111,
                time: 0,
                rng: StdRng::seed_from_u64(42),
            }
        }

        fn run(&mut self, n: usize) -> (Vec<f32>, Vec<f32>) {
            let mut y = Vec::<f32>::new();
            let mut f = Vec::<f32>::new();
            for _ in 0..n {
                let timestamp = if self.time - self.next_noisy >= 0 {
                    assert!(self.time - self.next_noisy < 1 << self.dt2);
                    self.next = self.next.wrapping_add(self.period);
                    let timestamp = self.next_noisy;
                    let p_noise = self.rng.gen_range(-self.noise..=self.noise);
                    self.next_noisy = self.next.wrapping_add(p_noise);
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
                let p_sig = fi as u64 * self.period as u64;
                // relative frequency error
                f.push(
                    p_sig.wrapping_sub(p_ref) as i64 as f32
                        / 2f32.powi(32 + self.dt2 as i32),
                );

                // advance time
                self.time = self.time.wrapping_add(1 << self.dt2);
            }
            (y, f)
        }

        fn measure(&mut self, n: usize, limits: [f32; 4]) {
            assert!(self.period >= 1 << self.dt2);
            assert!(self.dt2 <= self.shift_frequency);
            assert!(self.period < 1 << self.shift_frequency);
            assert!(self.period < 1 << self.shift_frequency + 1);
            let t_settle = (1 << self.shift_frequency - self.dt2 + 4)
                + (1 << self.shift_phase - self.dt2 + 4);
            self.run(t_settle);

            let (y, f) = self.run(n);
            let y = Array::from(y);
            let f = Array::from(f);
            // println!("{:?} {:?}", f, y);

            let fm = f.mean().unwrap();
            let fs = f.std_axis(Axis(0), 0.).into_scalar();
            let ym = y.mean().unwrap();
            let ys = y.std_axis(Axis(0), 0.).into_scalar();

            println!("f: {:.2e}±{:.2e}; y: {:.2e}±{:.2e}", fm, fs, ym, ys);

            let m = [fm, fs, ym, ys];

            print!("relative: ");
            for i in 0..m.len() {
                let rel = m[i].abs() / limits[i].abs();
                print!("{:.2e} ", rel);
                assert!(
                    rel <= 1.,
                    "idx {}, have |{}| > want {}",
                    i,
                    m[i],
                    limits[i]
                );
            }
            println!();
        }
    }

    #[test]
    fn default() {
        let mut h = Harness::default();

        h.measure(1 << 16, [1e-11, 4e-8, 2e-8, 2e-8]);
    }

    #[test]
    fn noisy() {
        let mut h = Harness::default();
        h.noise = 10;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        h.measure(1 << 16, [3e-9, 3e-6, 4e-4, 2e-4]);
    }

    #[test]
    fn narrow_fast() {
        let mut h = Harness::default();
        h.period = 990;
        h.next = 351;
        h.next_noisy = h.next;
        h.noise = 5;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        h.measure(1 << 16, [2e-9, 2e-6, 1e-3, 1e-4]);
    }

    #[test]
    fn narrow_slow() {
        let mut h = Harness::default();
        h.period = 1818181;
        h.next = 35281;
        h.next_noisy = h.next;
        h.noise = 1000;
        h.shift_frequency = 23;
        h.shift_phase = 22;

        h.measure(1 << 16, [2e-5, 6e-4, 2e-4, 2e-4]);
    }

    #[test]
    fn wide_fast() {
        let mut h = Harness::default();
        h.period = 990;
        h.next = 351;
        h.next_noisy = h.next;
        h.noise = 5;
        h.shift_frequency = 10;
        h.shift_phase = 9;

        h.measure(1 << 16, [5e-7, 3e-2, 2e-5, 2e-2]);
    }

    #[test]
    fn wide_slow() {
        let mut h = Harness::default();
        h.period = 1818181;
        h.next = 35281;
        h.next_noisy = h.next;
        h.noise = 1000;
        h.shift_frequency = 21;
        h.shift_phase = 20;

        h.measure(1 << 16, [2e-4, 6e-3, 2e-4, 2e-3]);
    }
}
