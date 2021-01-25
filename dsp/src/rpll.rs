#[derive(Copy, Clone, Default)]
pub struct RPLL {
    dt2: u8,
    t: i32,
    f2: i64,
    y1: i32,
    xj1: i32,
    f1: i32,
}

impl RPLL {
    pub fn new(dt2: u8) -> RPLL {
        let mut pll = RPLL::default();
        pll.dt2 = dt2;
        pll
    }

    pub fn update(
        &mut self,
        input: Option<i32>,
        shift_frequency: u8,
        shift_phase: u8,
    ) -> (i32, i32) {
        debug_assert!(shift_frequency > 0);
        debug_assert!(shift_phase > 0);
        debug_assert!(32 + self.dt2 >= shift_frequency);
        self.y1 = self.y1.wrapping_add(self.f1);
        if let Some(xj) = input {
            self.f2 = self.f2.wrapping_add((1i64 << 32 + self.dt2 - shift_frequency)
                - (self.f2.wrapping_mul(xj.wrapping_sub(self.xj1) as i64)
                    + (1i64 << shift_frequency - 1)
                    >> shift_frequency));
            self.f1 = (self.f2 as i32)
                .wrapping_add((self.f2.wrapping_mul(self.t.wrapping_sub(xj) as i64)
                    - ((self.y1 as i64) << self.dt2)
                    + (1i64 << shift_phase - 1)
                    >> shift_phase) as i32);
        }
        self.t = self.t.wrapping_add(1 << self.dt2);
        (self.y1, self.f1)
    }
}
