use serde::Serialize;

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize)]
pub struct State {
    x0: i32,
    count: u32,
    min: i32,
    max: i32,
    m1: i64,
    // Note: The variance computation is (almost) the most naive one.
    // Alternative algorithms (e.g. Welford) are less limited in dynamic
    // range but need a floating point division for each `update()`.
    //
    // Here we increase dynamic range by taking data relative to the first sample.
    // That works well in many cases. Still, `m2` overflows if `sum((x-x0)**2) > u64::MAX`.
    // There are further (unmentioned) constraints in `get()` and `get_scaled()`.
    m2: u64,
}

impl Default for State {
    fn default() -> Self {
        Self {
            x0: 0,
            count: 0,
            max: i32::MIN,
            min: i32::MAX,
            m1: 0,
            m2: 0,
        }
    }
}

impl State {
    pub fn update(&mut self, x: i32) {
        if self.count == 0 {
            self.x0 = x;
        }
        let x64 = (x - self.x0) as i64;
        self.count += 1;
        self.m1 += x64;
        self.m2 += (x64 * x64) as u64;
        self.max = self.max.max(x);
        self.min = self.min.min(x);
    }

    pub fn get(&self) -> Statistics {
        let mut stat = Statistics {
            mean: 0,
            var: 0,
            max: self.max,
            min: self.min,
        };
        if self.count != 0 {
            let mean = self.m1 / self.count as i64;
            stat.mean = mean as i32 + self.x0;
            stat.var = (self.m2 / self.count as u64) - (mean * mean) as u64;
        }
        stat
    }

    pub fn get_scaled(&self, scale: f32) -> ScaledStatistics {
        let mut stat = ScaledStatistics {
            mean: 0.,
            std: 0.,
            max: self.max as f32 * scale,
            min: self.min as f32 * scale,
        };
        if self.count != 0 {
            let c = 1. / self.count as f64;
            let mean = self.m1 as f64 * c;
            stat.mean = (mean as f32 + self.x0 as f32) * scale;
            let var = self.m2 as f64 * c - mean * mean;
            stat.std = libm::sqrtf(var as _) * scale;
        }
        stat
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Default, Serialize)]
pub struct Statistics {
    pub min: i32,
    pub max: i32,
    pub mean: i32,
    pub var: u64,
}

#[derive(Copy, Clone, PartialEq, Debug, Default, Serialize)]
pub struct ScaledStatistics {
    pub min: f32,
    pub max: f32,
    pub mean: f32,
    pub std: f32,
}
