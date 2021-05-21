use serde::Serialize;

pub struct TaskTimer {
    // With a 400MHz CPU, total ticks will overflow after ~1500 years.
    total_ticks: u64,
    executions: u32,
    start_tick: Option<u32>,
}

#[derive(Serialize)]
pub struct TaskInfo {
    duration_avg: f32,
    count: u32,
}

impl TaskTimer {
    pub fn new() -> Self {
        Self {
            total_ticks: 0,
            executions: 0,
            start_tick: None,
        }
    }

    fn get_time() -> u32 {
        cortex_m::peripheral::DWT::get_cycle_count()
    }

    pub fn enter(&mut self) {
        let now = Self::get_time();
        self.start_tick.replace(now);
    }

    pub fn exit(&mut self) {
        let now = Self::get_time();

        // Note(unwrap): exit() must be called exactly once after `enter()`.
        let start = self.start_tick.take().unwrap();

        // Note(wrapping_sub): We can only measure durations up to the period of the underlying
        // timer. We cannot detect timer overflows.
        let duration = now.wrapping_sub(start);

        self.total_ticks += duration as u64;
        self.executions += 1;
    }

    pub fn get_info(&self, clock_frequency: f32) -> TaskInfo {
        let tick_period = 1.0 / clock_frequency;

        let average_ticks = if self.executions == 0 {
            0
        } else {
            self.total_ticks / (self.executions as u64)
        };

        TaskInfo {
            duration_avg: average_ticks as f32 * tick_period,
            count: self.executions,
        }
    }
}
