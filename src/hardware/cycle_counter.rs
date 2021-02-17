use rtic::cyccnt::{Duration, Instant, U32Ext};

use stm32h7xx_hal::time::Hertz;

pub struct CycleCounter {
    next_tick: Instant,
    ticks: u32,
    increment: Duration,
}

impl CycleCounter {
    pub fn new(
        mut dwt: cortex_m::peripheral::DWT,
        cpu_frequency: impl Into<Hertz>,
    ) -> Self {
        dwt.enable_cycle_counter();
        let increment =
            ((cpu_frequency.into().0 as f32 / 1000.0) as u32).cycles();

        Self {
            increment,
            ticks: 0,
            next_tick: Instant::now() + increment,
        }
    }

    pub fn current_ms(&mut self) -> u32 {
        let now = Instant::now();
        while now > self.next_tick {
            self.next_tick += self.increment;
            self.ticks += 1;
        }

        self.ticks
    }
}
