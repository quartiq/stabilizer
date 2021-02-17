use rtic::cyccnt::{Duration, Instant, U32Ext};

use stm32h7xx_hal::time::Hertz;

/// A simple clock for counting elapsed milliseconds.
pub struct CycleCounter {
    // The time of the next millisecond in the system.
    next_tick: Option<Instant>,

    // The number of elapsed milliseconds recorded.
    ticks: u32,

    // The increment amount of clock cycles for each elapsed millisecond.
    increment: Duration,
}

impl CycleCounter {
    /// Construct the cycle counting clock.
    ///
    /// # Args
    /// * `dwt` - The debug watch and trace unit of the CPU core.
    /// * `cpu_frequency` - The frequency that the cycle counter counts at.
    ///
    /// # Returns
    /// A clock that can be used for measuring elapsed milliseconds.
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
            next_tick: None,
        }
    }

    /// Get the current number of milliseconds elapsed in the system.
    ///
    /// # Note
    /// This function must be called more often than once per 10 seconds to prevent internal
    /// wrapping of the cycle counter.
    ///
    /// The internal millisecond accumulator will overflow just shy of every 50 days.
    ///
    /// This function does not start counting milliseconds until the very first invocation.
    ///
    /// # Returns
    /// The number of elapsed milliseconds since the system started.
    pub fn current_ms(&mut self) -> u32 {
        if self.next_tick.is_none() {
            self.next_tick = Some(Instant::now() + self.increment);
        }

        let now = Instant::now();
        while now > self.next_tick.unwrap() {
            *self.next_tick.as_mut().unwrap() += self.increment;
            self.ticks += 1;
        }

        self.ticks
    }
}
