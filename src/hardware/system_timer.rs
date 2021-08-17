///! System timer used for RTIC scheduling
///!
///! # Design
///! The SystemTimer is an RTIC monotonic timer that can be used for scheduling tasks in RTIC.
///! This timer is used in place of the cycle counter to allow the timer to tick at a slower rate
///! than the CPU clock. This allows for longer scheduling periods with less resolution. This is
///! needed for infrequent (e.g. multiple second) telemetry periods.
use hal::prelude::*;
use rtic::{
    time::{clock::Error, fraction::Fraction, Clock, Instant},
    Monotonic,
};
use stm32h7xx_hal as hal;

// A global buffer indicating how many times the internal counter has overflowed.
static mut OVERFLOWS: u32 = 0;

/// System timer used for implementing RTIC scheduling.
///
/// This implementation synchronizes access to the timer peripheral, so it is safe to copy/clone
/// and/or instantiate multiple timers. All timers will reference the same underlying hardware
/// clock.
///
/// This timer supports durations up to (just shy of) 5 days. Any duration larger than that will
/// wrap and behave incorrectly.
///
/// # Note
/// The system timer must be initialized before being used.
#[derive(Copy, Clone, Debug, Default)]
pub struct SystemTimer {}

impl SystemTimer {
    /// Initialize the system timer.
    ///
    /// # Args
    /// * `timer` - The hardware timer used for implementing the RTIC monotonic.
    pub fn initialize(mut timer: hal::timer::Timer<hal::device::TIM15>) {
        timer.pause();
        // Have the system timer operate at a tick rate of 10 KHz (100 µs per tick). With this
        // configuration and a 65535 period, we get an overflow once every 6.5 seconds.
        timer.set_tick_freq(10.khz());
        timer.apply_freq();

        timer.resume();
    }
}

impl Clock for SystemTimer {
    type T = u32;

    // The duration of each tick in seconds.
    const SCALING_FACTOR: Fraction = Fraction::new(1, 10_000);

    fn try_now(&self) -> Result<Instant<Self>, Error> {
        // Note(unsafe): Multiple interrupt contexts have access to the underlying timer, so care
        // is taken when reading and modifying register values.
        let regs = unsafe { &*hal::device::TIM15::ptr() };

        cortex_m::interrupt::free(|_cs| {
            loop {
                // Checking for overflows of the current counter must be performed atomically. Any
                // other task that is accessing the current time could potentially race for the
                // registers. Note that this is only required for writing to global state (e.g. timer
                // registers and overflow counter)
                // Check for overflows and clear the overflow bit atomically. This must be done in
                // a critical section to prevent race conditions on the status register.
                if regs.sr.read().uif().bit_is_set() {
                    regs.sr.modify(|_, w| w.uif().clear_bit());
                    unsafe {
                        OVERFLOWS = OVERFLOWS.wrapping_add(1);
                    }
                }

                let current_value = regs.cnt.read().bits();

                // Check that an overflow didn't occur since we just cleared the overflow bit. If
                // it did, loop around and retry.
                if regs.sr.read().uif().bit_is_clear() {
                    // Note(unsafe): We are in a critical section, so it is safe to read the
                    // global variable.
                    return unsafe {
                        Ok(Instant::new(
                            ((OVERFLOWS << 16) + current_value) as u32,
                        ))
                    };
                }
            }
        })
    }
}

impl Monotonic for SystemTimer {
    /// Reset the timer count.
    unsafe fn reset(&mut self) {
        // Note: The timer must be safely configured in `SystemTimer::initialize()`.
        let regs = &*hal::device::TIM15::ptr();

        OVERFLOWS = 0;
        regs.cnt.reset();
    }

    fn set_compare(&mut self, instant: &Instant<Self>) {
        let regs = unsafe { &*hal::device::TIM15::ptr() };

        // Note(cs): When operating on the timer registers, we must be atomic to prevent
        // pre-emption.
        cortex_m::interrupt::free(|_cs| {
            let now = self.try_now().unwrap();
            match now.checked_duration_until(instant) {
                // If the scheduled instant is too far in the future, we can't set an exact
                // deadline because it's too far in the future. Instead, just set a time in the
                // future and retry then.
                Some(duration) if duration.integer() > (1 << 16) => {
                    // Set the deadline and enable the interrupt
                    regs.ccr1.write(|w| w.ccr().bits(0xFFFF));
                    regs.dier.modify(|_, w| w.cc1ie().set_bit());
                }

                Some(_reachable_future_time) => {
                    // Else, the instant is within a single overflow. Set it for the future.
                    // Ignore the duration and truncate the overflow (top 16 bits) of the final
                    // deadline. We just checked that we can fit within a single overflow.
                    let deadline_ticks =
                        instant.duration_since_epoch().integer() as u16;

                    // Set the deadline and enable the interrupt
                    regs.ccr1.write(|w| w.ccr().bits(deadline_ticks));
                    regs.dier.modify(|_, w| w.cc1ie().set_bit());
                }

                // If the deadline has already passed, schedule an interrupt immediately.
                None => {
                    cortex_m::peripheral::NVIC::pend(hal::interrupt::TIM15);
                }
            }

            // Finally, perform a sanity check to ensure the scheduled deadline is still in
            // the future. This checks for a race condition of the timer stepping past the deadline
            // while we are configuring it. If we've proceeded past the dealdine, reschedule the
            // compare to occur immediately.
            if self.try_now().unwrap() > *instant {
                self.clear_compare_flag();
                cortex_m::peripheral::NVIC::pend(hal::interrupt::TIM15);
            }
        });
    }

    fn clear_compare_flag(&mut self) {
        let regs = unsafe { &*hal::device::TIM15::ptr() };

        // Note(cs): When operating on the timer registers, we must be atomic to prevent
        // pre-emption.
        cortex_m::interrupt::free(|_cs| {
            regs.sr.modify(|_, w| w.cc1if().clear_bit());
            regs.dier.modify(|_, w| w.cc1ie().clear_bit());
        });
    }
}
