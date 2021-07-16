///! System timer used for RTIC scheduling
///!
///! # Design
///! The SystemTimer is an RTIC monotonic timer that can be used for scheduling tasks in RTIC.
///! This timer is used in place of the cycle counter to allow the timer to tick at a slower rate
///! than the CPU clock. This allows for longer scheduling periods with less resolution. This is
///! needed for infrequent (e.g. multiple second) telemetry periods.
///!
///! # Limitations
///! This implementation relies on sufficient timer polling to not miss timer counter overflows. If
///! the timer is not polled often enough, it's possible that an overflow would be missed and time
///! would "halt" for a shore period of time. This could be fixed in the future by instead
///! listening for the overflow interrupt instead of polling the overflow state.
use hal::prelude::*;
use stm32h7xx_hal as hal;

// A global buffer indicating how many times the internal counter has overflowed.
static mut OVERFLOWS: u32 = 0;

/// System timer used for implementing RTIC scheduling.
///
/// # Note
/// The system timer must be initialized before being used.
pub struct SystemTimer {}

impl SystemTimer {
    /// Initialize the system timer.
    ///
    /// # Args
    /// * `timer` - The hardware timer used for implementing the RTIC monotonic.
    pub fn initialize(mut timer: hal::timer::Timer<hal::device::TIM15>) {
        timer.pause();
        // Have the system timer operate at a tick rate of 10KHz (100uS per tick). With this
        // configuration and a 65535 period, we get an overflow once every 6.5 seconds.
        timer.set_tick_freq(10.khz());
        timer.apply_freq();

        timer.resume();
    }

    /// Convert a provided number of seconds into timer ticks.
    pub fn ticks_from_secs(secs: u32) -> i32 {
        (secs * 10_000) as i32
    }
}

impl rtic::Monotonic for SystemTimer {
    /// Instants are stored in 32-bit signed integers. With a 10KHz tick rate, this means an
    /// instant can store up to ~59 hours of time before overflowing.
    type Instant = i32;

    /// The ratio of the CPU clock to the system timer.
    fn ratio() -> rtic::Fraction {
        rtic::Fraction {
            // At 10KHz with a 400MHz CPU clock, the CPU clock runs 40,000 times faster than
            // the system timer.
            numerator: 40_000,
            denominator: 1,
        }
    }

    /// Get the current time instant.
    ///
    /// # Note
    /// The time will overflow into -59 hours after the first 59 hours. This time value is intended
    /// for use in calculating time delta, and should not be used for timestamping purposes due to
    /// roll-over.
    fn now() -> i32 {
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
                        OVERFLOWS += 1;
                    }
                }

                let current_value = regs.cnt.read().bits();

                // Check that an overflow didn't occur since we just cleared the overflow bit. If
                // it did, loop around and retry.
                if regs.sr.read().uif().bit_is_clear() {
                    // Note(unsafe): We are in a critical section, so it is safe to read the
                    // global variable.
                    return unsafe {
                        ((OVERFLOWS << 16) + current_value) as i32
                    };
                }
            }
        })
    }

    /// Reset the timer count.
    unsafe fn reset() {
        // Note: The timer must be safely configured in `SystemTimer::initialize()`.
        let regs = &*hal::device::TIM15::ptr();

        OVERFLOWS = 0;
        regs.cnt.reset();
    }

    /// Get a timestamp correlating to zero time.
    fn zero() -> i32 {
        0
    }
}
