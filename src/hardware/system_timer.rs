use hal::prelude::*;
use stm32h7xx_hal as hal;

static mut OVERFLOWS: u32 = 0;

pub struct SystemTimer {}

impl SystemTimer {
    pub fn initialize(mut timer: hal::timer::Timer<hal::device::TIM15>) {
        timer.pause();
        // Have the system timer operate at a tick rate of 10KHz (100uS per tick). With this
        // configuration and a 65535 period, we get an overflow once every 6.5 seconds.
        timer.set_tick_freq(10.khz());
        timer.apply_freq();

        timer.resume();
    }

    pub fn ticks_from_secs(secs: u32) -> i32 {
        (secs * 10_000) as i32
    }
}

impl rtic::Monotonic for SystemTimer {
    type Instant = i32;

    fn ratio() -> rtic::Fraction {
        rtic::Fraction {
            numerator: 40_000,
            denominator: 1,
        }
    }

    fn now() -> i32 {
        let regs = unsafe { &*hal::device::TIM15::ptr() };

        loop {
            // Check for overflows
            if regs.sr.read().uif().bit_is_set() {
                regs.sr.modify(|_, w| w.uif().clear_bit());
                unsafe {
                    OVERFLOWS += 1;
                }
            }

            let current_value = regs.cnt.read().bits();

            // If the overflow is still unset, return our latest count, as it indicates we weren't
            // pre-empted.
            if regs.sr.read().uif().bit_is_clear() {
                unsafe {
                    return (OVERFLOWS * 65535 + current_value) as i32;
                }
            }
        }
    }

    unsafe fn reset() {
        // Note: The timer must be safely configured in `SystemTimer::initialize()`.
        let regs = &*hal::device::TIM15::ptr();

        regs.cnt.reset();
    }

    fn zero() -> i32 {
        0
    }
}
