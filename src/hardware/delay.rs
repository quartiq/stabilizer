//! Basic blocking delay
//!
//! This module provides a basic asm-based blocking delay.
//!
//! # Note
//! This implementation takes into account the Cortex-M7 CPU pipeline architecture to ensure delays
//! are at least as long as specified.
use embedded_hal_02::blocking::delay::DelayUs;

/// A basic delay implementation.
pub struct AsmDelay {
    frequency_us: u32,
}

impl AsmDelay {
    /// Create a new delay.
    ///
    /// # Args
    /// * `freq` - The CPU core frequency.
    pub fn new(freq: u32) -> AsmDelay {
        // Note: Frequencies are scaled by 2 to account for the M7 dual instruction pipeline.
        AsmDelay {
            frequency_us: (freq / 1_000_000) * 2,
        }
    }
}

impl<U> DelayUs<U> for AsmDelay
where
    U: Into<u32>,
{
    fn delay_us(&mut self, us: U) {
        cortex_m::asm::delay(self.frequency_us * us.into())
    }
}
