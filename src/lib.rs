#![no_std]

#[macro_use]
extern crate log;

pub mod hardware;

// The number of ticks in the ADC sampling timer. The timer runs at 100MHz, so the step size is
// equal to 10ns per tick.
// Currently, the sample rate is equal to: Fsample = 100/256 MHz = 390.625 KHz
pub const ADC_SAMPLE_TICKS_LOG2: u16 = 8;
pub const ADC_SAMPLE_TICKS: u16 = 1 << ADC_SAMPLE_TICKS_LOG2;

// The desired ADC sample processing buffer size.
pub const SAMPLE_BUFFER_SIZE_LOG2: usize = 3;
pub const SAMPLE_BUFFER_SIZE: usize = 1 << SAMPLE_BUFFER_SIZE_LOG2;
