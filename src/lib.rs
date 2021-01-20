#![no_std]

#[macro_use]
extern crate log;

pub mod hardware;
pub mod server;

// The number of ticks in the ADC sampling timer. The timer runs at 100MHz, so the step size is
// equal to 10ns per tick.
// Currently, the sample rate is equal to: Fsample = 100/256 MHz = 390.625 KHz
const ADC_SAMPLE_TICKS: u16 = 256;

// The desired ADC sample processing buffer size.
const SAMPLE_BUFFER_SIZE: usize = 8;
