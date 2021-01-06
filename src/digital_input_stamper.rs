///! Digital Input 0 (DI0) reference clock timestamper
///!
///! This module provides a means of timestamping the rising edges of an external reference clock on
///! the DI0 with a timer value from TIM5.
///!
///! # Design
///! An input capture channel is configured on DI0 and fed into TIM5's capture channel 4. TIM5 is
///! then run in a free-running mode with a configured tick rate (PSC) and maximum count value
///! (ARR). Whenever an edge on DI0 triggers, the current TIM5 counter value is captured and
///! recorded as a timestamp. This timestamp can be either directly read from the timer channel or
///! can be collected asynchronously via DMA collection.
///!
///! To prevent silently discarding timestamps, the TIM5 input capture over-capture flag is
///! continually checked. Any over-capture event (which indicates an overwritten timestamp) then
///! triggers a panic to indicate the dropped timestamp so that design parameters can be adjusted.
///!
///! # Tradeoffs
///! It appears that DMA transfers can take a significant amount of time to disable (400ns) if they
///! are being prematurely stopped (such is the case here). As such, for a sample batch size of 1,
///! this can take up a significant amount of the total available processing time for the samples.
///! This module checks for any captured timestamps from the timer capture channel manually. In
///! this mode, the maximum input clock frequency supported is dependant on the sampling rate and
///! batch size.
///!
///! This module only supports DI0 for timestamping due to trigger constraints on the DIx pins. If
///! timestamping is desired in DI1, a separate timer + capture channel will be necessary.
use super::{hal, timers, ADC_SAMPLE_TICKS, SAMPLE_BUFFER_SIZE};

/// Calculate the period of the digital input timestampe timer.
///
/// # Note
/// The period returned will be 1 less than the required period in timer ticks. The value returned
/// can be immediately programmed into a hardware timer period register.
///
/// The period is calcualted to be some power-of-two multiple of the batch size, such that N batches
/// will occur between each timestamp timer overflow.
///
/// # Returns
/// A 32-bit value that can be programmed into a hardware timer period register.
pub fn calculate_timestamp_timer_period() -> u32 {
    // Calculate how long a single batch requires in timer ticks.
    let batch_duration_ticks: u64 =
        SAMPLE_BUFFER_SIZE as u64 * ADC_SAMPLE_TICKS as u64;

    // Calculate the largest power-of-two that is less than or equal to
    // `batches_per_overflow`.  This is completed by eliminating the least significant
    // bits of the value until only the msb remains, which is always a power of two.
    let batches_per_overflow: u64 =
        (1u64 + u32::MAX as u64) / batch_duration_ticks;
    let mut j = batches_per_overflow;
    while (j & (j - 1)) != 0 {
        j = j & (j - 1);
    }

    // Once the number of batches per timestamp overflow is calculated, we can figure out the final
    // period of the timestamp timer. The period is always 1 larger than the value configured in the
    // register.
    let period: u64 = batch_duration_ticks * j - 1u64;
    assert!(period <= u32::MAX as u64);

    period as u32
}

/// The timestamper for DI0 reference clock inputs.
pub struct InputStamper {
    _di0_trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
    capture_channel: timers::tim5::Channel4InputCapture,
}

impl InputStamper {
    /// Construct the DI0 input timestamper.
    ///
    /// # Args
    /// * `trigger` - The capture trigger input pin.
    /// * `timer_channel - The timer channel used for capturing timestamps.
    pub fn new(
        trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
        timer_channel: timers::tim5::Channel4,
    ) -> Self {
        // Utilize the TIM5 CH4 as an input capture channel - use TI4 (the DI0 input trigger) as the
        // capture source.
        let input_capture =
            timer_channel.into_input_capture(timers::CaptureTrigger::Input24);

        Self {
            capture_channel: input_capture,
            _di0_trigger: trigger,
        }
    }

    /// Start to capture timestamps on DI0.
    pub fn start(&mut self) {
        self.capture_channel.enable();
    }

    /// Get the latest timestamp that has occurred.
    ///
    /// # Note
    /// This function must be called sufficiently often. If an over-capture event occurs, this
    /// function will panic, as this indicates a timestamp was inadvertently dropped.
    ///
    /// To prevent timestamp loss, the batch size and sampling rate must be adjusted such that at
    /// most one timestamp will occur in each data processing cycle.
    pub fn latest_timestamp(&mut self) -> Option<u32> {
        self.capture_channel
            .latest_capture()
            .expect("DI0 timestamp overrun")
    }
}
