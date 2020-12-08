///! Digital Input 0 (DI0) reference clock timestamper
///!
///! This module provides a means of timestamping the rising edges of an external reference clock on
///! the DI0 with a timer value from TIM5.
///!
///! This module only supports input clocks on DI0 and may or may not utilize DMA to collect
///! timestamps.
///!
///! # Design
///! An input capture channel is configured on DI0 and fed into TIM5's capture channel 4. TIM5 is
///! then run in a free-running mode with a configured frequency and period. Whenever an edge on DI0
///! triggers, the current TIM5 counter value is captured and recorded as a timestamp. This timestamp can be
///! either directly read from the timer channel or can be collected asynchronously via DMA
///! collection.
///!
///! When DMA is used for timestamp collection, a DMA transfer is configured to collect as many
///! timestamps as there are samples, but it is intended that this DMA transfer should never
///! complete. Instead, when all samples are collected, the module pauses the DMA transfer and
///! checks to see how many timestamps were collected. These collected timestamps are then returned
///! for further processing.
///!
///! To prevent silently discarding timestamps, the TIM5 input capture over-capture interrupt is
///! used. Any over-capture event (which indicates an overwritten timestamp) then generates an ISR
///! which handles the over-capture.
///!
///! # Tradeoffs
///! It appears that DMA transfers can take a significant amount of time to disable (400ns) if they
///! are being prematurely stopped (such is the case here). As such, for a sample batch size of 1,
///! this can take up a significant amount of the total available processing time for the samples.
///! To avoid this, the module does not use DMA when the sample batch size is one. Instead, the
///! module manually checks for any captured timestamps from the timer capture channel manually. In
///! this mode, the maximum input clock frequency supported is equal to the configured sample rate.
///!
///! There is a small window while the DMA buffers are swapped where a timestamp could potentially
///! be lost. To prevent this, the `acuire_buffer()` method should not be pre-empted. Any lost
///! timestamp will trigger an over-capture interrupt.
use super::{
    hal, timers, DmaConfig, PeripheralToMemory, Transfer, SAMPLE_BUFFER_SIZE,
};

// The DMA buffers must exist in a location where DMA can access. By default, RAM uses DTCM, which
// is off-limits to the normal DMA peripheral. Instead, we use AXISRAM.
#[link_section = ".axisram.buffers"]
static mut BUF: [[u32; SAMPLE_BUFFER_SIZE]; 2] = [[0; SAMPLE_BUFFER_SIZE]; 2];

/// The timestamper for DI0 reference clock inputs.
pub struct InputStamper {
    _di0_trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
    next_buffer: Option<&'static mut [u32; SAMPLE_BUFFER_SIZE]>,
    transfer: Option<
        Transfer<
            hal::dma::dma::Stream6<hal::stm32::DMA1>,
            timers::tim5::Channel4InputCapture,
            PeripheralToMemory,
            &'static mut [u32; SAMPLE_BUFFER_SIZE],
        >,
    >,
    capture_channel: Option<timers::tim5::Channel4InputCapture>,
}

impl InputStamper {
    /// Construct the DI0 input timestamper.
    ///
    /// # Args
    /// * `trigger` - The capture trigger input pin.
    /// * `stream` - The DMA stream to use for collecting timestamps.
    /// * `timer_channel - The timer channel used for capturing timestamps.
    /// * `batch_size` - The number of samples collected per processing batch.
    pub fn new(
        trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
        stream: hal::dma::dma::Stream6<hal::stm32::DMA1>,
        timer_channel: timers::tim5::Channel4,
        batch_size: usize,
    ) -> Self {
        // Utilize the TIM5 CH4 as an input capture channel - use TI4 (the DI0 input trigger) as the
        // capture source.
        let input_capture =
            timer_channel.to_input_capture(timers::tim5::CC4S_A::TI4);

        // Listen for over-capture events, which indicates an over-run of DI0 timestamps.
        input_capture.listen_overcapture();

        // For small batch sizes, the overhead of DMA can become burdensome to the point where
        // timing is not met. The DMA requires 500ns overhead, whereas a direct register read only
        // requires ~80ns. When batches of 2-or-greater are used, use a DMA-based approach.
        let (transfer, input_capture) = if batch_size >= 2 {
            input_capture.listen_dma();

            // Set up the DMA transfer.
            let dma_config = DmaConfig::default().memory_increment(true);

            let mut timestamp_transfer: Transfer<_, _, PeripheralToMemory, _> =
                Transfer::init(
                    stream,
                    input_capture,
                    unsafe { &mut BUF[0] },
                    None,
                    dma_config,
                );

            timestamp_transfer.start(|_| {});
            (Some(timestamp_transfer), None)
        } else {
            (None, Some(input_capture))
        };

        Self {
            next_buffer: unsafe { Some(&mut BUF[1]) },
            transfer,
            capture_channel: input_capture,
            _di0_trigger: trigger,
        }
    }

    /// Get all of the timestamps that have occurred during the last processing cycle.
    pub fn acquire_buffer(&mut self) -> &[u32] {
        // If we are using DMA, finish the transfer and swap over buffers.
        if self.transfer.is_some() {
            let next_buffer = self.next_buffer.take().unwrap();

            let (prev_buffer, _, remaining_transfers) = self
                .transfer
                .as_mut()
                .unwrap()
                .next_transfer(next_buffer)
                .unwrap();
            let valid_count = prev_buffer.len() - remaining_transfers;

            self.next_buffer.replace(prev_buffer);

            // Note that we likely didn't finish the transfer, so only return the number of
            // timestamps actually collected.
            &self.next_buffer.as_ref().unwrap()[..valid_count]
        } else {
            // If we aren't using DMA, just manually check the input capture channel for a
            // timestamp.
            match self.capture_channel.as_mut().unwrap().latest_capture() {
                Some(stamp) => {
                    self.next_buffer.as_mut().unwrap()[0] = stamp;
                    &self.next_buffer.as_ref().unwrap()[..1]
                }
                None => &[],
            }
        }
    }
}
