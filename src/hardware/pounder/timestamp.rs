///! ADC sample timestamper using external Pounder reference clock.
///!
///! # Design
///!
///! The pounder timestamper utilizes the pounder SYNC_CLK output as a fast external reference clock
///! for recording a timestamp for each of the ADC samples.
///!
///! To accomplish this, a timer peripheral is configured to be driven by an external clock input.
///! Due to the limitations of clock frequencies allowed by the timer peripheral, the SYNC_CLK input
///! is divided by 4. This clock then clocks the timer peripheral in a free-running mode with an ARR
///! (max count register value) configured to overflow once per ADC sample batch.
///!
///! Once the timer is configured, an input capture is configured to record the timer count
///! register. The input capture is configured to utilize an internal trigger for the input capture.
///! The internal trigger is selected such that when a sample is generated on ADC0, the input
///! capture is simultaneously triggered. This results in the input capture triggering identically
///! to when the ADC samples the input.
///!
///! Once the input capture is properly configured, a DMA transfer is configured to collect all of
///! timestamps. The DMA transfer collects 1 timestamp for each ADC sample collected. In order to
///! avoid potentially losing a timestamp for a sample, the DMA transfer operates in double-buffer
///! mode. As soon as the DMA transfer completes, the hardware automatically swaps over to a second
///! buffer to continue capturing. This alleviates timing sensitivities of the DMA transfer
///! schedule.
use stm32h7xx_hal as hal;

use hal::dma::{dma::DmaConfig, PeripheralToMemory, Transfer};

use crate::hardware::{design_parameters::SAMPLE_BUFFER_SIZE, timers};

// Three buffers are required for double buffered mode - 2 are owned by the DMA stream and 1 is the
// working data provided to the application. These buffers must exist in a DMA-accessible memory
// region. Note that AXISRAM is not initialized on boot, so their initial contents are undefined.
#[link_section = ".axisram.buffers"]
static mut BUF: [[u16; SAMPLE_BUFFER_SIZE]; 3] = [[0; SAMPLE_BUFFER_SIZE]; 3];

/// Software unit to timestamp stabilizer ADC samples using an external pounder reference clock.
pub struct Timestamper {
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    timer: timers::PounderTimestampTimer,
    transfer: Transfer<
        hal::dma::dma::Stream0<hal::stm32::DMA2>,
        timers::tim8::Channel1InputCapture,
        PeripheralToMemory,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
        hal::dma::DBTransfer,
    >,
}

impl Timestamper {
    /// Construct the pounder sample timestamper.
    ///
    /// # Note
    /// The DMA is immediately configured after instantiation. It will not collect any samples
    /// until the sample timer begins to cause input capture triggers.
    ///
    /// # Args
    /// * `timestamp_timer` - The timer peripheral used for capturing timestamps from.
    /// * `stream` - The DMA stream to use for collecting timestamps.
    /// * `capture_channel` - The input capture channel for collecting timestamps.
    /// * `sampling_timer` - The stabilizer ADC sampling timer.
    /// * `_clock_input` - The input pin for the external clock from Pounder.
    ///
    /// # Returns
    /// The new pounder timestamper in an operational state.
    pub fn new(
        mut timestamp_timer: timers::PounderTimestampTimer,
        stream: hal::dma::dma::Stream0<hal::stm32::DMA2>,
        capture_channel: timers::tim8::Channel1,
        sampling_timer: &mut timers::SamplingTimer,
        _clock_input: hal::gpio::gpioa::PA0<
            hal::gpio::Alternate<hal::gpio::AF3>,
        >,
    ) -> Self {
        let config = DmaConfig::default()
            .memory_increment(true)
            .circular_buffer(true)
            .double_buffer(true);

        // The sampling timer should generate a trigger output when CH1 comparison occurs.
        sampling_timer.generate_trigger(timers::TriggerGenerator::ComparePulse);

        // The timestamp timer trigger input should use TIM2 (SamplingTimer)'s trigger, which is
        // mapped to ITR1.
        timestamp_timer.set_trigger_source(timers::TriggerSource::Trigger1);

        // The capture channel should capture whenever the trigger input occurs.
        let input_capture = capture_channel
            .into_input_capture(timers::tim8::CaptureSource1::TRC);
        input_capture.listen_dma();

        // The data transfer is always a transfer of data from the peripheral to a RAM buffer.
        let data_transfer: Transfer<_, _, PeripheralToMemory, _, _> =
            Transfer::init(
                stream,
                input_capture,
                // Note(unsafe): BUF[0] and BUF[1] are "owned" by this peripheral.
                // They shall not be used anywhere else in the module.
                unsafe { &mut BUF[0] },
                unsafe { Some(&mut BUF[1]) },
                config,
            );

        Self {
            timer: timestamp_timer,
            transfer: data_transfer,

            // Note(unsafe): BUF[2] is "owned" by this peripheral. It shall not be used anywhere
            // else in the module.
            next_buffer: unsafe { Some(&mut BUF[2]) },
        }
    }

    /// Start the DMA transfer for collecting timestamps.
    #[allow(dead_code)]
    pub fn start(&mut self) {
        self.transfer
            .start(|capture_channel| capture_channel.enable());
    }

    /// Update the period of the underlying timestamp timer.
    #[allow(dead_code)]
    pub fn update_period(&mut self, period: u16) {
        self.timer.set_period_ticks(period);
    }

    /// Obtain a buffer filled with timestamps.
    ///
    /// # Returns
    /// A reference to the underlying buffer that has been filled with timestamps.
    #[allow(dead_code)]
    pub fn acquire_buffer(&mut self) -> &[u16; SAMPLE_BUFFER_SIZE] {
        // Wait for the transfer to fully complete before continuing.
        // Note: If a device hangs up, check that this conditional is passing correctly, as there is
        // no time-out checks here in the interest of execution speed.
        while !self.transfer.get_transfer_complete_flag() {}

        let next_buffer = self.next_buffer.take().unwrap();

        // Start the next transfer.
        let (prev_buffer, _, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer); // .unwrap_none() https://github.com/rust-lang/rust/issues/62633

        self.next_buffer.as_ref().unwrap()
    }
}
