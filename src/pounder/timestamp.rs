///! ADC sample timestamper using external Pounder reference clock.
use stm32h7xx_hal as hal;

use hal::dma::{dma::DmaConfig, PeripheralToMemory, Transfer};

use crate::{timers, SAMPLE_BUFFER_SIZE};

#[link_section = ".axisram.buffers"]
static mut BUF: [[u16; SAMPLE_BUFFER_SIZE]; 3] = [[0; SAMPLE_BUFFER_SIZE]; 3];

pub struct Timestamper {
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    timer: timers::PounderTimestampTimer,
    transfer: Transfer<
        hal::dma::dma::Stream7<hal::stm32::DMA1>,
        timers::tim8::Channel1InputCapture,
        PeripheralToMemory,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
    >,
}

impl Timestamper {
    pub fn new(
        mut timestamp_timer: timers::PounderTimestampTimer,
        stream: hal::dma::dma::Stream7<hal::stm32::DMA1>,
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
            .to_input_capture(timers::CaptureTrigger::TriggerInput);
        input_capture.listen_dma();

        // The data transfer is always a transfer of data from the peripheral to a RAM buffer.
        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                stream,
                input_capture,
                // Note(unsafe): The BUF[0] and BUF[1] is "owned" by this peripheral.
                // It shall not be used anywhere else in the module.
                unsafe { &mut BUF[0] },
                unsafe { Some(&mut BUF[1]) },
                config,
            );

        data_transfer.start(|capture_channel| capture_channel.enable());

        Self {
            timer: timestamp_timer,
            transfer: data_transfer,
            next_buffer: unsafe { Some(&mut BUF[2]) },
        }
    }

    pub fn update_period(&mut self, period: u16) {
        self.timer.set_period(period);
    }

    /// Obtain a buffer filled with timestamps.
    ///
    /// # Returns
    /// A reference to the underlying buffer that has been filled with timestamps.
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
