use super::{SAMPLE_BUFFER_SIZE, hal, timers, DmaConfig, PeripheralToMemory, Transfer};

#[link_section = ".axisram.buffers"]
static mut BUF: [[u16; SAMPLE_BUFFER_SIZE]; 3] = [[0; SAMPLE_BUFFER_SIZE]; 3];

pub struct InputStamper {
    _di0_trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream6<hal::stm32::DMA1>,
        timers::tim5::Channel4InputCapture,
        PeripheralToMemory,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
    >,
}

impl InputStamper {
    pub fn new(
        trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF2>>,
        stream: hal::dma::dma::Stream6<hal::stm32::DMA1>,
        timer_channel: timers::tim5::Channel4,
    ) -> Self {
        // Utilize the TIM5 CH4 as an input capture channel - use TI4 (the DI0 input trigger) as the
        // capture source.
        timer_channel.listen_dma();
        let input_capture =
            timer_channel.to_input_capture(timers::tim5::CC4S_A::TI4);

        // Set up the DMA transfer.
        let dma_config = DmaConfig::default()
            .transfer_complete_interrupt(true)
            .memory_increment(true)
            .circular_buffer(true)
            .double_buffer(true)
            .peripheral_increment(false);

        // This needs to operate in double-buffer+circular mode so that we don't potentially drop
        // input timestamps.
        let mut timestamp_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                stream,
                input_capture,
                unsafe { &mut BUF[0] },
                unsafe { Some(&mut BUF[1]) },
                dma_config,
            );

        timestamp_transfer.start(|_| {});

        Self {
            next_buffer: unsafe { Some(&mut BUF[2]) },
            transfer: timestamp_transfer,
            _di0_trigger: trigger,
        }
    }

    pub fn acquire_buffer(&mut self) -> &[u16] {
        let next_buffer = self.next_buffer.take().unwrap();
        let (prev_buffer, _, remaining_transfers) =
            self.transfer.next_transfer(next_buffer).unwrap();

        let valid_count = prev_buffer.len() - remaining_transfers;

        self.next_buffer.replace(prev_buffer);

        &self.next_buffer.as_ref().unwrap()[..valid_count]
    }
}
