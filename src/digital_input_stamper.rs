use super::{hal, sampling_timer, DmaConfig, PeripheralToMemory, Transfer};

const INPUT_BUFFER_SIZE: usize = 1;

#[link_section = ".axisram.buffers"]
static mut BUF0: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut BUF1: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

pub struct InputStamper {
    _di0_trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF1>>,
    timestamp_buffer: heapless::Vec<u16, heapless::consts::U128>,
    next_buffer: Option<&'static mut [u16; INPUT_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream4<hal::stm32::DMA1>,
        sampling_timer::Timer2Channel4,
        PeripheralToMemory,
        &'static mut [u16; INPUT_BUFFER_SIZE],
    >,
}

impl InputStamper {
    pub fn new(
        trigger: hal::gpio::gpioa::PA3<hal::gpio::Alternate<hal::gpio::AF1>>,
        stream: hal::dma::dma::Stream4<hal::stm32::DMA1>,
        timer_channel: sampling_timer::Timer2Channel4,
    ) -> Self {
        // Utilize the TIM2 CH4 as an input capture channel - use TI4 (the DI0 input trigger) as the
        // capture source.
        timer_channel.listen_dma();
        timer_channel.to_input_capture(sampling_timer::CC4S_A::TI4);

        // Set up the DMA transfer.
        let dma_config = DmaConfig::default()
            .memory_increment(true)
            .peripheral_increment(false);

        let mut timestamp_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                stream,
                timer_channel,
                unsafe { &mut BUF0 },
                None,
                dma_config,
            );

        timestamp_transfer.start(|_| {});

        Self {
            timestamp_buffer: heapless::Vec::new(),
            next_buffer: unsafe { Some(&mut BUF1) },
            transfer: timestamp_transfer,
            _di0_trigger: trigger,
        }
    }

    pub fn transfer_complete_handler(&mut self) {
        let next_buffer = self.next_buffer.take().unwrap();
        self.transfer.clear_interrupts();
        let (prev_buffer, _, remaining_transfers) =
            self.transfer.next_transfer(next_buffer).unwrap();

        let valid_count = prev_buffer.len() - remaining_transfers;
        self.timestamp_buffer
            .extend_from_slice(&prev_buffer[..valid_count])
            .unwrap();

        self.next_buffer.replace(prev_buffer);
    }

    pub fn with_timestamps<F>(&mut self, f: F)
    where
        F: FnOnce(&[u16]),
    {
        // First, run the transfer complete handler to retrieve any timestamps that are pending in
        // the DMA transfer.
        self.transfer_complete_handler();

        f(self.timestamp_buffer.as_ref());

        self.timestamp_buffer.clear();
    }
}
