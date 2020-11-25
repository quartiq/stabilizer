///! Stabilizer DAC management interface
///!
///! The Stabilizer DAC utilize a DMA channel to generate output updates.  A timer channel is
///! configured to generate a DMA write into the SPI TXFIFO, which initiates a SPI transfer and
///! results in DAC update for both channels.
use super::{
    hal, sampling_timer, DMAReq, DmaConfig, MemoryToPeripheral, TargetAddress,
    Transfer, SAMPLE_BUFFER_SIZE,
};

// The following global buffers are used for the DAC code DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being prepared while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined.
#[link_section = ".axisram.buffers"]
static mut DAC0_BUF0: [u16; SAMPLE_BUFFER_SIZE] = [0; SAMPLE_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut DAC0_BUF1: [u16; SAMPLE_BUFFER_SIZE] = [0; SAMPLE_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut DAC1_BUF0: [u16; SAMPLE_BUFFER_SIZE] = [0; SAMPLE_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut DAC1_BUF1: [u16; SAMPLE_BUFFER_SIZE] = [0; SAMPLE_BUFFER_SIZE];

/// SPI4 is used as a type for indicating a DMA transfer into the SPI4 TX FIFO
struct SPI4 {
    spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Disabled, u16>,
    _channel: sampling_timer::tim2::Channel3,
}

impl SPI4 {
    pub fn new(
        _channel: sampling_timer::tim2::Channel3,
        spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Disabled, u16>,
    ) -> Self {
        Self { _channel, spi }
    }
}

// Note(unsafe): This is safe because the DMA request line is logically owned by this module.
// Additionally, the SPI is owned by this structure and is known to be configured for u16 word
// sizes.
unsafe impl TargetAddress<MemoryToPeripheral> for SPI4 {
    /// SPI2 is configured to operate using 16-bit transfer words.
    type MemSize = u16;

    /// SPI4 DMA requests are generated whenever TIM2 CH3 comparison occurs.
    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_CH3 as u8);

    /// Whenever the DMA request occurs, it should write into SPI4's TX FIFO.
    fn address(&self) -> u32 {
        &self.spi.inner().txdr as *const _ as u32
    }
}

/// SPI5 is used as a ZST (zero-sized type) for indicating a DMA transfer into the SPI5 TX FIFO
struct SPI5 {
    _channel: sampling_timer::tim2::Channel4,
    spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Disabled, u16>,
}

impl SPI5 {
    pub fn new(
        _channel: sampling_timer::tim2::Channel4,
        spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Disabled, u16>,
    ) -> Self {
        Self { _channel, spi }
    }
}

// Note(unsafe): This is safe because the DMA request line is logically owned by this module.
// Additionally, the SPI is owned by this structure and is known to be configured for u16 word
// sizes.
unsafe impl TargetAddress<MemoryToPeripheral> for SPI5 {
    /// SPI5 is configured to operate using 16-bit transfer words.
    type MemSize = u16;

    /// SPI5 DMA requests are generated whenever TIM2 CH4 comparison occurs.
    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_CH4 as u8);

    /// Whenever the DMA request occurs, it should write into SPI5's TX FIFO
    fn address(&self) -> u32 {
        &self.spi.inner().txdr as *const _ as u32
    }
}

/// Represents both DAC output channels.
pub struct DacOutputs {
    dac0: Dac0Output,
    dac1: Dac1Output,
}

impl DacOutputs {
    /// Construct the DAC outputs.
    pub fn new(dac0: Dac0Output, dac1: Dac1Output) -> Self {
        Self { dac0, dac1 }
    }

    /// Borrow the next DAC output buffers to populate the DAC output codes in-place.
    ///
    /// # Returns
    /// (dac0, dac1) where each value is a mutable reference to the output code array for DAC0 and
    /// DAC1 respectively.
    pub fn prepare_data(
        &mut self,
    ) -> (
        &mut [u16; SAMPLE_BUFFER_SIZE],
        &mut [u16; SAMPLE_BUFFER_SIZE],
    ) {
        (self.dac0.prepare_buffer(), self.dac1.prepare_buffer())
    }

    /// Enqueue the next DAC output codes for transmission.
    ///
    /// # Note
    /// It is assumed that data was populated using `prepare_data()` before this function is
    /// called.
    pub fn commit_data(&mut self) {
        self.dac0.commit_buffer();
        self.dac1.commit_buffer();
    }
}

/// Represents data associated with DAC0.
pub struct Dac0Output {
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    // Note: SPI TX functionality may not be used from this structure to ensure safety with DMA.
    transfer: Transfer<
        hal::dma::dma::Stream4<hal::stm32::DMA1>,
        SPI4,
        MemoryToPeripheral,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
    >,
    first_transfer: bool,
}

impl Dac0Output {
    /// Construct the DAC0 output channel.
    ///
    /// # Args
    /// * `spi` - The SPI interface used to communicate with the ADC.
    /// * `stream` - The DMA stream used to write DAC codes over SPI.
    /// * `trigger_channel` - The sampling timer output compare channel for update triggers.
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
        stream: hal::dma::dma::Stream4<hal::stm32::DMA1>,
        trigger_channel: sampling_timer::tim2::Channel3,
    ) -> Self {
        // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
        // occurs.
        trigger_channel.listen_dma();
        trigger_channel.to_output_compare(0);

        // The stream constantly writes to the TX FIFO to write new update codes.
        let trigger_config = DmaConfig::default()
            .memory_increment(true)
            .peripheral_increment(false);

        // Listen for any potential SPI error signals, which may indicate that we are not generating
        // update codes.
        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        // Allow the SPI FIFOs to operate using only DMA data channels.
        spi.enable_dma_tx();

        // Enable SPI and start it in infinite transaction mode.
        spi.inner().cr1.modify(|_, w| w.spe().set_bit());
        spi.inner().cr1.modify(|_, w| w.cstart().started());

        // Construct the trigger stream to write from memory to the peripheral.
        let transfer: Transfer<_, _, MemoryToPeripheral, _> = Transfer::init(
            stream,
            SPI4::new(trigger_channel, spi),
            // Note(unsafe): This buffer is only used once and provided for the DMA transfer.
            unsafe { &mut DAC0_BUF0 },
            None,
            trigger_config,
        );

        Self {
            transfer,
            // Note(unsafe): This buffer is only used once and provided for the next DMA transfer.
            next_buffer: unsafe { Some(&mut DAC0_BUF1) },
            first_transfer: true,
        }
    }

    /// Mutably borrow the next output buffer to populate it with DAC codes.
    pub fn prepare_buffer(&mut self) -> &mut [u16; SAMPLE_BUFFER_SIZE] {
        self.next_buffer.as_mut().unwrap()
    }

    /// Enqueue the next buffer for transmission to the DAC.
    ///
    /// # Args
    /// * `data` - The next data to write to the DAC.
    pub fn commit_buffer(&mut self) {
        let next_buffer = self.next_buffer.take().unwrap();

        // If the last transfer was not complete, we didn't write all our previous DAC codes.
        // Wait for all the DAC codes to get written as well.
        if self.first_transfer {
            self.first_transfer = false
        } else {
            // Note: If a device hangs up, check that this conditional is passing correctly, as
            // there is no time-out checks here in the interest of execution speed.
            while self.transfer.get_transfer_complete_flag() == false {}
        }

        // Start the next transfer.
        self.transfer.clear_interrupts();
        let (prev_buffer, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer);
    }
}

/// Represents the data output stream from DAC1.
pub struct Dac1Output {
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream5<hal::stm32::DMA1>,
        SPI5,
        MemoryToPeripheral,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
    >,
    first_transfer: bool,
}

impl Dac1Output {
    /// Construct a new DAC1 output data stream.
    ///
    /// # Args
    /// * `spi` - The SPI interface connected to DAC1.
    /// * `stream` - The DMA stream used to write DAC codes the SPI TX FIFO.
    /// * `trigger_channel` - The timer channel used to generate DMA requests for DAC updates.
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
        stream: hal::dma::dma::Stream5<hal::stm32::DMA1>,
        trigger_channel: sampling_timer::tim2::Channel4,
    ) -> Self {
        // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
        // occurs.
        trigger_channel.listen_dma();
        trigger_channel.to_output_compare(0);

        // The trigger stream constantly writes to the TX FIFO to generate DAC updates.
        let trigger_config = DmaConfig::default()
            .memory_increment(true)
            .peripheral_increment(false)
            .circular_buffer(true);

        // Listen for any SPI errors, as this may indicate that we are not generating updates on the
        // DAC.
        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        // Allow the SPI FIFOs to operate using only DMA data channels.
        spi.enable_dma_tx();

        // Enable SPI and start it in infinite transaction mode.
        spi.inner().cr1.modify(|_, w| w.spe().set_bit());
        spi.inner().cr1.modify(|_, w| w.cstart().started());

        // Construct the stream to write from memory to the peripheral.
        let transfer: Transfer<_, _, MemoryToPeripheral, _> = Transfer::init(
            stream,
            SPI5::new(trigger_channel, spi),
            // Note(unsafe): This buffer is only used once and provided to the transfer.
            unsafe { &mut DAC1_BUF0 },
            None,
            trigger_config,
        );

        Self {
            // Note(unsafe): This buffer is only used once and provided for the next DMA transfer.
            next_buffer: unsafe { Some(&mut DAC1_BUF1) },
            transfer,
            first_transfer: true,
        }
    }

    /// Mutably borrow the next output buffer to populate it with DAC codes.
    pub fn prepare_buffer(&mut self) -> &mut [u16; SAMPLE_BUFFER_SIZE] {
        self.next_buffer.as_mut().unwrap()
    }

    /// Enqueue the next buffer for transmission to the DAC.
    ///
    /// # Args
    /// * `data` - The next data to write to the DAC.
    pub fn commit_buffer(&mut self) {
        let next_buffer = self.next_buffer.take().unwrap();

        // If the last transfer was not complete, we didn't write all our previous DAC codes.
        // Wait for all the DAC codes to get written as well.
        if self.first_transfer {
            self.first_transfer = false
        } else {
            // Note: If a device hangs up, check that this conditional is passing correctly, as
            // there is no time-out checks here in the interest of execution speed.
            while self.transfer.get_transfer_complete_flag() == false {}
        }

        // Start the next transfer.
        self.transfer.clear_interrupts();
        let (prev_buffer, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer);
    }
}
