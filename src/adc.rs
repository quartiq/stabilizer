///! Stabilizer ADC management interface
///!
///! The Stabilizer ADCs utilize a DMA channel to trigger sampling. The SPI streams are configured
///! for full-duplex operation, but only RX is connected to physical pins. A timer channel is
///! configured to generate a DMA write into the SPI TXFIFO, which initiates a SPI transfer and
///! results in an ADC sample read for both channels.
///!
///! In order to read multiple samples without interrupting the CPU, a separate DMA transfer is
///! configured to read from each of the ADC SPI RX FIFOs. Due to the design of the SPI peripheral,
///! these DMA transfers stall when no data is available in the FIFO. Thus, the DMA transfer only
///! completes after all samples have been read. When this occurs, a CPU interrupt is generated so
///! that software can process the acquired samples from both ADCs. Only one of the ADC DMA streams
///! is configured to generate an interrupt to handle both transfers, so it is necessary to ensure
///! both transfers are completed before reading the data. This is usually not significant for
///! busy-waiting because the transfers should complete at approximately the same time.
use super::{
    hal, sampling_timer, DMAReq, DmaConfig, MemoryToPeripheral,
    PeripheralToMemory, Priority, TargetAddress, Transfer,
};

// The desired ADC input buffer size. This is use configurable.
const INPUT_BUFFER_SIZE: usize = 1;

// The following data is written by the timer ADC sample trigger into each of the SPI TXFIFOs. Note
// that because the SPI MOSI line is not connected, this data is dont-care. Data in AXI SRAM is not
// initialized on boot, so the contents are random.
#[link_section = ".axisram.buffers"]
static mut SPI_START: [u16; 1] = [0x00];

// The following global buffers are used for the ADC sample DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being acquired while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined.
#[link_section = ".axisram.buffers"]
static mut ADC0_BUF0: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC0_BUF1: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC1_BUF0: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC1_BUF1: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

/// SPI2 is used as a ZST (zero-sized type) for indicating a DMA transfer into the SPI2 TX FIFO
/// whenever the tim2 update dma request occurs.
struct SPI2 {}
impl SPI2 {
    pub fn new() -> Self {
        Self {}
    }
}

unsafe impl TargetAddress<MemoryToPeripheral> for SPI2 {
    /// SPI2 is configured to operate using 16-bit transfer words.
    type MemSize = u16;

    /// SPI2 DMA requests are generated whenever TIM2 CH1 comparison occurs.
    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_CH1 as u8);

    /// Whenever the DMA request occurs, it should write into SPI2's TX FIFO to start a DMA
    /// transfer.
    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::SPI2::ptr() };
        &regs.txdr as *const _ as u32
    }
}

/// SPI3 is used as a ZST (zero-sized type) for indicating a DMA transfer into the SPI3 TX FIFO
/// whenever the tim2 update dma request occurs.
struct SPI3 {}
impl SPI3 {
    pub fn new() -> Self {
        Self {}
    }
}

unsafe impl TargetAddress<MemoryToPeripheral> for SPI3 {
    /// SPI3 is configured to operate using 16-bit transfer words.
    type MemSize = u16;

    /// SPI3 DMA requests are generated whenever TIM2 CH2 comparison occurs.
    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_CH2 as u8);

    /// Whenever the DMA request occurs, it should write into SPI3's TX FIFO to start a DMA
    /// transfer.
    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::SPI3::ptr() };
        &regs.txdr as *const _ as u32
    }
}

/// Represents both ADC input channels.
pub struct AdcInputs {
    adc0: Adc0Input,
    adc1: Adc1Input,
}

impl AdcInputs {
    /// Construct the ADC inputs.
    pub fn new(adc0: Adc0Input, adc1: Adc1Input) -> Self {
        Self { adc0, adc1 }
    }

    /// Interrupt handler to handle when the sample collection DMA transfer completes.
    ///
    /// # Returns
    /// (adc0, adc1) where adcN is a reference to the collected ADC samples. Two array references
    /// are returned - one for each ADC sample stream.
    pub fn transfer_complete_handler(
        &mut self,
    ) -> (&[u16; INPUT_BUFFER_SIZE], &[u16; INPUT_BUFFER_SIZE]) {
        let adc0_buffer = self.adc0.transfer_complete_handler();
        let adc1_buffer = self.adc1.transfer_complete_handler();
        (adc0_buffer, adc1_buffer)
    }
}

/// Represents data associated with ADC0.
pub struct Adc0Input {
    next_buffer: Option<&'static mut [u16; INPUT_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream1<hal::stm32::DMA1>,
        hal::spi::Spi<hal::stm32::SPI2, hal::spi::Disabled, u16>,
        PeripheralToMemory,
        &'static mut [u16; INPUT_BUFFER_SIZE],
    >,
    _trigger_transfer: Transfer<
        hal::dma::dma::Stream0<hal::stm32::DMA1>,
        SPI2,
        MemoryToPeripheral,
        &'static mut [u16; 1],
    >,
}

impl Adc0Input {
    /// Construct the ADC0 input channel.
    ///
    /// # Args
    /// * `spi` - The SPI interface used to communicate with the ADC.
    /// * `trigger_stream` - The DMA stream used to trigger each ADC transfer by writing a word into
    ///   the SPI TX FIFO.
    /// * `data_stream` - The DMA stream used to read samples received over SPI into a data buffer.
    /// * `_trigger_channel` - The ADC sampling timer output compare channel for read triggers.
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI2, hal::spi::Enabled, u16>,
        trigger_stream: hal::dma::dma::Stream0<hal::stm32::DMA1>,
        data_stream: hal::dma::dma::Stream1<hal::stm32::DMA1>,
        trigger_channel: sampling_timer::Timer2Channel1,
    ) -> Self {
        // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
        // occurs.
        trigger_channel.listen_dma();
        trigger_channel.to_output_compare(0);

        // The trigger stream constantly writes to the TX FIFO using a static word (dont-care
        // contents). Thus, neither the memory or peripheral address ever change. This is run in
        // circular mode to be completed at every DMA request.
        let trigger_config = DmaConfig::default()
            .memory_increment(false)
            .peripheral_increment(false)
            .priority(Priority::High)
            .circular_buffer(true);

        // Construct the trigger stream to write from memory to the peripheral.
        let mut trigger_transfer: Transfer<_, _, MemoryToPeripheral, _> =
            Transfer::init(
                trigger_stream,
                SPI2::new(),
                unsafe { &mut SPI_START },
                None,
                trigger_config,
            );

        // The data stream constantly reads from the SPI RX FIFO into a RAM buffer. The peripheral
        // stalls reads of the SPI RX FIFO until data is available, so the DMA transfer completes
        // after the requested number of samples have been collected. Note that only ADC1's data
        // stream is used to trigger a transfer completion interrupt.
        let data_config = DmaConfig::default()
            .memory_increment(true)
            .priority(Priority::VeryHigh)
            .peripheral_increment(false);

        // A SPI peripheral error interrupt is used to determine if the RX FIFO overflows. This
        // indicates that samples were dropped due to excessive processing time in the main
        // application (e.g. a second DMA transfer completes before the first was done with
        // processing). This is used as a flow control indicator to guarantee that no ADC samples
        // are lost.
        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        // The data transfer is always a transfer of data from the peripheral to a RAM buffer.
        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                data_stream,
                spi,
                unsafe { &mut ADC0_BUF0 },
                None,
                data_config,
            );

        data_transfer.start(|spi| {
            // Allow the SPI FIFOs to operate using only DMA data channels.
            spi.enable_dma_rx();
            spi.enable_dma_tx();

            // Enable SPI and start it in infinite transaction mode.
            spi.inner().cr1.modify(|_, w| w.spe().set_bit());
            spi.inner().cr1.modify(|_, w| w.cstart().started());
        });

        trigger_transfer.start(|_| {});

        Self {
            next_buffer: unsafe { Some(&mut ADC0_BUF1) },
            transfer: data_transfer,
            _trigger_transfer: trigger_transfer,
        }
    }

    /// Handle a transfer completion.
    ///
    /// # Returns
    /// A reference to the underlying buffer that has been filled with ADC samples.
    pub fn transfer_complete_handler(&mut self) -> &[u16; INPUT_BUFFER_SIZE] {
        let next_buffer = self.next_buffer.take().unwrap();

        // Wait for the transfer to fully complete before continuing.
        while self.transfer.get_transfer_complete_flag() == false {}

        // Start the next transfer.
        self.transfer.clear_interrupts();
        let (prev_buffer, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer);
        self.next_buffer.as_ref().unwrap()
    }
}

/// Represents the data input stream from ADC1
pub struct Adc1Input {
    next_buffer: Option<&'static mut [u16; INPUT_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream3<hal::stm32::DMA1>,
        hal::spi::Spi<hal::stm32::SPI3, hal::spi::Disabled, u16>,
        PeripheralToMemory,
        &'static mut [u16; INPUT_BUFFER_SIZE],
    >,
    _trigger_transfer: Transfer<
        hal::dma::dma::Stream2<hal::stm32::DMA1>,
        SPI3,
        MemoryToPeripheral,
        &'static mut [u16; 1],
    >,
}

impl Adc1Input {
    /// Construct a new ADC1 input data stream.
    ///
    /// # Args
    /// * `spi` - The SPI interface connected to ADC1.
    /// * `trigger_stream` - The DMA stream used to trigger ADC conversions on the SPI interface.
    /// * `data_stream` - The DMA stream used to read ADC samples from the SPI RX FIFO.
    /// * `trigger_channel` - The ADC sampling timer output compare channel for read triggers.
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI3, hal::spi::Enabled, u16>,
        trigger_stream: hal::dma::dma::Stream2<hal::stm32::DMA1>,
        data_stream: hal::dma::dma::Stream3<hal::stm32::DMA1>,
        trigger_channel: sampling_timer::Timer2Channel2,
    ) -> Self {
        // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
        // occurs.
        trigger_channel.listen_dma();
        trigger_channel.to_output_compare(0);

        // The trigger stream constantly writes to the TX FIFO using a static word (dont-care
        // contents). Thus, neither the memory or peripheral address ever change. This is run in
        // circular mode to be completed at every DMA request.
        let trigger_config = DmaConfig::default()
            .memory_increment(false)
            .peripheral_increment(false)
            .priority(Priority::High)
            .circular_buffer(true);

        // Construct the trigger stream to write from memory to the peripheral.
        let mut trigger_transfer: Transfer<_, _, MemoryToPeripheral, _> =
            Transfer::init(
                trigger_stream,
                SPI3::new(),
                unsafe { &mut SPI_START },
                None,
                trigger_config,
            );

        // The data stream constantly reads from the SPI RX FIFO into a RAM buffer. The peripheral
        // stalls reads of the SPI RX FIFO until data is available, so the DMA transfer completes
        // after the requested number of samples have been collected. Note that only ADC1's data
        // stream is used to trigger a transfer completion interrupt.
        let data_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .priority(Priority::VeryHigh)
            .peripheral_increment(false);

        // A SPI peripheral error interrupt is used to determine if the RX FIFO overflows. This
        // indicates that samples were dropped due to excessive processing time in the main
        // application (e.g. a second DMA transfer completes before the first was done with
        // processing). This is used as a flow control indicator to guarantee that no ADC samples
        // are lost.
        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        // The data transfer is always a transfer of data from the peripheral to a RAM buffer.
        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                data_stream,
                spi,
                unsafe { &mut ADC1_BUF0 },
                None,
                data_config,
            );

        data_transfer.start(|spi| {
            // Allow the SPI FIFOs to operate using only DMA data channels.
            spi.enable_dma_rx();
            spi.enable_dma_tx();

            // Enable SPI and start it in infinite transaction mode.
            spi.inner().cr1.modify(|_, w| w.spe().set_bit());
            spi.inner().cr1.modify(|_, w| w.cstart().started());
        });

        trigger_transfer.start(|_| {});

        Self {
            next_buffer: unsafe { Some(&mut ADC1_BUF1) },
            transfer: data_transfer,
            _trigger_transfer: trigger_transfer,
        }
    }

    /// Handle a transfer completion.
    ///
    /// # Returns
    /// A reference to the underlying buffer that has been filled with ADC samples.
    pub fn transfer_complete_handler(&mut self) -> &[u16; INPUT_BUFFER_SIZE] {
        let next_buffer = self.next_buffer.take().unwrap();

        // Wait for the transfer to fully complete before continuing.
        while self.transfer.get_transfer_complete_flag() == false {}

        // Start the next transfer.
        self.transfer.clear_interrupts();
        let (prev_buffer, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer);
        self.next_buffer.as_ref().unwrap()
    }
}
