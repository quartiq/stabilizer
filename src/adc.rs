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
    hal, timers, DMAReq, DmaConfig, MemoryToPeripheral, PeripheralToMemory,
    Priority, TargetAddress, Transfer, SAMPLE_BUFFER_SIZE,
};

// The following data is written by the timer ADC sample trigger into each of the SPI TXFIFOs. Note
// that because the SPI MOSI line is not connected, this data is dont-care. Data in AXI SRAM is not
// initialized on boot, so the contents are random.
#[link_section = ".axisram.buffers"]
static mut SPI_START: [u16; 1] = [0x00];

// The following global buffers are used for the ADC sample DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being acquired while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined. The dimensions are `ADC_BUF[adc_index][ping_pong_index][sample_index]`.
#[link_section = ".axisram.buffers"]
static mut ADC_BUF: [[[u16; SAMPLE_BUFFER_SIZE]; 2]; 2] =
    [[[0; SAMPLE_BUFFER_SIZE]; 2]; 2];

macro_rules! adc_input {
    ($name:ident, $index:literal, $trigger_stream:ident, $data_stream:ident,
     $spi:ident, $trigger_channel:ident, $dma_req:ident) => {
        /// $spi is used as a type for indicating a DMA transfer into the SPI TX FIFO
        /// whenever the tim2 update dma request occurs.
        struct $spi {
            _channel: timers::tim2::$trigger_channel,
        }
        impl $spi {
            pub fn new(_channel: timers::tim2::$trigger_channel) -> Self {
                Self { _channel }
            }
        }

        // Note(unsafe): This structure is only safe to instantiate once. The DMA request is hard-coded and
        // may only be used if ownership of the timer2 $trigger_channel compare channel is assured, which is
        // ensured by maintaining ownership of the channel.
        unsafe impl TargetAddress<MemoryToPeripheral> for $spi {
            /// SPI is configured to operate using 16-bit transfer words.
            type MemSize = u16;

            /// SPI DMA requests are generated whenever TIM2 CHx ($dma_req) comparison occurs.
            const REQUEST_LINE: Option<u8> = Some(DMAReq::$dma_req as u8);

            /// Whenever the DMA request occurs, it should write into SPI's TX FIFO to start a DMA
            /// transfer.
            fn address(&self) -> usize {
                // Note(unsafe): It is assumed that SPI is owned by another DMA transfer and this DMA is
                // only used for the transmit-half of DMA.
                let regs = unsafe { &*hal::stm32::$spi::ptr() };
                &regs.txdr as *const _ as usize
            }
        }

        /// Represents data associated with ADC.
        pub struct $name {
            next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
            transfer: Transfer<
                hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                hal::spi::Spi<hal::stm32::$spi, hal::spi::Disabled, u16>,
                PeripheralToMemory,
                &'static mut [u16; SAMPLE_BUFFER_SIZE],
            >,
            _trigger_transfer: Transfer<
                hal::dma::dma::$trigger_stream<hal::stm32::DMA1>,
                $spi,
                MemoryToPeripheral,
                &'static mut [u16; 1],
            >,
        }

        impl $name {
            /// Construct the ADC input channel.
            ///
            /// # Args
            /// * `spi` - The SPI interface used to communicate with the ADC.
            /// * `trigger_stream` - The DMA stream used to trigger each ADC transfer by writing a word into
            ///   the SPI TX FIFO.
            /// * `data_stream` - The DMA stream used to read samples received over SPI into a data buffer.
            /// * `_trigger_channel` - The ADC sampling timer output compare channel for read triggers.
            pub fn new(
                spi: hal::spi::Spi<hal::stm32::$spi, hal::spi::Enabled, u16>,
                trigger_stream: hal::dma::dma::$trigger_stream<
                    hal::stm32::DMA1,
                >,
                data_stream: hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                trigger_channel: timers::tim2::$trigger_channel,
            ) -> Self {
                // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
                // occurs.
                trigger_channel.listen_dma();
                trigger_channel.to_output_compare(0);

                // The trigger stream constantly writes to the TX FIFO using a static word (dont-care
                // contents). Thus, neither the memory or peripheral address ever change. This is run in
                // circular mode to be completed at every DMA request.
                let trigger_config = DmaConfig::default()
                    .priority(Priority::High)
                    .circular_buffer(true);

                // Construct the trigger stream to write from memory to the peripheral.
                let mut trigger_transfer: Transfer<
                    _,
                    _,
                    MemoryToPeripheral,
                    _,
                > = Transfer::init(
                    trigger_stream,
                    $spi::new(trigger_channel),
                    // Note(unsafe): Because this is a Memory->Peripheral transfer, this data is never
                    // actually modified. It technically only needs to be immutably borrowed, but the
                    // current HAL API only supports mutable borrows.
                    unsafe { &mut SPI_START },
                    None,
                    trigger_config,
                );

                // The data stream constantly reads from the SPI RX FIFO into a RAM buffer. The peripheral
                // stalls reads of the SPI RX FIFO until data is available, so the DMA transfer completes
                // after the requested number of samples have been collected. Note that only ADC1's (sic!)
                // data stream is used to trigger a transfer completion interrupt.
                let data_config = DmaConfig::default()
                    .memory_increment(true)
                    .transfer_complete_interrupt($index == 1)
                    .priority(Priority::VeryHigh);

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
                        // Note(unsafe): The ADC_BUF[$index][0] is "owned" by this peripheral.
                        // It shall not be used anywhere else in the module.
                        unsafe { &mut ADC_BUF[$index][0] },
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
                    // Note(unsafe): The ADC_BUF[$index][1] is "owned" by this peripheral. It shall not be used
                    // anywhere else in the module.
                    next_buffer: unsafe { Some(&mut ADC_BUF[$index][1]) },
                    transfer: data_transfer,
                    _trigger_transfer: trigger_transfer,
                }
            }

            /// Obtain a buffer filled with ADC samples.
            ///
            /// # Returns
            /// A reference to the underlying buffer that has been filled with ADC samples.
            pub fn acquire_buffer(&mut self) -> &[u16; SAMPLE_BUFFER_SIZE] {
                // Wait for the transfer to fully complete before continuing.
                // Note: If a device hangs up, check that this conditional is passing correctly, as there is
                // no time-out checks here in the interest of execution speed.
                while !self.transfer.get_transfer_complete_flag() {}

                let next_buffer = self.next_buffer.take().unwrap();

                // Start the next transfer.
                self.transfer.clear_interrupts();
                let (prev_buffer, _, _) =
                    self.transfer.next_transfer(next_buffer).unwrap();

                self.next_buffer.replace(prev_buffer); // .unwrap_none() https://github.com/rust-lang/rust/issues/62633

                self.next_buffer.as_ref().unwrap()
            }
        }
    };
}

adc_input!(Adc0Input, 0, Stream0, Stream1, SPI2, Channel1, TIM2_CH1);
adc_input!(Adc1Input, 1, Stream2, Stream3, SPI3, Channel2, TIM2_CH2);
