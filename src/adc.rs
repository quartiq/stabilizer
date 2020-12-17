///! Stabilizer ADC management interface
///!
///! # Design
///!
///! Stabilizer ADCs are connected to the MCU via a simplex, SPI-compatible interface. The ADCs
///! require a setup conversion time after asserting the CSn (convert) signal to generate the ADC
///! code from the sampled level. Once the setup time has elapsed, the ADC data is clocked out of
///! MISO. The internal setup time is managed by the SPI peripheral via a CSn setup time parameter
///! during SPI configuration, which allows offloading the management of the setup time to hardware.
///!
///! Because of the SPI-compatibility of the ADCs, a single SPI peripheral + DMA is used to automate
///! the collection of multiple ADC samples without requiring processing by the CPU, which reduces
///! overhead and provides the CPU with more time for processing-intensive tasks, like DSP.
///!
///! The automation of sample collection utilizes two DMA streams, the SPI peripheral, and a timer
///! compare channel for each ADC. The timer comparison channel is configured to generate a
///! comparison event every time the timer is equal to a specific value. Each comparison then
///! generates a DMA transfer event to write into the SPI TX buffer. Although the SPI is a simplex,
///! RX-only interface, it is configured in full-duplex mode and the TX pin is left disconnected.
///! This allows the SPI interface to periodically read a single word whenever a word is written to
///! the TX side. Thus, by running a continuous DMA transfer to periodically write a value into the
///! TX FIFO, we can schedule the regular collection of ADC samples in the SPI RX buffer.
///!
///! In order to collect the acquired ADC samples into a RAM buffer, a second DMA transfer is
///! configured to read from the SPI RX FIFO into RAM. The request for this transfer is connected to
///! the SPI RX data signal, so the SPI peripheral will request to move data into RAM whenever it is
///! available. When enough samples have been collected, a transfer-complete interrupt is generated
///! and the ADC samples are available for processing.
///!
///! The SPI peripheral internally has an 8- or 16-byte TX and RX FIFO, which corresponds to a 4- or
///! 8-sample buffer for incoming ADC samples. During the handling of the DMA transfer completion,
///! there is a small window where buffers are swapped over where it's possible that a sample could
///! be lost. In order to avoid this, the SPI RX FIFO is effectively used as a "sample overflow"
///! region and can buffer a number of samples until the next DMA transfer is configured. If a DMA
///! transfer is still not set in time, the SPI peripheral will generate an input-overrun interrupt.
///! This interrupt then serves as a means of detecting if samples have been lost, which will occur
///! whenever data processing takes longer than the collection period.
///!
///!
///! ## Starting Data Collection
///!
///! Because the DMA data collection is automated via timer count comparisons and DMA transfers, the
///! ADCs can be initialized and configured, but will not begin sampling the external ADCs until the
///! sampling timer is enabled. As such, the sampling timer should be enabled after all
///! initialization has completed and immediately before the embedded processing loop begins.
///!
///!
///! ## Batch Sizing
///!
///! The ADCs collect a group of N samples, which is referred to as a batch. The size of the batch
///! is configured by the user at compile-time to allow for a custom-tailored implementation. Larger
///! batch sizes generally provide for more processing time per sample, but come at the expense of
///! increased input -> output latency.
///!
///!
///! # Note
///!
///! While there are two ADCs, only a single ADC is configured to generate transfer-complete
///! interrupts. This is done because it is assumed that the ADCs will always be sampled
///! simultaneously. If only a single ADC is used, it must always be ADC0, as ADC1 will not generate
///! transfer-complete interrupts.
///!
///! There is a very small amount of latency between sampling of ADCs due to bus matrix priority. As
///! such, one of the ADCs will be sampled marginally earlier before the other because the DMA
///! requests are generated simultaneously. This can be avoided by providing a known offset to the
///! sample DMA requests, which can be completed by setting e.g. ADC0's comparison to a counter
///! value of 0 and ADC1's comparison to a counter value of 1.
///!
///! In this implementation, single buffer mode DMA transfers are used because the SPI RX FIFO can
///! be used as a means to both detect and buffer ADC samples during the buffer swap-over. Because
///! of this, double-buffered mode does not offer any advantages over single-buffered mode (unless
///! double-buffered mode offers less overhead when accessing data).
use super::{
    hal, sampling_timer, DMAReq, DmaConfig, MemoryToPeripheral,
    PeripheralToMemory, Priority, TargetAddress, Transfer, SAMPLE_BUFFER_SIZE,
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
            _channel: sampling_timer::tim2::$trigger_channel,
        }
        impl $spi {
            pub fn new(
                _channel: sampling_timer::tim2::$trigger_channel,
            ) -> Self {
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
                trigger_channel: sampling_timer::tim2::$trigger_channel,
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
                let (prev_buffer, _) =
                    self.transfer.next_transfer(next_buffer).unwrap();

                self.next_buffer.replace(prev_buffer); // .unwrap_none() https://github.com/rust-lang/rust/issues/62633

                self.next_buffer.as_ref().unwrap()
            }
        }
    };
}

adc_input!(Adc0Input, 0, Stream0, Stream1, SPI2, Channel1, TIM2_CH1);
adc_input!(Adc1Input, 1, Stream2, Stream3, SPI3, Channel2, TIM2_CH2);
