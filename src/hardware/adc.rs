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
///! The automation of sample collection utilizes three DMA streams, the SPI peripheral, and two
///! timer compare channel for each ADC. One timer comparison channel is configured to generate a
///! comparison event every time the timer is equal to a specific value. Each comparison then
///! generates a DMA transfer event to write into the SPI CR1 register to initiate the transfer.
///! This allows the SPI interface to periodically read a single sample. The other timer comparison
///! channel is configured to generate a comparison event slightly before the first (~10 timer
///! cycles). This channel triggers a separate DMA stream to clear the EOT flag within the SPI
///! peripheral. The EOT flag must be cleared after each transfer or the SPI peripheral will not
///! properly complete the single conversion. Thus, by using two DMA streams and timer comparison
///! channels, the SPI can regularly acquire ADC samples.
///!
///! In order to collect the acquired ADC samples into a RAM buffer, a final DMA transfer is
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
///! batch sizes generally provide for lower overhead and more processing time per sample, but come
///! at the expense of increased input -> output latency.
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
///! double-buffered mode offers less overhead due to the DMA disable/enable procedure).
use stm32h7xx_hal as hal;

use super::design_parameters::SAMPLE_BUFFER_SIZE;
use super::timers;

use hal::dma::{
    config::Priority,
    dma::{DMAReq, DmaConfig},
    traits::TargetAddress,
    MemoryToPeripheral, PeripheralToMemory, Transfer,
};

/// A type representing an ADC sample.
#[derive(Copy, Clone)]
pub struct AdcCode(pub u16);

impl Into<f32> for AdcCode {
    /// Convert raw ADC codes to/from voltage levels.
    ///
    /// # Note
    /// This does not account for the programmable gain amplifier at the signal input.
    fn into(self) -> f32 {
        // The ADC has a differential input with a range of +/- 4.096 V and 16-bit resolution.
        // The gain into the two inputs is 1/5.
        let adc_volts_per_lsb = 5.0 / 2.0 * 4.096 / (1u16 << 15) as f32;

        (self.0 as i16) as f32 * adc_volts_per_lsb
    }
}

// The following data is written by the timer ADC sample trigger into the SPI CR1 to start the
// transfer. Data in AXI SRAM is not initialized on boot, so the contents are random. This value is
// initialized during setup.
#[link_section = ".axisram.buffers"]
static mut SPI_START: [u32; 1] = [0x00; 1];

// The following data is written by the timer flag clear trigger into the SPI IFCR register to clear
// the EOT flag. Data in AXI SRAM is not initialized on boot, so the contents are random. This
// value is initialized during setup.
#[link_section = ".axisram.buffers"]
static mut SPI_EOT_CLEAR: [u32; 1] = [0x00];

// The following global buffers are used for the ADC sample DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being acquired while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined. The dimensions are `ADC_BUF[adc_index][ping_pong_index][sample_index]`.
#[link_section = ".axisram.buffers"]
static mut ADC_BUF: [[[u16; SAMPLE_BUFFER_SIZE]; 2]; 2] =
    [[[0; SAMPLE_BUFFER_SIZE]; 2]; 2];

macro_rules! adc_input {
    ($name:ident, $index:literal, $trigger_stream:ident, $data_stream:ident, $clear_stream:ident,
     $spi:ident, $trigger_channel:ident, $dma_req:ident, $clear_channel:ident, $dma_clear_req:ident) => {
        paste::paste! {
            /// $spi-CR is used as a type for indicating a DMA transfer into the SPI control
            /// register whenever the tim2 update dma request occurs.
            struct [< $spi CR >] {
                _channel: timers::tim2::$trigger_channel,
            }
            impl [< $spi CR >] {
                pub fn new(_channel: timers::tim2::$trigger_channel) -> Self {
                    Self { _channel }
                }
            }

            // Note(unsafe): This structure is only safe to instantiate once. The DMA request is
            // hard-coded and may only be used if ownership of the timer2 $trigger_channel compare
            // channel is assured, which is ensured by maintaining ownership of the channel.
            unsafe impl TargetAddress<MemoryToPeripheral> for [< $spi CR >] {

                type MemSize = u32;

                /// SPI DMA requests are generated whenever TIM2 CHx ($dma_req) comparison occurs.
                const REQUEST_LINE: Option<u8> = Some(DMAReq::$dma_req as u8);

                /// Whenever the DMA request occurs, it should write into SPI's CR1 to start the
                /// transfer.
                fn address(&self) -> usize {
                    // Note(unsafe): It is assumed that SPI is owned by another DMA transfer. This
                    // is only safe because we are writing to a configuration register.
                    let regs = unsafe { &*hal::stm32::$spi::ptr() };
                    &regs.cr1 as *const _ as usize
                }
            }

            /// $spi-IFCR is used as a type for indicating a DMA transfer into the SPI flag clear
            /// register whenever the tim3 compare dma request occurs. The flag must be cleared
            /// before the transfer starts.
            struct [< $spi IFCR >] {
                _channel: timers::tim3::$clear_channel,
            }

            impl [< $spi IFCR >] {
                pub fn new(_channel: timers::tim3::$clear_channel) -> Self {
                    Self { _channel }
                }
            }

            // Note(unsafe): This structure is only safe to instantiate once. The DMA request is
            // hard-coded and may only be used if ownership of the timer3 $clear_channel compare
            // channel is assured, which is ensured by maintaining ownership of the channel.
            unsafe impl TargetAddress<MemoryToPeripheral> for [< $spi IFCR >] {
                type MemSize = u32;

                /// SPI DMA requests are generated whenever TIM3 CHx ($dma_clear_req) comparison
                /// occurs.
                const REQUEST_LINE: Option<u8> = Some(DMAReq::$dma_clear_req as u8);

                /// Whenever the DMA request occurs, it should write into SPI's IFCR to clear the
                /// EOT flag to allow the next transmission.
                fn address(&self) -> usize {
                    // Note(unsafe): It is assumed that SPI is owned by another DMA transfer and
                    // this DMA is only used for writing to the configuration registers.
                    let regs = unsafe { &*hal::stm32::$spi::ptr() };
                    &regs.ifcr as *const _ as usize
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
                    hal::dma::DBTransfer,
                >,
                trigger_transfer: Transfer<
                    hal::dma::dma::$trigger_stream<hal::stm32::DMA1>,
                    [< $spi CR >],
                    MemoryToPeripheral,
                    &'static mut [u32; 1],
                    hal::dma::DBTransfer,
                >,
                clear_transfer: Transfer<
                    hal::dma::dma::$clear_stream<hal::stm32::DMA1>,
                    [< $spi IFCR >],
                    MemoryToPeripheral,
                    &'static mut [u32; 1],
                    hal::dma::DBTransfer,
                >,
            }

            impl $name {
                /// Construct the ADC input channel.
                ///
                /// # Args
                /// * `spi` - The SPI interface used to communicate with the ADC.
                /// * `trigger_stream` - The DMA stream used to trigger each ADC transfer by
                ///    writing a word into the SPI TX FIFO.
                /// * `data_stream` - The DMA stream used to read samples received over SPI into a data buffer.
                /// * `clear_stream` - The DMA stream used to clear the EOT flag in the SPI peripheral.
                /// * `trigger_channel` - The ADC sampling timer output compare channel for read triggers.
                /// * `clear_channel` - The shadow sampling timer output compare channel used for
                ///   clearing the SPI EOT flag.
                pub fn new(
                    spi: hal::spi::Spi<hal::stm32::$spi, hal::spi::Enabled, u16>,
                    trigger_stream: hal::dma::dma::$trigger_stream<
                        hal::stm32::DMA1,
                    >,
                    data_stream: hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                    clear_stream: hal::dma::dma::$clear_stream<hal::stm32::DMA1>,
                    trigger_channel: timers::tim2::$trigger_channel,
                    clear_channel: timers::tim3::$clear_channel,
                ) -> Self {
                    // The flag clear DMA transfer always clears the EOT flag in the SPI
                    // peripheral. It has the highest priority to ensure it is completed before the
                    // transfer trigger.
                    let clear_config = DmaConfig::default()
                        .priority(Priority::VeryHigh)
                        .circular_buffer(true);

                    unsafe {
                        SPI_EOT_CLEAR[0] = 1 << 3;
                    }

                    // Generate DMA events when the timer hits zero (roll-over). This must be before
                    // the trigger channel DMA occurs, as if the trigger occurs first, the
                    // transmission will not occur.
                    clear_channel.listen_dma();
                    clear_channel.to_output_compare(0);

                    let clear_transfer: Transfer<
                        _,
                        _,
                        MemoryToPeripheral,
                        _,
                        _,
                    > = Transfer::init(
                        clear_stream,
                        [< $spi IFCR >]::new(clear_channel),
                        // Note(unsafe): Because this is a Memory->Peripheral transfer, this data is
                        // never actually modified. It technically only needs to be immutably
                        // borrowed, but the current HAL API only supports mutable borrows.
                        unsafe { &mut SPI_EOT_CLEAR },
                        None,
                        clear_config,
                    );

                    // Generate DMA events when an output compare of the timer hits the specified
                    // value.
                    trigger_channel.listen_dma();
                    trigger_channel.to_output_compare(2 + $index);

                    // The trigger stream constantly writes to the SPI CR1 using a static word
                    // (which is a static value to enable the SPI transfer).  Thus, neither the
                    // memory or peripheral address ever change. This is run in circular mode to be
                    // completed at every DMA request.
                    let trigger_config = DmaConfig::default()
                        .priority(Priority::High)
                        .circular_buffer(true);

                    // Note(unsafe): This word is initialized once per ADC initialization to verify
                    // it is initialized properly.
                    unsafe {
                        // Write a binary code into the SPI control register to initiate a transfer.
                        SPI_START[0] = 0x201;
                    };

                    // Construct the trigger stream to write from memory to the peripheral.
                    let trigger_transfer: Transfer<
                        _,
                        _,
                        MemoryToPeripheral,
                        _,
                        _,
                    > = Transfer::init(
                        trigger_stream,
                        [< $spi CR >]::new(trigger_channel),
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

                    // A SPI peripheral error interrupt is used to determine if the RX FIFO
                    // overflows. This indicates that samples were dropped due to excessive
                    // processing time in the main application (e.g. a second DMA transfer completes
                    // before the first was done with processing). This is used as a flow control
                    // indicator to guarantee that no ADC samples are lost.
                    let mut spi = spi.disable();
                    spi.listen(hal::spi::Event::Error);

                    // The data transfer is always a transfer of data from the peripheral to a RAM
                    // buffer.
                    let data_transfer: Transfer<_, _, PeripheralToMemory, _, _> =
                        Transfer::init(
                            data_stream,
                            spi,
                            // Note(unsafe): The ADC_BUF[$index][0] is "owned" by this peripheral.
                            // It shall not be used anywhere else in the module.
                            unsafe { &mut ADC_BUF[$index][0] },
                            None,
                            data_config,
                        );

                    Self {
                        // Note(unsafe): The ADC_BUF[$index][1] is "owned" by this peripheral. It
                        // shall not be used anywhere else in the module.
                        next_buffer: unsafe { Some(&mut ADC_BUF[$index][1]) },
                        transfer: data_transfer,
                        trigger_transfer,
                        clear_transfer,
                    }
                }

                /// Enable the ADC DMA transfer sequence.
                pub fn start(&mut self) {
                    self.transfer.start(|spi| {
                        spi.enable_dma_rx();

                        spi.inner().cr2.modify(|_, w| w.tsize().bits(1));
                        spi.inner().cr1.modify(|_, w| w.spe().set_bit());
                    });

                    self.clear_transfer.start(|_| {});
                    self.trigger_transfer.start(|_| {});

                }

                /// Obtain a buffer filled with ADC samples.
                ///
                /// # Returns
                /// A reference to the underlying buffer that has been filled with ADC samples.
                pub fn acquire_buffer(&mut self) -> &[u16; SAMPLE_BUFFER_SIZE] {
                    // Wait for the transfer to fully complete before continuing.  Note: If a device
                    // hangs up, check that this conditional is passing correctly, as there is no
                    // time-out checks here in the interest of execution speed.
                    while !self.transfer.get_transfer_complete_flag() {}

                    let next_buffer = self.next_buffer.take().unwrap();

                    // Start the next transfer.
                    self.transfer.clear_interrupts();
                    let (prev_buffer, _, _) =
                        self.transfer.next_transfer(next_buffer).unwrap();

                     // .unwrap_none() https://github.com/rust-lang/rust/issues/62633
                    self.next_buffer.replace(prev_buffer);

                    self.next_buffer.as_ref().unwrap()
                }
            }
        }
    };
}

adc_input!(
    Adc0Input, 0, Stream0, Stream1, Stream2, SPI2, Channel1, TIM2_CH1,
    Channel1, TIM3_CH1
);
adc_input!(
    Adc1Input, 1, Stream3, Stream4, Stream5, SPI3, Channel2, TIM2_CH2,
    Channel2, TIM3_CH2
);
