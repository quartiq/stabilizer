//! Stabilizer ADC management interface
//!
//! # Design
//!
//! Stabilizer ADCs are connected to the MCU via a simplex, SPI-compatible interface. The ADCs
//! require a setup conversion time after asserting the CSn (convert) signal to generate the ADC
//! code from the sampled level. Once the setup time has elapsed, the ADC data is clocked out of
//! MISO. The internal setup time is managed by the SPI peripheral via a CSn setup time parameter
//! during SPI configuration, which allows offloading the management of the setup time to hardware.
//!
//! Because of the SPI-compatibility of the ADCs, a single SPI peripheral + DMA is used to automate
//! the collection of multiple ADC samples without requiring processing by the CPU, which reduces
//! overhead and provides the CPU with more time for processing-intensive tasks, like DSP.
//!
//! The automation of sample collection utilizes three DMA streams, the SPI peripheral, and two
//! timer compare channel for each ADC. One timer comparison channel is configured to generate a
//! comparison event every time the timer is equal to a specific value. Each comparison then
//! generates a DMA transfer event to write into the SPI CR1 register to initiate the transfer.
//! This allows the SPI interface to periodically read a single sample. The other timer comparison
//! channel is configured to generate a comparison event slightly before the first (~10 timer
//! cycles). This channel triggers a separate DMA stream to clear the EOT flag within the SPI
//! peripheral. The EOT flag must be cleared after each transfer or the SPI peripheral will not
//! properly complete the single conversion. Thus, by using two DMA streams and timer comparison
//! channels, the SPI can regularly acquire ADC samples.
//!
//! In order to collect the acquired ADC samples into a RAM buffer, a final DMA transfer is
//! configured to read from the SPI RX FIFO into RAM. The request for this transfer is connected to
//! the SPI RX data signal, so the SPI peripheral will request to move data into RAM whenever it is
//! available. When enough samples have been collected, a transfer-complete interrupt is generated
//! and the ADC samples are available for processing.
//!
//! After a complete transfer of a batch of samples, the inactive buffer is available to the
//! user for processing. The processing must complete before the DMA transfer of the next batch
//! completes.
//!
//! ## Starting Data Collection
//!
//! Because the DMA data collection is automated via timer count comparisons and DMA transfers, the
//! ADCs can be initialized and configured, but will not begin sampling the external ADCs until the
//! sampling timer is enabled. As such, the sampling timer should be enabled after all
//! initialization has completed and immediately before the embedded processing loop begins.
//!
//!
//! ## Batch Sizing
//!
//! The ADCs collect a group of N samples, which is referred to as a batch. The size of the batch
//! is configured by the user at compile-time to allow for a custom-tailored implementation. Larger
//! batch sizes generally provide for lower overhead and more processing time per sample, but come
//! at the expense of increased input -> output latency.
//!
//!
//! # Note
//!
//! While there are two ADCs, only a single ADC is configured to generate transfer-complete
//! interrupts. This is done because it is assumed that the ADCs will always be sampled
//! simultaneously. If only a single ADC is used, it must always be ADC0, as ADC1 will not generate
//! transfer-complete interrupts.
//!
//! There is a very small amount of latency between sampling of ADCs due to bus matrix priority. As
//! such, one of the ADCs will be sampled marginally earlier before the other because the DMA
//! requests are generated simultaneously. This can be avoided by providing a known offset to the
//! sample DMA requests, which can be completed by setting e.g. ADC0's comparison to a counter
//! value of 0 and ADC1's comparison to a counter value of 1.
//!
//! In this implementation, double buffer mode DMA transfers are used because the SPI RX FIFOs
//! have finite depth, FIFO access is slower than AXISRAM access, and because the single
//! buffer mode DMA disable/enable and buffer update sequence is slow.
use core::mem::MaybeUninit;

use stm32h7xx_hal as hal;

use mutex_trait::Mutex;

use super::design_parameters::SampleBuffer;
use super::timers;

use hal::{
    dma::{
        config::Priority,
        dma::{DMAReq, DmaConfig},
        traits::TargetAddress,
        DMAError, MemoryToPeripheral, PeripheralToMemory, Transfer,
    },
    spi::{HalDisabledSpi, HalEnabledSpi, HalSpi},
};

/// A type representing an ADC sample.
#[derive(Copy, Clone)]
pub struct AdcCode(pub u16);

impl AdcCode {
    // The ADC has a differential input with a range of +/- 4.096 V and 16-bit resolution.
    // The gain into the two inputs is 1/5.
    const FULL_SCALE: f32 = 5.0 / 2.0 * 4.096;
    const VOLT_PER_LSB: f32 = -Self::FULL_SCALE / i16::MIN as f32;
    const LSB_PER_VOLT: f32 = 1. / Self::VOLT_PER_LSB;
}

impl From<u16> for AdcCode {
    /// Construct an ADC code from a provided binary (ADC-formatted) code.
    fn from(value: u16) -> Self {
        Self(value)
    }
}

impl From<i16> for AdcCode {
    /// Construct an ADC code from the stabilizer-defined code (i16 full range).
    fn from(value: i16) -> Self {
        Self(value as u16)
    }
}

impl From<AdcCode> for i16 {
    /// Get a stabilizer-defined code from the ADC code.
    fn from(code: AdcCode) -> i16 {
        code.0 as i16
    }
}

impl From<AdcCode> for u16 {
    /// Get an ADC-frmatted binary value from the code.
    fn from(code: AdcCode) -> u16 {
        code.0
    }
}

impl From<AdcCode> for f32 {
    /// Convert raw ADC codes to/from voltage levels.
    ///
    /// # Note
    /// This does not account for the programmable gain amplifier at the signal input.
    fn from(code: AdcCode) -> f32 {
        i16::from(code) as f32 * AdcCode::VOLT_PER_LSB
    }
}

impl TryFrom<f32> for AdcCode {
    type Error = ();

    fn try_from(voltage: f32) -> Result<AdcCode, ()> {
        let code = voltage * Self::LSB_PER_VOLT;
        if !(i16::MIN as f32..=i16::MAX as f32).contains(&code) {
            Err(())
        } else {
            Ok(AdcCode::from(code as i16))
        }
    }
}

// The following data is written by the timer ADC sample trigger into the SPI CR1 to start the
// transfer. Data in AXI SRAM is not initialized on boot, so the contents are random. This value is
// initialized during setup.
#[link_section = ".axisram.buffers"]
static mut SPI_START: MaybeUninit<[u32; 1]> = MaybeUninit::uninit();

// The following data is written by the timer flag clear trigger into the SPI IFCR register to clear
// the EOT flag. Data in AXI SRAM is not initialized on boot, so the contents are random. This
// value is initialized during setup.
#[link_section = ".axisram.buffers"]
static mut SPI_EOT_CLEAR: MaybeUninit<[u32; 1]> = MaybeUninit::uninit();

// The following global buffers are used for the ADC sample DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being acquired while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined. The dimensions are `ADC_BUF[adc_index][ping_pong_index][sample_index]`.
#[link_section = ".axisram.buffers"]
static mut ADC_BUF: MaybeUninit<[[SampleBuffer; 2]; 2]> = MaybeUninit::uninit();

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
                transfer: Transfer<
                    hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                    hal::spi::Spi<hal::stm32::$spi, hal::spi::Disabled, u16>,
                    PeripheralToMemory,
                    &'static mut [u16],
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
                    batch_size: usize,
                ) -> Self {
                    // The flag clear DMA transfer always clears the EOT flag in the SPI
                    // peripheral. It has the highest priority to ensure it is completed before the
                    // transfer trigger.
                    let clear_config = DmaConfig::default()
                        .priority(Priority::VeryHigh)
                        .circular_buffer(true);

                    // Note(unsafe): Because this is a Memory->Peripheral transfer, this data is
                    // never actually modified. It technically only needs to be immutably
                    // borrowed, but the current HAL API only supports mutable borrows.
                    let spi_eot_clear = unsafe {
                        SPI_EOT_CLEAR.write([1 << 3])
                    };

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
                        spi_eot_clear,
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
                    // Note(unsafe): Because this is a Memory->Peripheral transfer, this data is never
                    // actually modified. It technically only needs to be immutably borrowed, but the
                    // current HAL API only supports mutable borrows.
                    // Write a binary code into the SPI control register to initiate a transfer.
                    let spi_start = unsafe {
                        SPI_START.write([0x201])
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
                        spi_start,
                        None,
                        trigger_config,
                    );

                    // The data stream constantly reads from the SPI RX FIFO into a RAM buffer. The peripheral
                    // stalls reads of the SPI RX FIFO until data is available, so the DMA transfer completes
                    // after the requested number of samples have been collected. Note that only ADC1's (sic!)
                    // data stream is used to trigger a transfer completion interrupt.
                    let data_config = DmaConfig::default()
                        .memory_increment(true)
                        .double_buffer(true)
                        .transfer_complete_interrupt($index == 1)
                        .priority(Priority::VeryHigh);

                    // A SPI peripheral error interrupt is used to determine if the RX FIFO
                    // overflows. This indicates that samples were dropped due to excessive
                    // processing time in the main application (e.g. a second DMA transfer completes
                    // before the first was done with processing). This is used as a flow control
                    // indicator to guarantee that no ADC samples are lost.
                    let mut spi = spi.disable();
                    spi.listen(hal::spi::Event::Error);

                    let adc_buf = unsafe {
                        ADC_BUF.write(Default::default())
                    };
                    let adc_bufs = adc_buf[$index].split_at_mut(1);

                    // The data transfer is always a transfer of data from the peripheral to a RAM
                    // buffer.
                    let data_transfer: Transfer<_, _, PeripheralToMemory, _, _> =
                        Transfer::init(
                            data_stream,
                            spi,
                            // Note(unsafe): The ADC_BUF[$index] is "owned" by this peripheral.
                            // It shall not be used anywhere else in the module.
                            &mut adc_bufs.0[0][..batch_size],
                            Some(&mut adc_bufs.1[0][..batch_size]),
                            data_config,
                        );

                    Self {
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

                /// Wait for the transfer of the currently active buffer to complete,
                /// then call a function on the now inactive buffer and acknowledge the
                /// transfer complete flag.
                ///
                /// NOTE(unsafe): Memory safety and access ordering is not guaranteed
                /// (see the HAL DMA docs).
                pub fn with_buffer<F, R>(&mut self, f: F) -> Result<R, DMAError>
                where
                    F: FnOnce(&mut &'static mut [u16]) -> R,
                {
                    unsafe { self.transfer.next_dbm_transfer_with(|buf, _current| f(buf)) }
                }
            }

            // This is not actually a Mutex. It only re-uses the semantics and macros of mutex-trait
            // to reduce rightward drift when jointly calling `with_buffer(f)` on multiple DAC/ADCs.
            impl Mutex for $name {
                type Data = &'static mut [u16];
                fn lock<R>(&mut self, f: impl FnOnce(&mut Self::Data) -> R) -> R {
                    self.with_buffer(f).unwrap()
                }
            }
        }
    };
}

adc_input!(
    Adc0Input, 0, Stream0, Stream1, Stream2, SPI2, Channel1, Tim2Ch1, Channel1,
    Tim3Ch1
);
adc_input!(
    Adc1Input, 1, Stream3, Stream4, Stream5, SPI3, Channel2, Tim2Ch2, Channel2,
    Tim3Ch2
);
