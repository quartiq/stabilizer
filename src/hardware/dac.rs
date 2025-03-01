//! Stabilizer DAC management interface
//!
//! # Design
//!
//! Stabilizer DACs are connected to the MCU via a simplex, SPI-compatible interface. Each DAC
//! accepts a 16-bit output code.
//!
//! In order to maximize CPU processing time, the DAC code updates are offloaded to hardware using
//! a timer compare channel, DMA stream, and the DAC SPI interface.
//!
//! The timer comparison channel is configured to generate a DMA request whenever the comparison
//! occurs. Thus, whenever a comparison happens, a single DAC code can be written to the output. By
//! configuring a DMA stream for a number of successive DAC codes, hardware can regularly update
//! the DAC without requiring the CPU.
//!
//! In order to ensure alignment between the ADC sample batches and DAC output code batches, a DAC
//! output batch is always exactly 3 batches after the ADC batch that generated it.
//!
//! The DMA transfer for the DAC output codes utilizes a double-buffer mode to avoid losing any
//! transfer events generated by the timer (for example, when 2 update cycles occur before the DMA
//! transfer completion is handled). In this mode, by the time DMA swaps buffers, there is always a valid buffer in the
//! "next-transfer" double-buffer location for the DMA transfer. Once a transfer completes,
//! software then has exactly one batch duration to fill the next buffer before its
//! transfer begins. If software does not meet this deadline, old data will be repeatedly generated
//! on the output and output will be shifted by one batch.
//!
//! ## Multiple Samples to Single DAC Codes
//!
//! For some applications, it may be desirable to generate a single DAC code from multiple ADC
//! samples. In order to maintain timing characteristics between ADC samples and DAC code outputs,
//! applications are required to generate one DAC code for each ADC sample. To accomodate mapping
//! multiple inputs to a single output, the output code can be repeated a number of times in the
//! output buffer corresponding with the number of input samples that were used to generate it.
//!
//!
//! # Note
//!
//! There is a very small amount of latency between updating the two DACs due to bus matrix
//! priority. As such, one of the DACs will be updated marginally earlier before the other because
//! the DMA requests are generated simultaneously. This can be avoided by providing a known offset
//! to other DMA requests, which can be completed by setting e.g. DAC0's comparison to a
//! counter value of 2 and DAC1's comparison to a counter value of 3. This will have the effect of
//! generating the DAC updates with a known latency of 1 timer tick to each other and prevent the
//! DMAs from racing for the bus. As implemented, the DMA channels utilize natural priority of the
//! DMA channels to arbitrate which transfer occurs first.
//!
//!
//! # Limitations
//!
//! While double-buffered mode is used for DMA to avoid lost DAC-update events, there is no check
//! for re-use of a previously provided DAC output buffer. It is assumed that the DMA request is
//! served promptly after the transfer completes.
use stm32h7xx_hal as hal;

use rtic::Mutex;

use super::design_parameters::{SampleBuffer, MAX_SAMPLE_BUFFER_SIZE};
use super::timers;

use core::convert::TryFrom;

use hal::{
    dma::{
        dma::{DMAReq, DmaConfig},
        traits::TargetAddress,
        DMAError, MemoryToPeripheral, Transfer,
    },
    spi::{HalDisabledSpi, HalEnabledSpi, HalSpi},
};

// The following global buffers are used for the DAC code DMA transfers. Two buffers are used for
// each transfer in a ping-pong buffer configuration (one is being prepared while the other is being
// processed). Note that the contents of AXI SRAM is uninitialized, so the buffer contents on
// startup are undefined. The dimensions are `ADC_BUF[adc_index][ping_pong_index][sample_index]`.
#[link_section = ".axisram.buffers"]
static mut DAC_BUF: [[SampleBuffer; 2]; 2] =
    [[[0; MAX_SAMPLE_BUFFER_SIZE]; 2]; 2];

/// Custom type for referencing DAC output codes.
/// The internal integer is the raw code written to the DAC output register.
#[derive(Copy, Clone, Default)]
pub struct DacCode(pub u16);
impl DacCode {
    // The DAC output range in bipolar mode (including the external output op-amp) is +/- 4.096
    // V with 16-bit resolution. The anti-aliasing filter has an additional gain of 2.5.
    pub const FULL_SCALE: f32 = 4.096 * 2.5;
    pub const VOLT_PER_LSB: f32 = -Self::FULL_SCALE / i16::MIN as f32;
    pub const LSB_PER_VOLT: f32 = 1. / Self::VOLT_PER_LSB;
}

impl TryFrom<f32> for DacCode {
    type Error = ();

    fn try_from(voltage: f32) -> Result<DacCode, ()> {
        let code = voltage * Self::LSB_PER_VOLT;
        if !(i16::MIN as f32..=i16::MAX as f32).contains(&code) {
            Err(())
        } else {
            Ok(DacCode::from(code as i16))
        }
    }
}

impl From<DacCode> for f32 {
    fn from(code: DacCode) -> f32 {
        i16::from(code) as f32 * DacCode::VOLT_PER_LSB
    }
}

impl From<DacCode> for i16 {
    fn from(code: DacCode) -> i16 {
        (code.0 as i16).wrapping_sub(i16::MIN)
    }
}

impl From<i16> for DacCode {
    /// Encode signed 16-bit values into DAC offset binary for a bipolar output configuration.
    fn from(value: i16) -> Self {
        Self(value.wrapping_add(i16::MIN) as u16)
    }
}

impl From<u16> for DacCode {
    /// Create a dac code from the provided DAC output code.
    fn from(value: u16) -> Self {
        Self(value)
    }
}

macro_rules! dac_output {
    ($name:ident, $index:literal, $data_stream:ident,
     $spi:ident, $trigger_channel:ident, $dma_req:ident) => {
        /// $spi is used as a type for indicating a DMA transfer into the SPI TX FIFO
        struct $spi {
            spi: hal::spi::Spi<hal::stm32::$spi, hal::spi::Disabled, u16>,
            _channel: timers::tim2::$trigger_channel,
        }

        impl $spi {
            pub fn new(
                _channel: timers::tim2::$trigger_channel,
                spi: hal::spi::Spi<hal::stm32::$spi, hal::spi::Disabled, u16>,
            ) -> Self {
                Self { spi, _channel }
            }

            /// Start the SPI and begin operating in a DMA-driven transfer mode.
            pub fn start_dma(&mut self) {
                // Allow the SPI FIFOs to operate using only DMA data channels.
                self.spi.enable_dma_tx();

                // Enable SPI and start it in infinite transaction mode.
                self.spi.inner().cr1.modify(|_, w| w.spe().set_bit());
                self.spi.inner().cr1.modify(|_, w| w.cstart().started());
            }
        }

        // Note(unsafe): This is safe because the DMA request line is logically owned by this module.
        // Additionally, the SPI is owned by this structure and is known to be configured for u16 word
        // sizes.
        unsafe impl TargetAddress<MemoryToPeripheral> for $spi {
            /// SPI is configured to operate using 16-bit transfer words.
            type MemSize = u16;

            /// SPI DMA requests are generated whenever TIM2 CHx ($dma_req) comparison occurs.
            const REQUEST_LINE: Option<u8> = Some(DMAReq::$dma_req as u8);

            /// Whenever the DMA request occurs, it should write into SPI's TX FIFO.
            fn address(&self) -> usize {
                &self.spi.inner().txdr as *const _ as usize
            }
        }

        /// Represents data associated with DAC.
        pub struct $name {
            // Note: SPI TX functionality may not be used from this structure to ensure safety with DMA.
            transfer: Transfer<
                hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                $spi,
                MemoryToPeripheral,
                &'static mut [u16],
                hal::dma::DBTransfer,
            >,
        }

        impl $name {
            /// Construct the DAC output channel.
            ///
            /// # Args
            /// * `spi` - The SPI interface used to communicate with the ADC.
            /// * `stream` - The DMA stream used to write DAC codes over SPI.
            /// * `trigger_channel` - The sampling timer output compare channel for update triggers.
            pub fn new(
                spi: hal::spi::Spi<hal::stm32::$spi, hal::spi::Enabled, u16>,
                stream: hal::dma::dma::$data_stream<hal::stm32::DMA1>,
                trigger_channel: timers::tim2::$trigger_channel,
                batch_size: usize,
            ) -> Self {
                // Generate DMA events when an output compare of the timer hitting zero (timer roll over)
                // occurs.
                trigger_channel.listen_dma();
                trigger_channel.to_output_compare(4 + $index);

                // The stream constantly writes to the TX FIFO to write new update codes.
                let trigger_config = DmaConfig::default()
                    .memory_increment(true)
                    .double_buffer(true)
                    .peripheral_increment(false);

                // Listen for any potential SPI error signals, which may indicate that we are not generating
                // update codes.
                let mut spi = spi.disable();
                spi.listen(hal::spi::Event::Error);

                // AXISRAM is uninitialized. As such, we manually initialize it for a 0V DAC output
                // here before starting the transfer .
                // Note(unsafe): We currently own all DAC_BUF[index] buffers and are not using them
                // elsewhere, so it is safe to access them here.
                for buf in unsafe { DAC_BUF[$index].iter_mut() } {
                    for byte in buf.iter_mut() {
                        *byte = DacCode::try_from(0.0f32).unwrap().0;
                    }
                }

                // Construct the trigger stream to write from memory to the peripheral.
                let transfer: Transfer<_, _, MemoryToPeripheral, _, _> =
                    Transfer::init(
                        stream,
                        $spi::new(trigger_channel, spi),
                        // Note(unsafe): This buffer is only used once and provided for the DMA transfer.
                        unsafe { &mut DAC_BUF[$index][0][..batch_size] },
                        // Note(unsafe): This buffer is only used once and provided for the DMA transfer.
                        unsafe { Some(&mut DAC_BUF[$index][1][..batch_size]) },
                        trigger_config,
                    );

                Self { transfer }
            }

            pub fn start(&mut self) {
                self.transfer.start(|spi| spi.start_dma());
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
                unsafe {
                    self.transfer.next_dbm_transfer_with(|buf, _current| f(buf))
                }
            }
        }

        // This is not actually a Mutex. It only re-uses the semantics and macros of mutex-trait
        // to reduce rightward drift when jointly calling `with_buffer(f)` on multiple DAC/ADCs.
        impl Mutex for $name {
            type T = &'static mut [u16];
            fn lock<R>(&mut self, f: impl FnOnce(&mut Self::T) -> R) -> R {
                self.with_buffer(f).unwrap()
            }
        }
    };
}

dac_output!(Dac0Output, 0, Stream6, SPI4, Channel3, Tim2Ch3);
dac_output!(Dac1Output, 1, Stream7, SPI5, Channel4, Tim2Ch4);
