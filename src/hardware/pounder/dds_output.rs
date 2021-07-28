///! The DdsOutput is used as an output stream to the pounder DDS.
///!
///! # Design
///!
///! The DDS stream interface is a means of quickly updating pounder DDS (direct digital synthesis)
///! outputs of the AD9959 DDS chip. The DDS communicates via a quad-SPI interface and a single
///! IO-update output pin.
///!
///! In order to update the DDS interface, the frequency tuning word, amplitude control word, and
///! the phase offset word for a channel can be modified to change the frequency, amplitude, or
///! phase on any of the 4 available output channels. Changes do not propagate to DDS outputs until
///! the IO-update pin is toggled high to activate the new configurations. This allows multiple
///! channels or parameters to be updated and then effects can take place simultaneously.
///!
///! In this implementation, the phase, frequency, or amplitude can be updated for any single
///! collection of outputs simultaneously. This is done by serializing the register writes to the
///! DDS into a single buffer of data and then writing the data over QSPI to the DDS.
///!
///! In order to minimize software overhead, data is written directly into the QSPI output FIFO. In
///! order to accomplish this most efficiently, serialized data is written as 32-bit words to
///! minimize the number of bus cycles necessary to write to the peripheral FIFO. A consequence of
///! this is that additional unneeded register writes may be appended to align a transfer to 32-bit
///! word sizes.
///!
///! In order to pulse the IO-update signal, the high-resolution timer output is used. The timer is
///! configured to assert the IO-update signal after a predefined delay and then de-assert the
///! signal after a predefined assertion duration. This allows for the actual QSPI transfer and
///! IO-update toggle to be completed asynchronously to the rest of software processing - that is,
///! software can schedule the DDS updates and then continue data processing. DDS updates then take
///! place in the future when the IO-update is toggled by hardware.
///!
///!
///! # Limitations
///!
///! The QSPI output FIFO is used as an intermediate buffer for holding pending QSPI writes. Because
///! of this, the implementation only supports up to 16 serialized bytes (the QSPI FIFO is 4 32-bit
///! words wide) in a single update.
///!
///! There is currently no synchronization between completion of the QSPI data write and the
///! IO-update signal. It is currently assumed that the QSPI transfer will always complete within a
///! predefined delay (the pre-programmed IO-update timer delay).
///!
///!
///! # Future Improvement
///!
///! In the future, it would be possible to utilize a DMA transfer to complete the QSPI transfer.
///! Once the QSPI transfer completed, this could trigger the IO-update timer to start to
///! asynchronously complete IO-update automatically. This would allow for arbitrary profile sizes
///! and ensure that IO-update was in-sync with the QSPI transfer.
///!
///! Currently, serialization is performed on each processing cycle. If there is a
///! compile-time-known register update sequence needed for the application, the serialization
///! process can be done once and then register values can be written into a pre-computed serialized
///! buffer to avoid the software overhead of much of the serialization process.
use log::warn;
use stm32h7xx_hal as hal;

use super::{hrtimer::HighResTimerE, QspiInterface};
use ad9959::{Channel, DdsConfig, ProfileSerializer};

/// The DDS profile update stream.
pub struct DdsOutput {
    _qspi: QspiInterface,
    io_update_trigger: HighResTimerE,
    config: DdsConfig,
}

impl DdsOutput {
    /// Construct a new DDS output stream.
    ///
    /// # Note
    /// It is assumed that the QSPI stream and the IO_Update trigger timer have been configured in a
    /// way such that the profile has sufficient time to be written before the IO_Update signal is
    /// generated.
    ///
    /// # Args
    /// * `qspi` - The QSPI interface to the run the stream on.
    /// * `io_update_trigger` - The HighResTimerE used to generate IO_Update pulses.
    /// * `config` - The frozen DDS configuration.
    pub fn new(
        mut qspi: QspiInterface,
        io_update_trigger: HighResTimerE,
        config: DdsConfig,
    ) -> Self {
        qspi.start_stream().unwrap();
        Self {
            config,
            _qspi: qspi,
            io_update_trigger,
        }
    }

    /// Get a builder for serializing a Pounder DDS profile.
    #[allow(dead_code)]
    pub fn builder(&mut self) -> ProfileBuilder {
        let serializer = self.config.serializer();
        ProfileBuilder {
            dds_output: self,
            serializer,
        }
    }

    /// Write a profile to the stream.
    ///
    /// # Note:
    /// If a profile of more than 4 words is provided, it is possible that the QSPI interface will
    /// stall execution.
    ///
    /// # Args
    /// * `profile` - The serialized DDS profile to write.
    fn write_profile(&mut self, profile: &[u32]) {
        // Note(unsafe): We own the QSPI interface, so it is safe to access the registers in a raw
        // fashion.
        let regs = unsafe { &*hal::stm32::QUADSPI::ptr() };

        if regs.sr.read().flevel() != 0 {
            warn!("QSPI stalling")
        }

        for word in profile.iter() {
            // Note(unsafe): We are writing to the SPI TX FIFO in a raw manner for performance. This
            // is safe because we know the data register is a valid address to write to.
            unsafe {
                core::ptr::write_volatile(
                    &regs.dr as *const _ as *mut u32,
                    *word,
                );
            }
        }

        // Trigger the IO_update signal generating timer to asynchronous create the IO_Update pulse.
        self.io_update_trigger.trigger();
    }
}

/// A temporary builder for serializing and writing profiles.
pub struct ProfileBuilder<'a> {
    dds_output: &'a mut DdsOutput,
    serializer: ProfileSerializer,
}

impl<'a> ProfileBuilder<'a> {
    /// Update a number of channels with the provided configuration
    ///
    /// # Args
    /// * `channels` - A list of channels to apply the configuration to.
    /// * `ftw` - If provided, indicates a frequency tuning word for the channels.
    /// * `pow` - If provided, indicates a phase offset word for the channels.
    /// * `acr` - If provided, indicates the amplitude control register for the channels. The
    ///   24-bits of the ACR should be stored in the last 3 LSB.
    #[allow(dead_code)]
    pub fn update_channels(
        mut self,
        channels: &[Channel],
        ftw: Option<u32>,
        pow: Option<u16>,
        acr: Option<u32>,
    ) -> Self {
        self.serializer.update_channels(channels, ftw, pow, acr);
        self
    }

    /// Write the profile to the DDS asynchronously.
    #[allow(dead_code)]
    pub fn write_profile(mut self) {
        let profile = self.serializer.finalize();
        self.dds_output.write_profile(profile);
    }
}
