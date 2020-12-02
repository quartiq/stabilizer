///! The DdsOutput is used as an output stream to the pounder DDS.
use super::QspiInterface;
use crate::hrtimer::HighResTimerE;
use ad9959::{Channel, DdsConfig, ProfileSerializer};
use stm32h7xx_hal as hal;

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
    /// * `dds_config` - The frozen DDS configuration.
    pub fn new(
        mut qspi: QspiInterface,
        io_update_trigger: HighResTimerE,
        dds_config: DdsConfig,
    ) -> Self {
        qspi.start_stream();
        Self {
            config: dds_config,
            _qspi,
            io_update_trigger,
        }
    }

    /// Get a builder for serializing a Pounder DDS profile.
    pub fn builder(&mut self) -> ProfileBuilder {
        let builder = self.config.builder();
        ProfileBuilder {
            dds_stream: self,
            serializer: builder,
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
    dds_stream: &'a mut DdsOutput,
    serializer: ProfileSerializer,
}

impl<'a> ProfileBuilder<'a> {
    /// Update a number of channels with the provided configuration
    ///
    /// # Args
    /// * `channels` - A list of channels to apply the configuration to.
    /// * `ftw` - If provided, indicates a frequency tuning word for the channels.
    /// * `pow` - If provided, indicates a phase offset word for the channels.
    /// * `acr` - If provided, indicates the amplitude control register for the channels.
    pub fn update_channels(
        mut self,
        channels: &[Channel],
        ftw: Option<u32>,
        pow: Option<u16>,
        acr: Option<u16>,
    ) -> Self {
        self.serializer.update_channels(channels, ftw, pow, acr);
        self
    }

    /// Write the profile to the DDS asynchronously.
    pub fn write_profile(mut self) {
        let profile = self.serializer.finalize();
        self.dds_stream.write_profile(profile);
    }
}
