use super::QspiInterface;
use crate::hrtimer::HighResTimerE;
use ad9959::{Channel, DdsConfig, ProfileSerializer};
use stm32h7xx_hal as hal;

pub struct DdsOutput {
    _qspi: QspiInterface,
    io_update_trigger: HighResTimerE,
    config: DdsConfig,
}

impl DdsOutput {
    pub fn new(
        _qspi: QspiInterface,
        io_update_trigger: HighResTimerE,
        dds_config: DdsConfig,
    ) -> Self {
        Self {
            config: dds_config,
            _qspi,
            io_update_trigger,
        }
    }

    pub fn builder(&mut self) -> ProfileBuilder {
        let builder = self.config.builder();
        ProfileBuilder {
            dds_stream: self,
            serializer: builder,
        }
    }

    fn write_profile(&mut self, profile: &[u32]) {
        assert!(profile.len() <= 16);

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

pub struct ProfileBuilder<'a> {
    dds_stream: &'a mut DdsOutput,
    serializer: ProfileSerializer,
}

impl<'a> ProfileBuilder<'a> {
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

    pub fn write_profile(mut self) {
        let profile = self.serializer.finalize();
        self.dds_stream.write_profile(profile);
    }
}
