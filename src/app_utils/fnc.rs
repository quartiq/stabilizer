//! Fibre noise cancellation (bin/fnc.rs) utils

use ad9959::{self, amplitude_to_acr, frequency_to_ftw};
use miniconf::Tree;
use serde::{Deserialize, Serialize};

use crate::hardware::{
    design_parameters::DDS_SYSTEM_CLK,
    pounder::{self, attenuators::AttenuatorInterface},
    setup::PounderDevices,
};

const DEFAULT_AOM_FREQUENCY: f32 = 80_000_000.0;

#[derive(Copy, Clone, Serialize, Deserialize, Debug)]
pub enum Channel {
    ZERO = 0,
    ONE = 1,
}

/// Convert `fnc::Channel` to `pounder::Channel`s (in, out)
impl Into<(pounder::Channel, pounder::Channel)> for Channel {
    fn into(self) -> (pounder::Channel, pounder::Channel) {
        match self {
            Self::ZERO => (pounder::Channel::In0, pounder::Channel::Out0),
            Self::ONE => (pounder::Channel::In1, pounder::Channel::Out1),
        }
    }
}

#[derive(Debug)]
pub enum Error {
    DdsInUnset,
    DdsOutUnset,
    AttenuationInUnset,
    AttenuationOutUnset,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, Tree)]
pub struct PounderFncSettings {
    /// Specifies the centre frequency of the fnc double-pass AOM in hertz
    ///
    /// # Path
    /// `frequency_dds_out`
    ///
    /// # Value
    /// A positive 32-bit float in the range [1 MHz, 200 Mhz]
    pub frequency_dds_out: f32,

    /// Specifies the centre frequency of the fnc double-pass AOM in hertz
    ///
    /// # Path
    /// `frequency_dds_in`
    ///
    /// # Value
    /// A positive 32-bit float in the range [1 MHz, 200 Mhz]
    pub frequency_dds_in: f32,

    /// Specifies the amplitude of the dds output driving the aom relative to max (10 dBm)
    ///
    /// # Path
    /// `amplitude_out`
    ///
    /// # Value
    /// A positive 32-bit float in the range [0.0, 1.0]
    pub amplitude_dds_out: f32,

    /// Specifies the amplitude of the dds output to mix down the error signal relative to max (10 dBm)
    ///
    /// # Path
    /// `amplitude_in`
    ///
    /// # Value
    /// A positive 32-bit float in the range [0.0, 1.0]
    pub amplitude_dds_in: f32,

    /// Specifies the attenuation applied to the output channel driving the aom (dB)
    ///
    /// # Path
    /// `attenuation_out`
    ///
    /// # Value
    /// A positive 32-bit float in the range [0.5, 31.5] in steps of 0.5
    pub attenuation_out: f32,

    /// Specifies the attenuation applied to the input channel from the photodiode (dB)
    ///
    /// # Path
    /// `attenuation_in`
    ///
    /// # Value
    /// A positive 32-bit float in the range [0.5, 31.5] in steps of 0.5
    pub attenuation_in: f32,

    /// Specifies the FNC channel being used
    #[tree(skip)]
    pub channel: Channel,
}

impl Default for PounderFncSettings {
    fn default() -> Self {
        Self {
            frequency_dds_out: DEFAULT_AOM_FREQUENCY,
            frequency_dds_in: 2.0 * DEFAULT_AOM_FREQUENCY,
            amplitude_dds_out: 0.1,
            amplitude_dds_in: 0.1,
            attenuation_out: 31.5,
            attenuation_in: 31.5,
            channel: Channel::ZERO,
        }
    }
}

impl PounderFncSettings {
    pub fn new(channel: Channel) -> Self {
        Self {
            channel,
            ..Default::default()
        }
    }

    /// Get the dds frequency and amplitude words for an fnc settings update
    ///
    /// Returns:
    /// Result<(ftw_in, acr_in, ftw_out, acr_out), Error>
    ///
    pub fn get_dds_words(self) -> Result<(u32, u32, u32, u32), Error> {
        let ftw_in = frequency_to_ftw(
            self.frequency_dds_in,
            DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .map_err(|_| Error::DdsInUnset)?;
        let acr_in = amplitude_to_acr(self.amplitude_dds_in)
            .map_err(|_| Error::DdsInUnset)?;
        let ftw_out = frequency_to_ftw(
            self.frequency_dds_out,
            DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .map_err(|_| Error::DdsOutUnset)?;
        let acr_out = amplitude_to_acr(self.amplitude_dds_out)
            .map_err(|_| Error::DdsOutUnset)?;

        Ok((ftw_in, acr_in, ftw_out, acr_out))
    }

    pub fn set_all_dds(
        self,
        pounder: &mut PounderDevices,
    ) -> Result<(), Error> {
        let (dds_in, dds_out) = self.channel.into();
        let (ftw_in, acr_in, ftw_out, acr_out) = self.get_dds_words()?;

        let mut dds = pounder.dds_output.builder();

        dds.update_channels(dds_in.into(), Some(ftw_in), None, Some(acr_in));
        pounder
            .pounder
            .set_attenuation(dds_in, self.attenuation_in)
            .map_err(|_| Error::AttenuationInUnset)?;

        dds.update_channels(dds_out.into(), Some(ftw_out), None, Some(acr_out));
        pounder
            .pounder
            .set_attenuation(dds_out, self.attenuation_out)
            .map_err(|_| Error::AttenuationOutUnset)?;

        dds.write();

        Ok(())
    }
}
