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

impl Into<(pounder::Channel, pounder::Channel)> for Channel {
    fn into(self) -> (pounder::Channel, pounder::Channel) {
        match self {
            Self::ZERO => (pounder::Channel::In0, pounder::Channel::Out0),
            Self::ONE => (pounder::Channel::In1, pounder::Channel::Out1),
        }
    }
}

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
    /// `aom_frequency`
    ///
    /// # Value
    /// A positive 32-bit float in the range [1 MHz, 200 Mhz]
    pub aom_frequency: f32,

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

impl PounderFncSettings {
    pub fn new(channel: Channel) -> Self {
        Self {
            aom_frequency: DEFAULT_AOM_FREQUENCY,
            amplitude_dds_out: 1.0,
            amplitude_dds_in: 1.0,
            attenuation_out: 0.5,
            attenuation_in: 0.5,
            channel,
        }
    }

    pub fn update_dds(self, pounder: &mut PounderDevices) -> Result<(), Error> {
        let (dds_in, dds_out) = self.channel.into();

        let mut dds = pounder.dds_output.builder();

        let ftw_in = frequency_to_ftw(
            2.0 * self.aom_frequency,
            DDS_SYSTEM_CLK.to_Hz() as f32,
        )
        .map_err(|_| Error::DdsInUnset)?;
        let acr_in = amplitude_to_acr(self.amplitude_dds_in)
            .map_err(|_| Error::DdsInUnset)?;

        dds.update_channels(dds_in.into(), Some(ftw_in), None, Some(acr_in));
        pounder
            .pounder
            .set_attenuation(dds_in, self.attenuation_in)
            .map_err(|_| Error::AttenuationInUnset)?;

        let ftw_out =
            frequency_to_ftw(self.aom_frequency, DDS_SYSTEM_CLK.to_Hz() as f32)
                .map_err(|_| Error::DdsOutUnset)?;
        let acr_out = amplitude_to_acr(self.amplitude_dds_out)
            .map_err(|_| Error::DdsOutUnset)?;

        dds.update_channels(dds_out.into(), Some(ftw_out), None, Some(acr_out));
        pounder
            .pounder
            .set_attenuation(dds_out, self.attenuation_out)
            .map_err(|_| Error::AttenuationOutUnset)?;

        dds.write();

        Ok(())
    }
}
