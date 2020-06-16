use super::{Channel, Error};

/// Provide an interface for managing digital attenuators on Pounder hardware.
///
/// Note: The digital attenuators do not allow read-back of attenuation. To circumvent this, this
/// driver maintains the attenuation code in both the shift register as well as the latched output
/// register of the attenuators. This allows the "active" attenuation code to be read back by
/// reading the shfit register. The downside of this approach is that any read is destructive, so a
/// read-writeback approach is employed.
pub trait AttenuatorInterface {
    /// Set the attenuation of a single channel.
    ///
    /// Args:
    /// * `channel` - The pounder channel to configure the attenuation of.
    /// * `attenuation` - The desired attenuation of the channel in dB. This has a resolution of
    ///   0.5dB.
    fn set_attenuation(
        &mut self,
        channel: Channel,
        attenuation: f32,
    ) -> Result<f32, Error> {
        if attenuation > 31.5 || attenuation < 0.0 {
            return Err(Error::Bounds);
        }

        // Calculate the attenuation code to program into the attenuator. The attenuator uses a
        // code where the LSB is 0.5 dB.
        let attenuation_code = (attenuation * 2.0) as u8;

        // Read all the channels, modify the channel of interest, and write all the channels back.
        // This ensures the staging register and the output register are always in sync.
        let mut channels = [0_u8; 4];
        self.read_all_attenuators(&mut channels)?;

        // The lowest 2 bits of the 8-bit shift register on the attenuator are ignored. Shift the
        // attenuator code into the upper 6 bits of the register value. Note that the attenuator
        // treats inputs as active-low, so the code is inverted before writing.
        channels[channel as usize] = (!attenuation_code) << 2;
        self.write_all_attenuators(&channels)?;

        // Finally, latch the output of the updated channel to force it into an active state.
        self.latch_attenuators(channel)?;

        Ok(attenuation_code as f32 / 2.0)
    }

    /// Get the attenuation of a channel.
    ///
    /// Args:
    /// * `channel` - The channel to get the attenuation of.
    ///
    /// Returns:
    /// The programmed attenuation of the channel in dB.
    fn get_attenuation(&mut self, channel: Channel) -> Result<f32, Error> {
        let mut channels = [0_u8; 4];

        // Reading the data always shifts data out of the staging registers, so we perform a
        // duplicate write-back to ensure the staging register is always equal to the output
        // register.
        self.read_all_attenuators(&mut channels)?;
        self.write_all_attenuators(&channels)?;

        // The attenuation code is stored in the upper 6 bits of the register, where each LSB
        // represents 0.5 dB. The attenuator stores the code as active-low, so inverting the result
        // (before the shift) has the affect of transforming the bits of interest (and the
        // dont-care bits) into an active-high state and then masking off the don't care bits. If
        // the shift occurs before the inversion, the upper 2 bits (which would then be don't
        // care) would contain erroneous data.
        let attenuation_code = (!channels[channel as usize]) >> 2;

        // Convert the desired channel code into dB of attenuation.
        Ok(attenuation_code as f32 / 2.0)
    }

    fn reset_attenuators(&mut self) -> Result<(), Error>;

    fn latch_attenuators(&mut self, channel: Channel) -> Result<(), Error>;
    fn read_all_attenuators(
        &mut self,
        channels: &mut [u8; 4],
    ) -> Result<(), Error>;
    fn write_all_attenuators(
        &mut self,
        channels: &[u8; 4],
    ) -> Result<(), Error>;
}
