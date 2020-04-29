use super::error::Error;
use super::DdsChannel;

pub trait AttenuatorInterface {
    fn modify(&mut self, attenuation: f32, channel: DdsChannel) -> Result<f32, Error> {
        if attenuation > 31.5 {
            return Err(Error::Bounds);
        }

        // Calculate the attenuation code to program into the attenuator.
        let attenuation_code = (attenuation * 2.0) as u8;

        // Read all the channels, modify the channel of interest, and write all the channels back.
        // This ensures the staging register and the output register are always in sync.
        let mut channels = [0_u8; 4];
        self.read_all(&mut channels)?;
        channels[channel as usize] = attenuation_code;
        self.write_all(&channels)?;

        // Finally, latch the output of the updated channel to force it into an active state.
        self.latch(channel)?;

        Ok(attenuation_code as f32 / 2.0)
    }

    fn read(&mut self, channel: DdsChannel) -> Result<f32, Error> {
        let mut channels = [0_u8; 4];

        // Reading the data always shifts data out of the staging registers, so we perform a
        // duplicate write-back to ensure the staging register is always equal to the output
        // register.
        self.read_all(&mut channels)?;
        self.write_all(&channels)?;

        // Convert the desired channel code into dB of attenuation.
        let attenuation_code = channels[channel as usize];

        Ok(attenuation_code as f32 / 2.0)
    }

    fn reset(&mut self) -> Result<(), Error>;

    fn latch(&mut self, channel: DdsChannel) -> Result<(), Error>;
    fn read_all(&mut self, channels: &mut [u8; 4]) -> Result<(), Error>;
    fn write_all(&mut self, channels: &[u8; 4]) -> Result<(), Error>;
}
