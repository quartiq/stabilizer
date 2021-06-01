use super::{Channel, Error};

/// Provide an interface to measure RF input power in dBm.
pub trait PowerMeasurementInterface {
    fn sample_converter(&mut self, channel: Channel) -> Result<f32, Error>;

    /// Measure the power of an input channel in dBm.
    ///
    /// Args:
    /// * `channel` - The pounder input channel to measure the power of.
    ///
    /// Returns:
    /// Power in dBm after the digitally controlled attenuator before the amplifier.
    fn measure_power(&mut self, channel: Channel) -> Result<f32, Error> {
        let analog_measurement = self.sample_converter(channel)?;

        // The AD8363 with VSET connected to VOUT provides an output voltage of 51.7 mV/dB at
        // 100MHz with an intercept of -58 dBm.
        // It is placed behind a 20 dB tap.
        Ok(analog_measurement * (1. / 0.0517) + (-58. + 20.))
    }
}
