use super::{Channel, Error};

/// Provide an interface to measure RF input power in dB.
pub trait PowerMeasurementInterface {
    fn sample_converter(&mut self, channel: Channel) -> Result<f32, Error>;

    /// Measure the power of an input channel in dBm.
    ///
    /// Note: This function assumes the input channel is connected to an AD8363 output.
    ///
    /// Args:
    /// * `channel` - The pounder channel to measure the power of in dBm.
    fn measure_power(&mut self, channel: Channel) -> Result<f32, Error> {
        let analog_measurement = self.sample_converter(channel)?;

        // The AD8363 with VSET connected to VOUT provides an output voltage of 51.7mV / dB at
        // 100MHz. It also indicates  a y-intercept of -58dBm.
        Ok(analog_measurement / 0.0517 - 58.0)
    }
}
