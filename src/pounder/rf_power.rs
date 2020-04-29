use super::Error;
use super::InputChannel;

pub trait PowerMeasurementInterface {
    fn sample_converter(&mut self, channel: InputChannel) -> Result<f32, Error>;

    fn measure_power(&mut self, channel: InputChannel) -> Result<f32, Error> {
        let analog_measurement = self.sample_converter(channel)?;

        // The AD8363 with VSET connected to VOUT provides an output voltage of 52mV / dB.
        Ok(analog_measurement / 0.052)
    }
}
