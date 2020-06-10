use super::{Error, Channel};

pub trait PowerMeasurementInterface {
    fn sample_converter(&mut self, channel: Channel) -> Result<f32, Error>;

    fn measure_power(&mut self, channel: Channel) -> Result<f32, Error> {
        let analog_measurement = self.sample_converter(channel)?;

        // The AD8363 with VSET connected to VOUT provides an output voltage of 51.7mV / dB at
        // 100MHz.
        Ok(analog_measurement / 0.0517)
    }
}
