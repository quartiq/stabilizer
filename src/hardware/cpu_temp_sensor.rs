//! STM32 Temperature Sensor Driver
//!
//! # Description
//! This file provides an API for measuring the internal STM32 temperature sensor. This temperature
//! sensor measures the silicon junction temperature (Tj) and is connected via an internal ADC.
use super::hal::{
    self,
    signature::{TS_CAL_30, TS_CAL_110},
};

use super::shared_adc::{AdcChannel, AdcError};

/// Helper utility to convert raw codes into temperature measurements.
struct Calibration {
    slope: f32,
    offset: f32,
}

impl Calibration {
    /// Construct the calibration utility.
    pub fn new() -> Self {
        let ts_cal2 = TS_CAL_110::read();
        let ts_cal1 = TS_CAL_30::read();
        let slope = (110. - 30.) / (ts_cal2 as f32 - ts_cal1 as f32);
        let offset = 30. - slope * ts_cal1 as f32;
        Self { slope, offset }
    }

    /// Convert a raw ADC sample to a temperature in degrees Celsius.
    pub fn sample_to_temperature(&self, sample: u32) -> f32 {
        // We use a 2.048V reference, but calibration data was taken at 3.3V.
        let sample_3v3 = sample as f32 * 2.048 / 3.3;

        self.slope * sample_3v3 + self.offset
    }
}

/// A driver to access the CPU temeprature sensor.
pub struct CpuTempSensor {
    sensor: AdcChannel<'static, hal::stm32::ADC3, hal::adc::Temperature>,
    calibration: Calibration,
}

impl CpuTempSensor {
    /// Construct the temperature sensor.
    ///
    /// # Args
    /// * `sensor` - The ADC channel of the integrated temperature sensor.
    pub fn new(
        sensor: AdcChannel<'static, hal::stm32::ADC3, hal::adc::Temperature>,
    ) -> Self {
        Self {
            sensor,
            calibration: Calibration::new(),
        }
    }

    /// Get the temperature of the CPU in degrees Celsius.
    pub fn get_temperature(&mut self) -> Result<f32, AdcError> {
        let t = self.sensor.read_raw()?;
        Ok(self.calibration.sample_to_temperature(t))
    }
}
