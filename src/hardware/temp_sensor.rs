use stm32h7xx_hal::{
    self as hal,
    signature::{TS_CAL_110, TS_CAL_30},
};

use super::shared_adc::{AdcError, AdcChannel};

struct Calibration {
    slope: f32,
    offset: f32,
}

impl Calibration {
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

pub struct CpuTempSensor {
    sensor: AdcChannel<'static, hal::stm32::ADC3, hal::adc::Temperature>,
    calibration: Calibration,
}

impl CpuTempSensor {
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
        self.sensor.read_raw().map(|raw| self.calibration.sample_to_temperature(raw))
    }
}
