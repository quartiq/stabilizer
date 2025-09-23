use crate::convert::{AdcCode, DacCode, Gain};
use serde::Serialize;

/// The telemetry buffer is used for storing sample values during execution.
///
/// # Note
/// These values can be converted to SI units immediately before reporting to save processing time.
/// This allows for the DSP process to continually update the values without incurring significant
/// run-time overhead during conversion to SI units.
#[derive(Clone, Default)]
pub struct TelemetryBuffer {
    /// The latest input sample on ADC0/ADC1.
    pub adcs: [AdcCode; 2],
    /// The latest output code on DAC0/DAC1.
    pub dacs: [DacCode; 2],
    /// The latest digital input states during processing.
    pub digital_inputs: [bool; 2],
}

/// The telemetry structure is data that is ultimately reported as telemetry over MQTT.
///
/// # Note
/// This structure should be generated on-demand by the buffer when required to minimize conversion
/// overhead.
#[derive(Serialize)]
pub struct Telemetry {
    /// Most recent input voltage measurement.
    pub adcs: [f32; 2],

    /// Most recent output voltage.
    pub dacs: [f32; 2],

    /// Most recent digital input assertion state.
    pub digital_inputs: [bool; 2],

    /// The CPU temperature in degrees Celsius.
    pub cpu_temp: f32,
}

impl TelemetryBuffer {
    /// Convert the telemetry buffer to finalized, SI-unit telemetry for reporting.
    ///
    /// # Args
    /// * `afe0` - The current AFE configuration for channel 0.
    /// * `afe1` - The current AFE configuration for channel 1.
    /// * `cpu_temp` - The current CPU temperature.
    ///
    /// # Returns
    /// The finalized telemetry structure that can be serialized and reported.
    pub fn finalize(self, afe0: Gain, afe1: Gain, cpu_temp: f32) -> Telemetry {
        let in0_volts = f32::from(self.adcs[0]) / afe0.gain();
        let in1_volts = f32::from(self.adcs[1]) / afe1.gain();

        Telemetry {
            cpu_temp,
            adcs: [in0_volts, in1_volts],
            dacs: [self.dacs[0].into(), self.dacs[1].into()],
            digital_inputs: self.digital_inputs,
        }
    }
}
