//! Stabilizer Telemetry Capabilities
//!
//! # Design
//! Telemetry is reported regularly using an MQTT client. All telemetry is reported in SI units
//! using standard JSON format.
//!
//! In order to report ADC/DAC codes generated during the DSP routines, a telemetry buffer is
//! employed to track the latest codes. Converting these codes to SI units would result in
//! repetitive and unnecessary calculations within the DSP routine, slowing it down and limiting
//! sampling frequency. Instead, the raw codes are stored and the telemetry is generated as
//! required immediately before transmission. This ensures that any slower computation required
//! for unit conversion can be off-loaded to lower priority tasks.
use heapless::{String, Vec};
use serde::Serialize;

use super::NetworkReference;
use crate::hardware::{adc::AdcCode, afe::Gain, dac::DacCode, SystemTimer};

/// The telemetry client for reporting telemetry data over MQTT.
pub struct TelemetryClient<T: Serialize> {
    mqtt: minimq::Minimq<
        'static,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
    >,
    telemetry_topic: String<128>,
    _telemetry: core::marker::PhantomData<T>,
}

/// The telemetry buffer is used for storing sample values during execution.
///
/// # Note
/// These values can be converted to SI units immediately before reporting to save processing time.
/// This allows for the DSP process to continually update the values without incurring significant
/// run-time overhead during conversion to SI units.
#[derive(Copy, Clone)]
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

impl Default for TelemetryBuffer {
    fn default() -> Self {
        Self {
            adcs: [AdcCode(0), AdcCode(0)],
            dacs: [DacCode(0), DacCode(0)],
            digital_inputs: [false, false],
        }
    }
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
        let in0_volts = Into::<f32>::into(self.adcs[0]) / afe0.as_multiplier();
        let in1_volts = Into::<f32>::into(self.adcs[1]) / afe1.as_multiplier();

        Telemetry {
            cpu_temp,
            adcs: [in0_volts, in1_volts],
            dacs: [self.dacs[0].into(), self.dacs[1].into()],
            digital_inputs: self.digital_inputs,
        }
    }
}

impl<T: Serialize> TelemetryClient<T> {
    /// Construct a new telemetry client.
    ///
    /// # Args
    /// * `mqtt` - The MQTT client
    /// * `prefix` - The device prefix to use for MQTT telemetry reporting.
    ///
    /// # Returns
    /// A new telemetry client.
    pub fn new(
        mqtt: minimq::Minimq<
            'static,
            NetworkReference,
            SystemTimer,
            minimq::broker::NamedBroker<NetworkReference>,
        >,
        prefix: &str,
    ) -> Self {
        let mut telemetry_topic: String<128> = String::from(prefix);
        telemetry_topic.push_str("/telemetry").unwrap();

        Self {
            mqtt,
            telemetry_topic,
            _telemetry: core::marker::PhantomData,
        }
    }

    /// Publish telemetry over MQTT
    ///
    /// # Note
    /// Telemetry is reported in a "best-effort" fashion. Failure to transmit telemetry will cause
    /// it to be silently dropped.
    ///
    /// # Args
    /// * `telemetry` - The telemetry to report
    pub fn publish(&mut self, telemetry: &T) {
        let telemetry: Vec<u8, 512> =
            serde_json_core::to_vec(telemetry).unwrap();
        self.mqtt
            .client()
            .publish(
                minimq::Publication::<&[u8]>::new(&telemetry)
                    .topic(&self.telemetry_topic)
                    .finish()
                    .unwrap(),
            )
            .map_err(|e| log::error!("Telemetry publishing error: {:?}", e))
            .ok();
    }

    /// Update the telemetry client
    ///
    /// # Note
    /// This function is provided to force the underlying MQTT state machine to process incoming
    /// and outgoing messages. Without this, the client will never connect to the broker. This
    /// should be called regularly.
    pub fn update(&mut self) {
        match self.mqtt.poll(|_client, _topic, _message, _properties| {}) {
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::TcpConnectionFailure(
                    smoltcp_nal::smoltcp::socket::tcp::ConnectError::Unaddressable
                ),
            )) => {}

            Err(error) => log::info!("Unexpected error: {:?}", error),
            _ => {}
        }
    }
}
