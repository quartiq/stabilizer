use heapless::{consts, String, Vec};
use serde::Serialize;

use super::NetworkReference;
use crate::hardware::design_parameters::MQTT_BROKER;
use minimq::QoS;

use crate::hardware::AfeGain;

#[derive(Copy, Clone)]
pub struct TelemetryBuffer {
    pub latest_samples: [i16; 2],
    pub latest_outputs: [u16; 2],
    pub digital_inputs: [bool; 2],
}

#[derive(Serialize)]
pub struct Telemetry {
    input_levels: [f32; 2],
    output_levels: [f32; 2],
    digital_inputs: [bool; 2],
}

impl Default for TelemetryBuffer {
    fn default() -> Self {
        Self {
            latest_samples: [0, 0],
            latest_outputs: [0, 0],
            digital_inputs: [false, false],
        }
    }
}

impl TelemetryBuffer {
    pub fn to_telemetry(self, afe0: AfeGain, afe1: AfeGain) -> Telemetry {
        let in0_volts =
            (self.latest_samples[0] as f32 / i16::MAX as f32) * 4.096 / 2.0
                * 5.0
                / afe0.to_multiplier() as f32;
        let in1_volts =
            (self.latest_samples[1] as f32 / i16::MAX as f32) * 4.096 / 2.0
                * 5.0
                / afe1.to_multiplier() as f32;

        let out0_volts = (10.24 * 2.0)
            * (self.latest_outputs[0] as f32 / (u16::MAX as f32))
            - 10.24;
        let out1_volts = (10.24 * 2.0)
            * (self.latest_outputs[1] as f32 / (u16::MAX as f32))
            - 10.24;

        Telemetry {
            input_levels: [in0_volts, in1_volts],
            output_levels: [out0_volts, out1_volts],
            digital_inputs: self.digital_inputs,
        }
    }
}

pub struct TelemetryClient<T: Serialize> {
    mqtt: minimq::MqttClient<minimq::consts::U256, NetworkReference>,
    telemetry_topic: String<consts::U128>,
    _telemetry: core::marker::PhantomData<T>,
}

impl<T: Serialize> TelemetryClient<T> {
    pub fn new(stack: NetworkReference, client_id: &str, prefix: &str) -> Self {
        let mqtt =
            minimq::MqttClient::new(MQTT_BROKER.into(), client_id, stack)
                .unwrap();

        let mut telemetry_topic: String<consts::U128> = String::from(prefix);
        telemetry_topic.push_str("/telemetry").unwrap();

        Self {
            mqtt,
            telemetry_topic,
            _telemetry: core::marker::PhantomData::default(),
        }
    }

    pub fn publish(&mut self, telemetry: &T) {
        let telemetry: Vec<u8, consts::U256> =
            serde_json_core::to_vec(telemetry).unwrap();
        self.mqtt
            .publish(&self.telemetry_topic, &telemetry, QoS::AtMostOnce, &[])
            .ok();
    }
}
