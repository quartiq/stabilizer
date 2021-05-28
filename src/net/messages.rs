use heapless::{String, Vec};
use serde::Serialize;

use core::fmt::Write;

#[derive(Debug, Copy, Clone)]
pub enum SettingsResponseCode {
    NoError = 0,
    MiniconfError = 1,
}

/// Represents a generic MQTT message.
pub struct MqttMessage<'a> {
    pub topic: &'a str,
    pub message: Vec<u8, 128>,
    pub properties: Vec<minimq::Property<'a>, 1>,
}

/// The payload of the MQTT response message to a settings update request.
#[derive(Serialize)]
pub struct SettingsResponse {
    code: u8,
    msg: String<64>,
}

impl<'a> MqttMessage<'a> {
    /// Construct a new MQTT message from an incoming message.
    ///
    /// # Args
    /// * `properties` - A list of properties associated with the inbound message.
    /// * `default_response` - The default response topic for the message
    /// * `msg` - The response associated with the message. Must fit within 128 bytes.
    pub fn new<'b: 'a>(
        properties: &[minimq::Property<'a>],
        default_response: &'b str,
        msg: &impl Serialize,
    ) -> Self {
        // Extract the MQTT response topic.
        let topic = properties
            .iter()
            .find_map(|prop| {
                if let minimq::Property::ResponseTopic(topic) = prop {
                    Some(topic)
                } else {
                    None
                }
            })
            .unwrap_or(&default_response);

        // Associate any provided correlation data with the response.
        let mut correlation_data: Vec<minimq::Property<'a>, 1> = Vec::new();
        if let Some(data) = properties
            .iter()
            .find(|prop| matches!(prop, minimq::Property::CorrelationData(_)))
        {
            // Note(unwrap): Unwrap can not fail, as we only ever push one value.
            correlation_data.push(*data).unwrap();
        }

        Self {
            topic,
            // Note(unwrap): All SettingsResponse objects are guaranteed to fit in the vector.
            message: miniconf::serde_json_core::to_vec(msg).unwrap(),
            properties: correlation_data,
        }
    }
}

impl From<Result<(), miniconf::Error>> for SettingsResponse {
    fn from(result: Result<(), miniconf::Error>) -> Self {
        match result {
            Ok(_) => Self {
                msg: String::from("OK"),
                code: SettingsResponseCode::NoError as u8,
            },

            Err(error) => {
                let mut msg = String::new();
                if write!(&mut msg, "{:?}", error).is_err() {
                    msg = String::from("Miniconf Error");
                }

                Self {
                    code: SettingsResponseCode::MiniconfError as u8,
                    msg,
                }
            }
        }
    }
}
