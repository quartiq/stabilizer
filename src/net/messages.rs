use heapless::{consts, String, Vec};
use serde::Serialize;

use core::fmt::Write;

#[derive(Debug, Copy, Clone)]
pub enum SettingsResponseCode {
    NoError = 0,
    NoTopic = 1,
    InvalidPrefix = 2,
    UnknownTopic = 3,
    UpdateFailure = 4,
}

/// Represents a generic MQTT message.
pub struct MqttMessage<'a> {
    pub topic: &'a str,
    pub message: Vec<u8, consts::U128>,
    pub properties: Vec<minimq::Property<'a>, consts::U1>,
}

/// The payload of the MQTT response message to a settings update request.
#[derive(Serialize)]
pub struct SettingsResponse {
    code: u8,
    msg: String<heapless::consts::U64>,
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
        let mut correlation_data: Vec<minimq::Property<'a>, consts::U1> =
            Vec::new();
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

impl SettingsResponse {
    /// Construct a settings response upon successful settings update.
    ///
    /// # Args
    /// * `path` - The path of the setting that was updated.
    pub fn update_success(path: &str) -> Self {
        let mut msg: String<consts::U64> = String::new();
        if write!(&mut msg, "{} updated", path).is_err() {
            msg = String::from("Latest update succeeded");
        }

        Self {
            msg,
            code: SettingsResponseCode::NoError as u8,
        }
    }

    /// Construct a response when a settings update failed.
    ///
    /// # Args
    /// * `path` - The settings path that configuration failed for.
    /// * `err` - The settings update error that occurred.
    pub fn update_failure(path: &str, err: miniconf::Error) -> Self {
        let mut msg: String<consts::U64> = String::new();
        if write!(&mut msg, "{} update failed: {:?}", path, err).is_err() {
            if write!(&mut msg, "Latest update failed: {:?}", err).is_err() {
                msg = String::from("Latest update failed");
            }
        }

        Self {
            msg,
            code: SettingsResponseCode::UpdateFailure as u8,
        }
    }

    /// Construct a response from a custom response code.
    ///
    /// # Args
    /// * `code` - The response code to provide.
    pub fn code(code: SettingsResponseCode) -> Self {
        let mut msg: String<consts::U64> = String::new();

        // Note(unwrap): All code debug names shall fit in the 64 byte string.
        write!(&mut msg, "{:?}", code).unwrap();

        Self {
            code: code as u8,
            msg,
        }
    }
}
