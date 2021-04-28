use heapless::{consts, String, Vec};
use serde::Serialize;

use core::fmt::Write;

pub struct RouteResult<'a> {
    pub response_topic: &'a str,
    pub message: Vec<u8, consts::U128>,
    pub properties: Vec<minimq::Property<'a>, consts::U1>,
}

#[derive(Serialize)]
pub struct SettingsResponse {
    code: u8,
    msg: String<heapless::consts::U64>,
}

impl<'a> RouteResult<'a> {
    pub fn new<'b: 'a>(
        properties: &[minimq::Property<'a>],
        default_response: &'b str,
    ) -> Self {
        // Extract the MQTT response topic.
        let response_topic = properties
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

        RouteResult {
            response_topic,
            message: Vec::new(),
            properties: correlation_data,
        }
    }

    pub fn set_message(&mut self, response: impl Serialize) {
        self.message = miniconf::serde_json_core::to_vec(&response).unwrap();
    }
}

impl SettingsResponse {
    pub fn new(result: Result<(), miniconf::Error>, path: &str) -> Self {
        match result {
            Ok(_) => {
                let mut msg: String<consts::U64> = String::new();
                if write!(&mut msg, "{} updated", path).is_err() {
                    msg = String::from("Latest update succeeded");
                }

                Self { msg, code: 0 }
            }
            Err(error) => {
                let mut msg: String<consts::U64> = String::new();
                if write!(&mut msg, "{} update failed: {:?}", path, error)
                    .is_err()
                {
                    if write!(&mut msg, "Latest update failed: {:?}", error)
                        .is_err()
                    {
                        msg = String::from("Latest update failed");
                    }
                }
                Self { msg, code: 5 }
            }
        }
    }

    pub fn custom(msg: &str, code: u8) -> Self {
        Self {
            code,
            msg: String::from(msg),
        }
    }
}
