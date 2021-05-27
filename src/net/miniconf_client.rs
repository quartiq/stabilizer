///! Stabilizer Run-time Settings Client
///!
///! # Design
///! Stabilizer allows for settings to be configured at run-time via MQTT using miniconf.
///! Settings are written in serialized JSON form to the settings path associated with the setting.
///!
///! # Limitations
///! The MQTT client logs failures to subscribe to the settings topic, but does not re-attempt to
///connect to it when errors occur.
///!
///! Respones to settings updates are sent without quality-of-service guarantees, so there's no
///! guarantee that the requestee will be informed that settings have been applied.
use heapless::{consts, String};

use super::{MqttMessage, NetworkReference, SettingsResponse, UpdateState};
use crate::hardware::design_parameters::MQTT_BROKER;

/// MQTT settings interface.
pub struct MiniconfClient<S>
where
    S: miniconf::Miniconf + Default + Clone,
{
    default_response_topic: String<consts::U128>,
    mqtt: minimq::Minimq<NetworkReference, 256>,
    settings: S,
    subscribed: bool,
    settings_prefix: String<consts::U64>,
}

impl<S> MiniconfClient<S>
where
    S: miniconf::Miniconf + Default + Clone,
{
    /// Construct a new MQTT settings interface.
    ///
    /// # Args
    /// * `stack` - The network stack to use for communication.
    /// * `client_id` - The ID of the MQTT client. May be an empty string for auto-assigning.
    /// * `prefix` - The MQTT device prefix to use for this device.
    pub fn new(stack: NetworkReference, client_id: &str, prefix: &str) -> Self {
        let mqtt =
            minimq::Minimq::new(MQTT_BROKER.into(), client_id, stack).unwrap();

        let mut response_topic: String<consts::U128> = String::from(prefix);
        response_topic.push_str("/log").unwrap();

        let mut settings_prefix: String<consts::U64> = String::from(prefix);
        settings_prefix.push_str("/settings").unwrap();

        Self {
            mqtt,
            settings: S::default(),
            settings_prefix,
            default_response_topic: response_topic,
            subscribed: false,
        }
    }

    /// Update the MQTT interface and service the network
    ///
    /// # Returns
    /// An option containing an action that should be completed as a result of network servicing.
    pub fn update(&mut self) -> UpdateState {
        let mqtt_connected = match self.mqtt.client.is_connected() {
            Ok(connected) => connected,
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::NoIpAddress,
            )) => false,
            Err(minimq::Error::Network(error)) => {
                log::info!("Unexpected network error: {:?}", error);
                false
            }
            Err(error) => {
                log::warn!("Unexpected MQTT error: {:?}", error);
                false
            }
        };

        // If we're no longer subscribed to the settings topic, but we are connected to the broker,
        // resubscribe.
        if !self.subscribed && mqtt_connected {
            // Note(unwrap): We construct a string with two more characters than the prefix
            // strucutre, so we are guaranteed to have space for storage.
            let mut settings_topic: String<consts::U66> =
                String::from(self.settings_prefix.as_str());
            settings_topic.push_str("/#").unwrap();

            // We do not currently handle or process potential subscription failures. Instead, this
            // failure will be logged through the stabilizer logging interface.
            self.mqtt.client.subscribe(&settings_topic, &[]).unwrap();
            self.subscribed = true;
        }

        // Handle any MQTT traffic.
        let settings = &mut self.settings;
        let mqtt = &mut self.mqtt;
        let prefix = self.settings_prefix.as_str();
        let default_response_topic = self.default_response_topic.as_str();

        let mut update = false;
        match mqtt.poll(|client, topic, message, properties| {
            let path = match topic.strip_prefix(prefix) {
                // For paths, we do not want to include the leading slash.
                Some(path) => {
                    if path.len() > 0 {
                        &path[1..]
                    } else {
                        path
                    }
                }
                None => {
                    info!("Unexpected MQTT topic: {}", topic);
                    return;
                }
            };

            let message: SettingsResponse = settings
                .string_set(path.split('/').peekable(), message)
                .and_then(|_| {
                    update = true;
                    Ok(())
                })
                .into();

            let response =
                MqttMessage::new(properties, default_response_topic, &message);

            client
                .publish(
                    response.topic,
                    &response.message,
                    // TODO: When Minimq supports more QoS levels, this should be increased to
                    // ensure that the client has received it at least once.
                    minimq::QoS::AtMostOnce,
                    &response.properties,
                )
                .ok();
        }) {
            // If settings updated,
            Ok(_) if update => UpdateState::Updated,
            Ok(_) => UpdateState::NoChange,
            Err(minimq::Error::SessionReset) => {
                self.subscribed = false;
                UpdateState::NoChange
            }
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::NoIpAddress,
            )) => UpdateState::NoChange,

            Err(error) => {
                log::info!("Unexpected error: {:?}", error);
                UpdateState::NoChange
            }
        }
    }

    /// Get the current settings from miniconf.
    pub fn settings(&self) -> &S {
        &self.settings
    }
}
