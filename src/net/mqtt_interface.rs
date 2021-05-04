use crate::hardware::{
    design_parameters::MQTT_BROKER, CycleCounter, EthernetPhy, NetworkStack,
};

use heapless::{consts, String};

use super::{Action, MqttMessage, SettingsResponse};

/// MQTT settings interface.
pub struct MqttInterface<S>
where
    S: miniconf::Miniconf + Default + Clone,
{
    default_response_topic: String<consts::U128>,
    mqtt: minimq::MqttClient<minimq::consts::U256, NetworkStack>,
    settings: S,
    clock: CycleCounter,
    phy: EthernetPhy,
    network_was_reset: bool,
    subscribed: bool,
    settings_prefix: String<consts::U64>,
}

impl<S> MqttInterface<S>
where
    S: miniconf::Miniconf + Default + Clone,
{
    /// Construct a new MQTT settings interface.
    ///
    /// # Args
    /// * `stack` - The network stack to use for communication.
    /// * `client_id` - The ID of the MQTT client. May be an empty string for auto-assigning.
    /// * `prefix` - The MQTT device prefix to use for this device.
    /// * `phy` - The PHY driver for querying the link state.
    /// * `clock` - The clock to utilize for querying the current system time.
    pub fn new(
        stack: NetworkStack,
        client_id: &str,
        prefix: &str,
        phy: EthernetPhy,
        clock: CycleCounter,
    ) -> Self {
        let mqtt =
            minimq::MqttClient::new(MQTT_BROKER.into(), client_id, stack)
                .unwrap();

        let mut response_topic: String<consts::U128> = String::from(prefix);
        response_topic.push_str("/log").unwrap();

        let mut settings_prefix: String<consts::U64> = String::from(prefix);
        settings_prefix.push_str("/settings").unwrap();

        Self {
            mqtt,
            settings: S::default(),
            settings_prefix,
            clock,
            phy,
            default_response_topic: response_topic,
            network_was_reset: false,
            subscribed: false,
        }
    }

    /// Update the MQTT interface and service the network
    ///
    /// # Returns
    /// An option containing an action that should be completed as a result of network servicing.
    pub fn update(&mut self) -> Option<Action> {
        // First, service the network stack to process any inbound and outbound traffic.
        let sleep = match self.mqtt.network_stack.poll(self.clock.current_ms())
        {
            Ok(updated) => !updated,
            Err(err) => {
                log::info!("Network error: {:?}", err);
                false
            }
        };

        // If the PHY indicates there's no more ethernet link, reset the DHCP server in the network
        // stack.
        match self.phy.poll_link() {
            true => self.network_was_reset = false,

            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            false if !self.network_was_reset => {
                self.network_was_reset = true;
                self.mqtt.network_stack.handle_link_reset();
            }
            _ => {},
        };

        let mqtt_connected = match self.mqtt.is_connected() {
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

            self.mqtt.subscribe(&settings_topic, &[]).unwrap();
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
            Ok(_) => {
                if update {
                    Some(Action::UpdateSettings)
                } else if sleep {
                    Some(Action::Sleep)
                } else {
                    None
                }
            }
            Err(minimq::Error::Disconnected) => {
                self.subscribed = false;
                None
            }
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::NoIpAddress,
            )) => None,

            Err(error) => {
                log::info!("Unexpected error: {:?}", error);
                None
            }
        }
    }

    pub fn settings(&self) -> &S {
        &self.settings
    }
}
