use crate::hardware::{
    design_parameters::MQTT_BROKER, CycleCounter, EthernetPhy, NetworkStack,
};

use core::{cell::RefCell, fmt::Write};

use heapless::{consts, String};
use serde::Serialize;

use super::{Action, RouteResult, SettingsResponse};

/// MQTT settings interface.
pub struct MqttInterface<S>
where
    S: miniconf::Miniconf + Default + Clone,
{
    telemetry_topic: String<consts::U128>,
    default_response_topic: String<consts::U128>,
    mqtt: RefCell<minimq::MqttClient<minimq::consts::U256, NetworkStack>>,
    settings: RefCell<S>,
    clock: CycleCounter,
    phy: EthernetPhy,
    network_was_reset: bool,
    subscribed: bool,
    id: String<consts::U32>,
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
        let mqtt_client =
            minimq::MqttClient::new(MQTT_BROKER.into(), client_id, stack)
                .unwrap();

        let mut telemetry_topic: String<consts::U128> = String::new();
        write!(&mut telemetry_topic, "{}/telemetry", prefix).unwrap();

        let mut response_topic: String<consts::U128> = String::new();
        write!(&mut response_topic, "{}/log", prefix).unwrap();

        Self {
            mqtt: RefCell::new(mqtt_client),
            settings: RefCell::new(S::default()),
            id: String::from(prefix),
            clock,
            phy,
            telemetry_topic,
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
        let sleep = match self
            .mqtt
            .borrow_mut()
            .network_stack
            .poll(self.clock.current_ms())
        {
            Ok(updated) => !updated,
            Err(err) => {
                log::info!("Network error: {:?}", err);
                false
            }
        };

        // If the PHY indicates there's no more ethernet link, reset the DHCP server in the network
        // stack.
        if self.phy.poll_link() == false {
            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            if !self.network_was_reset {
                self.network_was_reset = true;
                self.mqtt.borrow_mut().network_stack.handle_link_reset();
            }
        } else {
            self.network_was_reset = false;
        }

        // If we're no longer subscribed to the settings topic, but we are connected to the broker,
        // resubscribe.
        if !self.subscribed && self.mqtt.borrow_mut().is_connected().unwrap() {
            let mut settings_topic: String<consts::U128> = String::new();
            write!(&mut settings_topic, "{}/settings/#", self.id.as_str())
                .unwrap();

            self.mqtt
                .borrow_mut()
                .subscribe(&settings_topic, &[])
                .unwrap();
            self.subscribed = true;
        }

        // Handle any MQTT traffic.
        let mut update = false;
        match self.mqtt.borrow_mut().poll(
            |client, topic, message, properties| {
                let (response, settings_update) =
                    self.route_message(topic, message, properties);
                client
                    .publish(
                        response.response_topic,
                        &response.message,
                        minimq::QoS::AtMostOnce,
                        &response.properties,
                    )
                    .ok();
                update = settings_update;
            },
        ) {
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

    fn route_message<'a, 'me: 'a>(
        &'me self,
        topic: &str,
        message: &[u8],
        properties: &[minimq::Property<'a>],
    ) -> (RouteResult<'a>, bool) {
        let mut response =
            RouteResult::new(properties, &self.default_response_topic);
        let mut update = false;

        if let Some(path) = topic.strip_prefix(self.id.as_str()) {
            let mut parts = path[1..].split('/');
            match parts.next() {
                Some("settings") => {
                    let result = self
                        .settings
                        .borrow_mut()
                        .string_set(parts.peekable(), message);
                    update = result.is_ok();
                    response.set_message(SettingsResponse::new(result, topic));
                }
                Some(_) => response.set_message(SettingsResponse::custom(
                    "Unknown topic",
                    255,
                )),
                _ => response
                    .set_message(SettingsResponse::custom("No topic", 254)),
            }
        } else {
            response
                .set_message(SettingsResponse::custom("Invalid prefix", 253));
        }

        (response, update)
    }

    pub fn publish_telemetry(&mut self, telemetry: &impl Serialize) {
        let telemetry =
            miniconf::serde_json_core::to_string::<consts::U256, _>(telemetry)
                .unwrap();
        self.mqtt
            .borrow_mut()
            .publish(
                &self.telemetry_topic,
                telemetry.as_bytes(),
                minimq::QoS::AtMostOnce,
                &[],
            )
            .ok();
    }

    pub fn settings(&self) -> S {
        self.settings.borrow().clone()
    }
}
