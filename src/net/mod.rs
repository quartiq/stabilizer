use crate::hardware::{
    design_parameters::MQTT_BROKER, CycleCounter, EthernetPhy, NetworkStack,
};

use miniconf::{minimq, MqttInterface};

/// Potential actions for firmware to take.
pub enum Action {
    /// Indicates that firmware can sleep for the next event.
    Sleep,

    /// Indicates that settings have updated and firmware needs to propogate changes.
    UpdateSettings,
}

/// MQTT settings interface.
pub struct MqttSettings<S>
where
    S: miniconf::Miniconf + Default,
{
    pub mqtt: MqttInterface<S, NetworkStack, minimq::consts::U256>,
    clock: CycleCounter,
    phy: EthernetPhy,
    network_was_reset: bool,
}

impl<S> MqttSettings<S>
where
    S: miniconf::Miniconf + Default,
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
        let mqtt = {
            let mqtt_client = {
                minimq::MqttClient::new(MQTT_BROKER.into(), client_id, stack)
                    .unwrap()
            };

            MqttInterface::new(mqtt_client, prefix, S::default()).unwrap()
        };

        Self {
            mqtt,
            clock,
            phy,
            network_was_reset: false,
        }
    }

    /// Update the MQTT interface and service the network
    ///
    /// # Returns
    /// An option containing an action that should be completed as a result of network servicing.
    pub fn update(&mut self) -> Option<Action> {
        let now = self.clock.current_ms();

        // First, service the network stack to process and inbound and outbound traffic.
        let sleep = match self.mqtt.network_stack().poll(now) {
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
                self.mqtt.network_stack().handle_link_reset();
            }
        } else {
            self.network_was_reset = false;
        }

        // Finally, service the MQTT interface and handle any necessary messages.
        match self.mqtt.update() {
            Ok(true) => Some(Action::UpdateSettings),
            Ok(false) if sleep => Some(Action::Sleep),
            Ok(_) => None,

            Err(miniconf::MqttError::Network(
                smoltcp_nal::NetworkError::NoIpAddress,
            )) => None,

            Err(error) => {
                log::info!("Unexpected error: {:?}", error);
                None
            }
        }
    }
}
