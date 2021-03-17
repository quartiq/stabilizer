use crate::hardware::{
    design_parameters::MQTT_BROKER, CycleCounter, EthernetPhy, NetworkStack,
};

use miniconf::{minimq, MqttInterface};

pub enum Action {
    Sleep,
    UpdateSettings,
}

pub struct MqttSettings<S>
where
    S: miniconf::Miniconf + Default,
{
    pub mqtt_interface: MqttInterface<S, NetworkStack, minimq::consts::U256>,
    clock: CycleCounter,
    phy: EthernetPhy,
    network_was_reset: bool,
}

impl<S> MqttSettings<S>
where
    S: miniconf::Miniconf + Default,
{
    pub fn new(
        stack: NetworkStack,
        client_id: &str,
        prefix: &str,
        phy: EthernetPhy,
        clock: CycleCounter,
    ) -> Self {
        let mqtt_interface = {
            let mqtt_client = {
                minimq::MqttClient::new(MQTT_BROKER.into(), client_id, stack)
                    .unwrap()
            };

            MqttInterface::new(mqtt_client, prefix, S::default()).unwrap()
        };

        Self {
            mqtt_interface,
            clock,
            phy,
            network_was_reset: false,
        }
    }

    pub fn update(&mut self) -> Option<Action> {
        let now = self.clock.current_ms();

        let sleep = match self.mqtt_interface.network_stack().poll(now) {
            Ok(updated) => !updated,
            Err(err) => {
                log::info!("Network error: {:?}", err);
                false
            }
        };

        // If the PHY indicates there's no more ethernet link, reset the network stack and close all
        // sockets.
        if self.phy.poll_link() == false {
            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            if !self.network_was_reset {
                self.network_was_reset = true;
                self.mqtt_interface.network_stack().reset();
            }
        } else {
            self.network_was_reset = false;
        }

        match self.mqtt_interface.update() {
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
