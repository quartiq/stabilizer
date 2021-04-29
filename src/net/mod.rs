use crate::hardware::{
    design_parameters::MQTT_BROKER, CycleCounter, EthernetPhy, NetworkStack,
};

use core::fmt::Write;

use heapless::{consts, String};
use miniconf::minimq;

/// Potential actions for firmware to take.
pub enum Action {
    /// Indicates that firmware can sleep for the next event.
    Sleep,

    /// Indicates that settings have updated and firmware needs to propogate changes.
    UpdateSettings,
}

/// MQTT settings interface.
pub struct MiniconfInterface<S>
where
    S: miniconf::Miniconf + Default,
{
    pub mqtt: miniconf::MqttInterface<S, NetworkStack, minimq::consts::U256>,
    clock: CycleCounter,
    phy: EthernetPhy,
    network_was_reset: bool,
}

impl<S> MiniconfInterface<S>
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

            miniconf::MqttInterface::new(mqtt_client, prefix, S::default())
                .unwrap()
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

/// Get the MQTT prefix of a device.
///
/// # Args
/// * `app` - The name of the application that is executing.
/// * `mac` - The ethernet MAC address of the device.
///
/// # Returns
/// The MQTT prefix used for this device.
pub fn get_device_prefix(
    app: &str,
    mac: smoltcp_nal::smoltcp::wire::EthernetAddress,
) -> String<consts::U128> {
    let mac_string = {
        let mut mac_string: String<consts::U32> = String::new();
        let mac = mac.as_bytes();

        // Note(unwrap): 32-bytes is guaranteed to be valid for any mac address, as the address has
        // a fixed length.
        write!(
            &mut mac_string,
            "{:02x}-{:02x}-{:02x}-{:02x}-{:02x}-{:02x}",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]
        )
        .unwrap();

        mac_string
    };

    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut prefix: String<consts::U128> = String::new();
    write!(&mut prefix, "dt/sinara/{}/{}", app, mac_string).unwrap();

    prefix
}
