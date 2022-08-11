use super::NetworkReference;
use crate::hardware::SystemTimer;
///! Thermostat MQTT client
///!
///! An MQTT client that handles Thermostat communication like the MQTT interlock.
///! This might also handle a temperature datastream to Thermostat at a later point.
///
/// Todo: make stabilizer search for Thermostats on the mqtt network
use heapless::String;
use minimq::embedded_nal::IpAddr;

pub struct InterlockError;

pub struct ThermostatClient {
    mqtt: minimq::Minimq<NetworkReference, SystemTimer, 512, 1>,
    interlock_topic: String<128>, // The prefix of Thermostat and the interlock topic
}

impl ThermostatClient {
    /// Construct a new Thermostat client.
    ///
    /// # Args
    /// * `stack` - A reference to the (shared) underlying network stack.
    /// * `clock` - A `SystemTimer` implementing `Clock`.
    /// * `client_id` - The MQTT client ID of the Thermostat client.
    /// * `prefix` - Thermostat MQTT prefix
    /// * `broker` - The IP address of the MQTT broker to use.
    ///
    /// # Returns
    /// A new Thermostat client.
    pub fn new(
        stack: NetworkReference,
        clock: SystemTimer,
        client_id: &str,
        prefix: &str,
        broker: IpAddr,
        interlock_timeout: fugit::MillisDurationU32,
    ) -> Self {
        let mqtt =
            minimq::Minimq::new(broker, client_id, stack, clock).unwrap();

        let mut interlock_topic: String<128> = String::from(prefix);
        interlock_topic.push_str("/interlock").unwrap();

        Self {
            mqtt,
            interlock_topic,
        }
    }

    // pub fn handle_interlock() -> Result<(), InterlockError> {}

    pub fn update(&mut self) {
        self.mqtt
            .poll(|_, topic, _, _| {
                let str: String<64> =
                    String::from(self.interlock_topic.as_str());
                match topic {
                    str => log::info!("interlock!"),
                    _ => log::info!("somethign else"),
                }
            })
            .unwrap();
    }

    // pub fn publish_setpoint()
}
