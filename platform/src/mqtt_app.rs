use core::fmt::Write;
use heapless::String;
use miniconf::Tree;
use smoltcp_nal::smoltcp::wire::EthernetAddress;

/// Settings that are used for configuring the network interface to Stabilizer.
#[derive(Clone, Debug, Tree)]
pub struct NetSettings {
    /// The broker domain name (or IP address) to use for MQTT connections.
    pub broker: String<255>,

    /// The MQTT ID to use upon connection with a broker.
    pub id: String<23>,

    /// An optional static IP address to use. An unspecified IP address (or malformed address) will
    /// use DHCP.
    pub ip: String<15>,
    #[tree(skip)]
    /// The MAC address of Stabilizer, which is used to reinitialize the ID to default settings.
    pub mac: EthernetAddress,
}

impl NetSettings {
    pub fn new(mac: EthernetAddress) -> Self {
        let mut id = String::new();
        write!(&mut id, "{mac}").unwrap();

        Self {
            broker: String::try_from("mqtt").unwrap(),
            ip: String::try_from("0.0.0.0").unwrap(),
            id,
            mac,
        }
    }
}

pub trait AppSettings {
    /// Construct the settings given known network settings.
    fn new(net: NetSettings) -> Self;

    /// Get the network settings from the application settings.
    fn net(&self) -> &NetSettings;
}
