use heapless::{consts, String};

use core::fmt::Write;

mod mqtt_interface;
mod router;
pub use mqtt_interface::MqttInterface;
use router::{RouteResult, SettingsResponse};

/// Potential actions for firmware to take.
pub enum Action {
    /// Indicates that firmware can sleep for the next event.
    Sleep,

    /// Indicates that settings have updated and firmware needs to propogate changes.
    UpdateSettings,
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
