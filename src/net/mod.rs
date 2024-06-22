//! Stabilizer network management module
//!
//! # Design
//! The stabilizer network architecture supports numerous layers to permit transmission of
//! telemetry (via MQTT), configuration of run-time settings (via MQTT + Miniconf), and live data
//! streaming over raw UDP/TCP sockets. This module encompasses the main processing routines
//! related to Stabilizer networking operations.
pub use heapless;
pub use miniconf;
pub use serde;

pub mod data_stream;
pub mod network_processor;
pub mod telemetry;

use crate::hardware::{
    metadata::ApplicationMetadata, EthernetPhy, NetworkManager, NetworkStack,
    SystemTimer,
};
use crate::settings::NetSettings;
use data_stream::{DataStream, FrameGenerator, StreamTarget};
use network_processor::NetworkProcessor;
use telemetry::TelemetryClient;

use core::fmt::Write;
use heapless::String;
use miniconf::JsonCoreSlash;
use miniconf_mqtt::minimq;

pub type NetworkReference =
    smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

pub struct MqttStorage {
    telemetry: [u8; 2048],
    settings: [u8; 1024],
}

impl Default for MqttStorage {
    fn default() -> Self {
        Self {
            telemetry: [0u8; 2048],
            settings: [0u8; 1024],
        }
    }
}

pub enum UpdateState {
    NoChange,
    Updated,
}

pub enum NetworkState {
    SettingsChanged,
    Updated,
    NoChange,
}

/// A structure of Stabilizer's default network users.
pub struct NetworkUsers<S, const Y: usize>
where
    for<'de> S: Default + JsonCoreSlash<'de, Y> + Clone,
{
    pub miniconf: miniconf_mqtt::MqttClient<
        'static,
        S,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
        Y,
    >,
    pub processor: NetworkProcessor,
    stream: DataStream,
    generator: Option<FrameGenerator>,
    pub telemetry: TelemetryClient,
}

impl<S, const Y: usize> NetworkUsers<S, Y>
where
    for<'de> S: Default + JsonCoreSlash<'de, Y> + Clone,
{
    /// Construct Stabilizer's default network users.
    ///
    /// # Args
    /// * `stack` - The network stack that will be used to share with all network users.
    /// * `phy` - The ethernet PHY connecting the network.
    /// * `clock` - A `SystemTimer` implementing `Clock`.
    /// * `app` - The name of the application.
    /// * `net_settings` - The network-specific settings to use for the application.
    /// * `metadata` - The application metadata
    ///
    /// # Returns
    /// A new struct of network users.
    pub fn new(
        stack: NetworkStack,
        phy: EthernetPhy,
        clock: SystemTimer,
        app: &str,
        net_settings: &NetSettings,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        let stack_manager =
            cortex_m::singleton!(: NetworkManager = NetworkManager::new(stack))
                .unwrap();

        let processor =
            NetworkProcessor::new(stack_manager.acquire_stack(), phy);

        let prefix = get_device_prefix(app, &net_settings.id);

        let store =
            cortex_m::singleton!(: MqttStorage = MqttStorage::default())
                .unwrap();

        let named_broker = minimq::broker::NamedBroker::new(
            &net_settings.broker,
            stack_manager.acquire_stack(),
        )
        .unwrap();
        let settings = miniconf_mqtt::MqttClient::new(
            stack_manager.acquire_stack(),
            &prefix,
            clock,
            minimq::ConfigBuilder::new(named_broker, &mut store.settings)
                .client_id(&get_client_id(&net_settings.id, "settings"))
                .unwrap(),
        )
        .unwrap();

        let named_broker = minimq::broker::NamedBroker::new(
            &net_settings.broker,
            stack_manager.acquire_stack(),
        )
        .unwrap();
        let mqtt = minimq::Minimq::new(
            stack_manager.acquire_stack(),
            clock,
            minimq::ConfigBuilder::new(named_broker, &mut store.telemetry)
                // The telemetry client doesn't receive any messages except MQTT control packets.
                // As such, we don't need much of the buffer for RX.
                .rx_buffer(minimq::config::BufferConfig::Maximum(100))
                .client_id(&get_client_id(&net_settings.id, "tlm"))
                .unwrap(),
        );

        let telemetry = TelemetryClient::new(mqtt, &prefix, metadata);

        let (generator, stream) =
            data_stream::setup_streaming(stack_manager.acquire_stack());

        NetworkUsers {
            miniconf: settings,
            processor,
            telemetry,
            stream,
            generator: Some(generator),
        }
    }

    /// Enable live data streaming.
    ///
    /// # Args
    /// * `format` - A unique u8 code indicating the format of the data.
    pub fn configure_streaming(
        &mut self,
        format: impl Into<u8>,
    ) -> FrameGenerator {
        let mut generator = self.generator.take().unwrap();
        generator.configure(format);
        generator
    }

    /// Direct the stream to the provided remote target.
    ///
    /// # Args
    /// * `remote` - The destination for the streamed data.
    pub fn direct_stream(&mut self, remote: StreamTarget) {
        if self.generator.is_none() {
            self.stream.set_remote(remote);
        }
    }

    /// Update and process all of the network users state.
    ///
    /// # Returns
    /// An indication if any of the network users indicated a state change.
    /// The SettingsChanged option contains the path of the settings that changed.
    pub fn update(&mut self, settings: &mut S) -> NetworkState {
        // Update the MQTT clients.
        self.telemetry.update();

        // Update the data stream.
        if self.generator.is_none() {
            self.stream.process();
        }

        // Poll for incoming data.
        let poll_result = match self.processor.update() {
            UpdateState::NoChange => NetworkState::NoChange,
            UpdateState::Updated => NetworkState::Updated,
        };

        let res = self.miniconf.update(settings);
        match res {
            Ok(true) => NetworkState::SettingsChanged,
            _ => poll_result,
        }
    }
}

/// Get an MQTT client ID for a client.
///
/// # Args
/// * `id` - The base client ID
/// * `mode` - The operating mode of this client. (i.e. tlm, settings)
///
/// # Returns
/// A client ID that may be used for MQTT client identification.
fn get_client_id(id: &str, mode: &str) -> String<64> {
    let mut identifier = String::new();
    write!(&mut identifier, "{id}-{mode}").unwrap();
    identifier
}

/// Get the MQTT prefix of a device.
///
/// # Args
/// * `app` - The name of the application that is executing.
/// * `id` - The MQTT ID of the device.
///
/// # Returns
/// The MQTT prefix used for this device.
pub fn get_device_prefix(app: &str, id: &str) -> String<128> {
    // Note(unwrap): The mac address + binary name must be short enough to fit into this string. If
    // they are defined too long, this will panic and the device will fail to boot.
    let mut prefix: String<128> = String::new();
    write!(&mut prefix, "dt/sinara/{app}/{id}").unwrap();

    prefix
}
