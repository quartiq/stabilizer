//! Stabilizer network management module
//!
//! # Design
//! The stabilizer network architecture supports numerous layers to permit transmission of
//! telemetry (via MQTT), configuration of run-time settings (via MQTT + Miniconf), and data
//! streaming over raw UDP/TCP sockets. This module encompasses the main processing routines
//! related to Stabilizer networking operations.
pub use heapless;
pub use miniconf;
pub use serde;

use crate::hardware::{adc::AdcCode, afe::Gain, dac::DacCode, SystemTimer};
use crate::hardware::{EthernetPhy, NetworkManager, NetworkStack};
use platform::{ApplicationMetadata, NetSettings, TelemetryClient};
use serde::Serialize;
use stream::{DataStream, FrameGenerator, StreamTarget};

use core::fmt::Write;
use heapless::String;
use miniconf::{TreeDeserializeOwned, TreeSchema, TreeSerialize};
use miniconf_mqtt::minimq;

pub type NetworkReference =
    smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

/// The telemetry buffer is used for storing sample values during execution.
///
/// # Note
/// These values can be converted to SI units immediately before reporting to save processing time.
/// This allows for the DSP process to continually update the values without incurring significant
/// run-time overhead during conversion to SI units.
#[derive(Clone, Default)]
pub struct TelemetryBuffer {
    /// The latest input sample on ADC0/ADC1.
    pub adcs: [AdcCode; 2],
    /// The latest output code on DAC0/DAC1.
    pub dacs: [DacCode; 2],
    /// The latest digital input states during processing.
    pub digital_inputs: [bool; 2],
}

/// The telemetry structure is data that is ultimately reported as telemetry over MQTT.
///
/// # Note
/// This structure should be generated on-demand by the buffer when required to minimize conversion
/// overhead.
#[derive(Serialize)]
pub struct Telemetry {
    /// Most recent input voltage measurement.
    pub adcs: [f32; 2],

    /// Most recent output voltage.
    pub dacs: [f32; 2],

    /// Most recent digital input assertion state.
    pub digital_inputs: [bool; 2],

    /// The CPU temperature in degrees Celsius.
    pub cpu_temp: f32,
}

impl TelemetryBuffer {
    /// Convert the telemetry buffer to finalized, SI-unit telemetry for reporting.
    ///
    /// # Args
    /// * `afe0` - The current AFE configuration for channel 0.
    /// * `afe1` - The current AFE configuration for channel 1.
    /// * `cpu_temp` - The current CPU temperature.
    ///
    /// # Returns
    /// The finalized telemetry structure that can be serialized and reported.
    pub fn finalize(self, afe0: Gain, afe1: Gain, cpu_temp: f32) -> Telemetry {
        let in0_volts = f32::from(self.adcs[0]) / afe0.gain();
        let in1_volts = f32::from(self.adcs[1]) / afe1.gain();

        Telemetry {
            cpu_temp,
            adcs: [in0_volts, in1_volts],
            dacs: [self.dacs[0].into(), self.dacs[1].into()],
            digital_inputs: self.digital_inputs,
        }
    }
}

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

const MAX_DEPTH: usize = 10;

/// A structure of Stabilizer's default network users.
pub struct NetworkUsers<S>
where
    S: Default + TreeDeserializeOwned + TreeSerialize + Clone,
{
    pub miniconf: miniconf_mqtt::MqttClient<
        'static,
        S,
        NetworkReference,
        SystemTimer,
        minimq::broker::NamedBroker<NetworkReference>,
        MAX_DEPTH,
    >,
    pub processor: NetworkProcessor,
    stream: DataStream<NetworkReference>,
    generator: Option<FrameGenerator>,
    pub telemetry: TelemetryClient<SystemTimer, NetworkReference>,
}

impl<S> NetworkUsers<S>
where
    S: Default + TreeDeserializeOwned + TreeSerialize + TreeSchema + Clone,
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

        let prefix = cortex_m::singleton!(: String<128> = get_device_prefix(app, &net_settings.id)).unwrap();

        let store =
            cortex_m::singleton!(: MqttStorage = MqttStorage::default())
                .unwrap();

        let named_broker = minimq::broker::NamedBroker::new(
            &net_settings.broker,
            stack_manager.acquire_stack(),
        )
        .unwrap();
        let miniconf = miniconf_mqtt::MqttClient::<_, _, _, _, MAX_DEPTH>::new(
            stack_manager.acquire_stack(),
            prefix.as_str(),
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

        let telemetry = TelemetryClient::new(mqtt, prefix, metadata);

        let (generator, stream) =
            stream::setup_streaming(stack_manager.acquire_stack());

        NetworkUsers {
            miniconf,
            processor,
            telemetry,
            stream,
            generator: Some(generator),
        }
    }

    /// Enable data streaming.
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

// Task to process network hardware.
//
// # Design
// The network processir is a small taks to regularly process incoming data over ethernet, handle
// the ethernet PHY state, and reset the network as appropriate.

/// Processor for managing network hardware.
pub struct NetworkProcessor {
    pub stack: NetworkReference,
    phy: EthernetPhy,
    network_was_reset: bool,
}

impl NetworkProcessor {
    /// Construct a new network processor.
    ///
    /// # Args
    /// * `stack` - A reference to the shared network stack
    /// * `phy` - The ethernet PHY used for the network.
    ///
    /// # Returns
    /// The newly constructed processor.
    pub fn new(stack: NetworkReference, phy: EthernetPhy) -> Self {
        Self {
            stack,
            phy,
            network_was_reset: false,
        }
    }

    /// Handle ethernet link connection status.
    ///
    /// # Note
    /// This may take non-trivial amounts of time to communicate with the PHY. As such, this should
    /// only be called as often as necessary (e.g. once per second or so).
    pub fn handle_link(&mut self) {
        // If the PHY indicates there's no more ethernet link, reset the DHCP server in the network
        // stack.
        let link_up = self.phy.poll_link();
        match (link_up, self.network_was_reset) {
            (true, true) => {
                log::warn!("Network link UP");
                self.network_was_reset = false;
            }
            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            (false, false) => {
                log::warn!("Network link DOWN");
                self.network_was_reset = true;
                self.stack.lock(|stack| stack.handle_link_reset());
            }
            _ => {}
        };
    }

    /// Process and update the state of the network.
    ///
    /// # Note
    /// This function should be called regularly before other network tasks to update the state of
    /// all relevant network sockets.
    ///
    /// # Returns
    /// An update state corresponding with any changes in the underlying network.
    pub fn update(&mut self) -> UpdateState {
        match self.stack.lock(|stack| stack.poll()) {
            Ok(true) => UpdateState::Updated,
            Ok(false) => UpdateState::NoChange,
            Err(_) => UpdateState::Updated,
        }
    }
}
