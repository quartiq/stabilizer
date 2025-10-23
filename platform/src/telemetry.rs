//! Stabilizer Telemetry Capabilities
//!
//! # Design
//! Telemetry is reported regularly using an MQTT client. All telemetry is reported in SI units
//! using standard JSON format.
//!
//! In order to report ADC/DAC codes generated during the DSP routines, a telemetry buffer is
//! employed to track the latest codes. Converting these codes to SI units would result in
//! repetitive and unnecessary calculations within the DSP routine, slowing it down and limiting
//! sampling frequency. Instead, the raw codes are stored and the telemetry is generated as
//! required immediately before transmission. This ensures that any slower computation required
//! for unit conversion can be off-loaded to lower priority tasks.
use crate::ApplicationMetadata;
use heapless::String;
use minimq::{
    PubError, Publication,
    embedded_nal::{Dns, TcpClientStack},
    embedded_time::Clock,
};
use serde::Serialize;
use smoltcp_nal::NetworkError;

/// Default metadata message if formatting errors occur.
const DEFAULT_METADATA: &str = "{\"message\":\"Truncated: See USB terminal\"}";

/// The telemetry client for reporting telemetry data over MQTT.
pub struct TelemetryClient<C: Clock, S: TcpClientStack> {
    mqtt: minimq::Minimq<'static, S, C, minimq::broker::NamedBroker<S>>,
    prefix: &'static str,
    meta_published: bool,
    metadata: &'static ApplicationMetadata,
}

impl<C: Clock, S: TcpClientStack<Error = smoltcp_nal::NetworkError> + Dns>
    TelemetryClient<C, S>
{
    /// Construct a new telemetry client.
    ///
    /// # Args
    /// * `mqtt` - The MQTT client
    /// * `prefix` - The device prefix to use for MQTT telemetry reporting.
    ///
    /// # Returns
    /// A new telemetry client.
    pub fn new(
        mqtt: minimq::Minimq<'static, S, C, minimq::broker::NamedBroker<S>>,
        prefix: &'static str,
        metadata: &'static ApplicationMetadata,
    ) -> Self {
        Self {
            mqtt,
            meta_published: false,
            prefix,
            metadata,
        }
    }

    /// Publish telemetry over MQTT
    ///
    /// # Note
    /// Telemetry is reported in a "best-effort" fashion. Failure to transmit telemetry will cause
    /// it to be silently dropped.
    ///
    /// # Args
    /// * `telemetry` - The telemetry to report
    pub fn publish_telemetry<T: Serialize>(
        &mut self,
        suffix: &str,
        telemetry: &T,
    ) {
        let mut topic: String<128> = self.prefix.try_into().unwrap();
        topic.push_str(suffix).unwrap();
        self.publish(&topic, telemetry)
            .map_err(|e| log::error!("Telemetry publishing error: {:?}", e))
            .ok();
    }

    pub fn publish<T: Serialize>(
        &mut self,
        topic: &str,
        payload: &T,
    ) -> Result<(), PubError<NetworkError, serde_json_core::ser::Error>> {
        self.mqtt
            .client()
            .publish(minimq::Publication::new(topic, |buf: &mut [u8]| {
                serde_json_core::to_slice(payload, buf)
            }))
    }

    /// Update the telemetry client
    ///
    /// # Note
    /// This function is provided to force the underlying MQTT state machine to process incoming
    /// and outgoing messages. Without this, the client will never connect to the broker. This
    /// should be called regularly.
    pub fn update(&mut self) {
        match self.mqtt.poll(|_client, _topic, _message, _properties| {}) {
            Err(minimq::Error::Network(
                smoltcp_nal::NetworkError::TcpConnectionFailure(
                    smoltcp_nal::smoltcp::socket::tcp::ConnectError::Unaddressable
                ),
            )) => {}

            Err(error) => log::info!("Unexpected error: {:?}", error),
            _ => {}
        }

        if !self.mqtt.client().is_connected() {
            self.meta_published = false;
            return;
        }

        // Publish application metadata
        if !self.meta_published
            && self.mqtt.client().can_publish(minimq::QoS::AtMostOnce)
        {
            let Self { mqtt, metadata, .. } = self;

            let mut topic: String<128> = self.prefix.try_into().unwrap();
            topic.push_str("/meta").unwrap();

            if mqtt
                .client()
                .publish(Publication::new(&topic, |buf: &mut [u8]| {
                    serde_json_core::to_slice(&metadata, buf)
                }))
                .is_err()
            {
                // Note(unwrap): We can guarantee that this message will be sent because we checked
                // for ability to publish above.
                mqtt.client()
                    .publish(Publication::new(
                        &topic,
                        DEFAULT_METADATA.as_bytes(),
                    ))
                    .unwrap();
            }

            self.meta_published = true;
        }
    }
}
