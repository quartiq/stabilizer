//! Stabilizer data stream capabilities
//!
//! # Design
//! Data streamining utilizes UDP packets to send data streams at high throughput.
//! Packets are always sent in a best-effort fashion, and data may be dropped.
//!
//! Stabilizer organizes streamed data into batches within a "Frame" that will be sent as a UDP
//! packet. Each frame consits of a header followed by sequential batch serializations. The packet
//! header is constant for all streaming capabilities, but the serialization format after the header
//! is application-defined.
//!
//! ## Frame Header
//! The header consists of the following, all in little-endian.
//!
//! * **Magic word 0x057B** (u16): a constant to identify Stabilizer streaming data.
//! * **Format Code** (u8): a unique ID that indicates the serialization format of each batch of data
//!   in the frame. Refer to [Format] for further information.
//! * **Batch Count** (u8): the number of batches of data.
//! * **Sequence Number** (u32): an the sequence number of the first batch in the frame.
//!   This can be used to determine if and how many stream batches are lost.
//!
//! # Example
//! A sample Python script is available in `scripts/stream_throughput.py` to demonstrate reception
//! of streamed data.

#![no_std]

use core::{fmt::Write, net::SocketAddr};
use heapless::String;
use num_enum::IntoPrimitive;
use serde::Serialize;
use serde_with::DeserializeFromStr;

/// Represents the destination for the UDP stream to send data to.
///
/// # Miniconf
/// `<addr>:<port>`
///
/// * `<addr>` is an IPv4 address. E.g. `192.168.0.1`
/// * `<port>` is any unsigned 16-bit value.
///
/// ## Example
/// `192.168.0.1:1234`
#[derive(Copy, Clone, Debug, DeserializeFromStr, PartialEq, Eq)]
pub struct Target(pub SocketAddr);

impl Default for Target {
    fn default() -> Self {
        Self("0.0.0.0:0".parse().unwrap())
    }
}

impl Serialize for Target {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        let mut display: String<30> = String::new();
        write!(&mut display, "{}", self.0).unwrap();
        serializer.serialize_str(&display)
    }
}

impl core::str::FromStr for Target {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let addr = SocketAddr::from_str(s)
            .map_err(|_| "Invalid socket address format")?;
        Ok(Self(addr))
    }
}

/// Specifies the format of streamed data
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, IntoPrimitive)]
pub enum Format {
    /// Reserved, unused format specifier.
    Unknown = 0,

    /// ADC0, ADC1, DAC0, and DAC1 sequentially in little-endian format.
    ///
    /// # Example
    /// With a batch size of 2, the serialization would take the following form:
    /// ```
    /// <ADC0[0]> <ADC0[1]> <ADC1[0]> <ADC1[1]> <DAC0[0]> <DAC0[1]> <DAC1[0]> <DAC1[1]>
    /// ```
    AdcDacData = 1,

    /// FLS (fiber length stabilization) format. See the FLS application.
    Fls = 2,

    /// Thermostat-EEM data. See `thermostat-eem` repo and application.
    ThermostatEem = 3,

    /// MPLL data
    Mpll = 4,
}

#[cfg(target_arch = "arm")]
mod stream;
#[cfg(target_arch = "arm")]
pub use stream::*;
