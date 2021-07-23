//! This module contains any compile-time configuration parameters for the Stabilizer firmware.

/// MQTT broker IPv4 address
///
/// In the default configuration, the IP address is defined as 10.34.16.10.
pub const MQTT_BROKER: [u8; 4] = [10, 35, 16, 10];

/// Sampling Frequency
///
/// Define the frequency at which ADCs (and DACs) are sampled at.
///
/// # Units
/// The units of this parameter are specified as a logarithmic number of ticks of the internal
/// timer, which runs at 100 MHz.
///
/// ## Example
/// With a value of 7, this corresponds to 2^7 = 128 ticks. Each tick of the 100 MHz timer requires
/// 10ns.
///
/// Sampling Period = 10ns * 128 = 1.28 us
/// Sampling Frequency = 781.25 KHz
///
/// Or more succinctly:
/// `F_s = 100 MHz / (2 ^ ADC_SAMPLE_TICKS_LOG2)`
pub const ADC_SAMPLE_TICKS_LOG2: u8 = 7;

/// Sample Batch Sizing
///
/// The sample batch size defines how many samples are collected before the DSP routines are
/// executed.
///
/// # Note
/// Smaller batch sizes result in less input -> output latency, but come at the cost of reduced
/// maximum sampling frequency.
///
/// # Units
/// The units of the batch size are specified logarithmically.
///
/// ## Example
/// With a value of 3, the number of samples per batch is 2^3 = 8.
pub const SAMPLE_BUFFER_SIZE_LOG2: u8 = 3;
