pub mod internal_adc;
pub mod ltc2320;
pub mod output;
pub mod relay;
use super::I2c1Proxy;
use lm75;
pub mod interlock;
use num_enum::TryFromPrimitive;

/// Devices on Driver + Driver headerboard
pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub internal_adc: internal_adc::InternalAdc,
    pub lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    pub relay_sm: [relay::sm::StateMachine<relay::Relay<I2c1Proxy>>; 2],
    pub output_sm: [output::sm::StateMachine<output::Output>; 2],
    // dac
}

#[derive(Clone, Copy, Debug, PartialEq, TryFromPrimitive)]
#[repr(usize)]
pub enum Channel {
    LowNoise = 0,
    HighPower = 1,
}
