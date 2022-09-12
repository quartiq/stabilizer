pub mod adc_internal;
pub mod ltc2320;
pub mod relay;
pub mod output;
use super::I2c1Proxy;
use lm75;
pub mod interlock;

use self::relay::sm::StateMachine;

/// Devices on Driver + Driver headerboard
pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub adc_internal: adc_internal::AdcInternal,
    pub lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    pub relay_sm: [StateMachine<relay::Relay<I2c1Proxy>>; 2],
    // dac
    // output_state
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(usize)]
pub enum Channel {
    LowNoise = 0,
    HighPower = 1,
}
