pub mod adc_internal;
pub mod ltc2320;
pub mod relay;
use super::I2c1Proxy;
use lm75;
use stm32h7xx_hal as hal;

use self::relay::sm::StateMachine;

// Convenience type definition for the I2C bus on Driver.
pub type I2C1 =
    &'static shared_bus_rtic::CommonBus<hal::i2c::I2c<hal::stm32::I2C1>>;

/// Devices on Driver + Driver headerboard
pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub adc_internal: adc_internal::AdcInternal,
    pub lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    pub relay_sm: [StateMachine<relay::Relay<'static, I2c1Proxy>>; 2], // dac
                                                                  // output_state
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(usize)]
pub enum Channel {
    LowNoise = 0,
    HighPower = 1,
}
