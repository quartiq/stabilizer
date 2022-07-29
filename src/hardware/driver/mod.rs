use enum_iterator::IntoEnumIterator;
use num_enum::TryFromPrimitive;
pub mod adc_internal;
pub mod ltc2320;
pub mod relays;
use lm75;
use stm32h7xx_hal as hal;

// Convenience type definition for the I2C bus on Driver.
pub type I2C1 =
    &'static shared_bus_rtic::CommonBus<hal::i2c::I2c<hal::stm32::I2C1>>;

// Devices on the Driver I2C bus. Used in conjunction with shared-bus-rtic to arbitrate bus.
pub struct I2cDevices {
    pub lm75: lm75::Lm75<I2C1, lm75::ic::Lm75>,
    pub relays: relays::Relays<I2C1>,
}
pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub adc_internal: adc_internal::AdcInternal,
    pub i2c_devices: I2cDevices,
}

#[derive(Clone, Copy, TryFromPrimitive, IntoEnumIterator, Debug)]
#[repr(usize)]
pub enum OutputChannelIdx {
    Zero = 0,
    One = 1,
}
