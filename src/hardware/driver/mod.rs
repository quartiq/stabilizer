pub mod dac;
pub mod internal_adc;
pub mod ltc2320;
pub mod output;
pub mod relay;
use super::I2c1Proxy;
use hal::gpio::PinState;
use lm75;
pub mod interlock;
use num_enum::TryFromPrimitive;
use serde::{Deserialize, Serialize};
use stm32h7xx_hal as hal;

pub type Spi1Proxy = &'static shared_bus_rtic::CommonBus<
    hal::spi::Spi<stm32h7xx_hal::stm32::SPI1, stm32h7xx_hal::spi::Enabled>,
>;

/// Devices on Driver + Driver headerboard
pub struct DriverDevices {
    pub ltc2320: ltc2320::Ltc2320,
    pub internal_adc: internal_adc::InternalAdc,
    pub lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    pub output_sm: [output::sm::StateMachine<output::Output<I2c1Proxy>>; 2],
    pub dac: [dac::Dac<Spi1Proxy>; 2],
    pub laser_interlock_pin: hal::gpio::Pin<'B', 13, hal::gpio::Output>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, TryFromPrimitive)]
#[repr(usize)]
pub enum Channel {
    LowNoise = 0,
    HighPower = 1,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(usize)]
pub enum ChannelVariant {
    LowNoiseSink,
    LowNoiseSource,
    HighPowerSink,
    HighPowerSource,
}

impl ChannelVariant {
    const R_OUT_LN: f32 = 40.0; // Low noise side output resistor
    const R_OUT_HP: f32 = 0.68; // High power side output resistor
    fn transimpedance(&self) -> f32 {
        match self {
            ChannelVariant::LowNoiseSink => -Self::R_OUT_LN, // negated
            ChannelVariant::LowNoiseSource => Self::R_OUT_LN,
            ChannelVariant::HighPowerSink => -Self::R_OUT_HP, // negated
            ChannelVariant::HighPowerSource => Self::R_OUT_HP,
        }
    }
}

#[derive(
    Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize,
)]
pub enum LaserInterlock {
    Asserted,
    #[default]
    Tripped,
}

impl From<PinState> for LaserInterlock {
    fn from(state: PinState) -> Self {
        match state {
            PinState::High => Self::Asserted,
            PinState::Low => Self::Tripped,
        }
    }
}
