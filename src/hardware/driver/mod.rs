pub mod dac;
pub mod internal_adc;
pub mod ltc2320;
pub mod output;
pub mod relay;

use self::output::Selftest;
use super::I2c1Proxy;
use idsp::iir;
use lm75;
pub mod alarm;
use log::error;
use miniconf::Miniconf;
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
    pub laser_interlock: LaserInterlock,
}

#[derive(
    Clone, Copy, Debug, PartialEq, Eq, TryFromPrimitive, Serialize, Deserialize,
)]
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

/// A [Reason] why the interlock has tripped.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub enum Reason {
    /// Default after device reset.
    #[default]
    Reset,

    /// Due to an active alarm event.
    Alarm,

    /// Due to an alarm timeout. Aka there was no alarm reresh in the timeout period.
    AlarmTimeout,

    /// An overcurrent condition on [Channel] while the output was enabled.
    Overcurrent(Channel),

    /// An overvoltage condition on [Channel] while the output was enabled.
    Overvoltage(Channel),

    /// The device selftest during the channel enabling sequence failed.
    /// See [SelftestFail] for details.
    Selftest(Selftest),
}
pub struct LaserInterlock {
    reason: Option<Reason>,
    pin: hal::gpio::Pin<'B', 13, hal::gpio::Output>,
}

impl LaserInterlock {
    pub fn new(
        mut pin: hal::gpio::Pin<'B', 13, hal::gpio::Output>,
    ) -> LaserInterlock {
        pin.set_low();
        LaserInterlock {
            reason: Some(Reason::Reset),
            pin,
        }
    }

    pub fn set(&mut self, reason: Option<Reason>) {
        // only update if no reason yet or if clearing (remember first reason)
        if self.reason.is_none() || reason.is_none() {
            self.pin.set_state(reason.is_none().into());
            self.reason = reason;
            if let Some(re) = reason {
                error!("Interlock tripped! {:?}", re)
            }
        }
    }

    pub fn reason(&self) -> Option<Reason> {
        self.reason
    }
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct LowNoiseSettings {
    /// Configure the IIR filter parameters. Only active once channel is enabled.
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    pub iir: iir::IIR<f32>,

    /// Specified true if DI1 should be used as a "hold" input.
    ///
    /// # Value
    /// "true" or "false"
    pub allow_hold: bool,

    /// Specified true if "hold" should be forced regardless of DI1 state and hold allowance.
    ///
    /// # Value
    /// "true" or "false"
    pub force_hold: bool,

    /// Output enabled. `True` to enable, `False` to disable.
    ///
    /// # Value
    /// [bool]
    pub output_enabled: bool,

    /// Configure the interlock current at which the laser interlock will trip.
    ///
    /// # Value
    /// Any positive value.
    pub interlock_current: f32,

    /// Configure the interlock voltage at which the laser interlock will trip.
    ///
    /// # Value
    /// Any positive value.
    pub interlock_voltage: f32,
}

impl Default for LowNoiseSettings {
    fn default() -> Self {
        Self {
            iir: iir::IIR::new(0., 0.0, 0.0),
            allow_hold: false,
            force_hold: false,
            output_enabled: false,
            interlock_current: 0.,
            interlock_voltage: 0.,
        }
    }
}

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct HighPowerSettings {
    /// Configure the output current. Only active once channel is enabled.
    ///
    /// # Value
    /// Any positive value up to the maximum current for the high power channel.
    pub current: f32,

    /// Output enabled. `True` to enable, `False` to disable.
    ///
    /// # Value
    /// bool
    pub output_enabled: bool,

    /// Configure the interlock current at which the laser interlock will trip.
    ///
    /// # Value
    /// Any positive value.
    pub interlock_current: f32,

    /// Configure the interlock voltage at which the laser interlock will trip.
    ///
    /// # Value
    /// Any positive value.
    pub interlock_voltage: f32,
}

impl Default for HighPowerSettings {
    fn default() -> Self {
        Self {
            current: 0.0,
            output_enabled: false,
            interlock_current: 0.,
            interlock_voltage: 0.,
        }
    }
}
