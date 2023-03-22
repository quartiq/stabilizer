pub mod dac;
pub mod internal_adc;
pub mod ltc2320;
pub mod output;
pub mod relay;

use self::output::SelfTest;
use super::I2c1Proxy;
use idsp::iir;
use lm75;
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
    pub driver_lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    pub header_lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
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
    const R_SHUNT_LN: f32 = 40.0; // Low noise side output resistor (Ω)
    const R_SHUNT_HP: f32 = 0.068; // High power side output resistor (Ω)
    const GAIN_FOLLOWER_HP: f32 = 1. / 41.; // High power side DAC output divider ratio (V/V)

    /// Returns the effective scale of the DAC output voltage to the channel output current.
    /// The scale is negative for source channels since a lower DAC voltage leads to more current.
    fn dac_to_output_current_scale(&self) -> f32 {
        match self {
            ChannelVariant::LowNoiseSource => -Self::R_SHUNT_LN, // negated
            ChannelVariant::LowNoiseSink => Self::R_SHUNT_LN,
            ChannelVariant::HighPowerSource => {
                -Self::R_SHUNT_HP / Self::GAIN_FOLLOWER_HP
            } // negated
            ChannelVariant::HighPowerSink => {
                Self::R_SHUNT_HP / Self::GAIN_FOLLOWER_HP
            }
        }
    }
    /// Returns the transadmittance of the channel output shunt resistor (A/V). Negative for sink variants
    /// since current flowing into the output leads to a positive shunt voltage.
    fn current_sense_transadmittance(&self) -> f32 {
        match self {
            ChannelVariant::LowNoiseSource => 1. / Self::R_SHUNT_LN,
            ChannelVariant::LowNoiseSink => -1. / Self::R_SHUNT_LN, // negative transadmittance for sink
            ChannelVariant::HighPowerSource => 1. / Self::R_SHUNT_HP,
            ChannelVariant::HighPowerSink => -1. / Self::R_SHUNT_HP, // negative transadmittance for sink
        }
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Condition {
    pub channel: Channel,
    pub threshold: f32,
    pub read: f32,
}

/// A [Reason] why the interlock has tripped.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub enum Reason {
    /// Default after device reset.
    #[default]
    Reset,

    /// Due to an active alarm event.
    Alarm,

    /// Due to an alarm timeout. Aka there was no alarm reresh in the timeout period.
    AlarmTimeout,

    /// An overcurrent condition on [Channel] while the output was enabled.
    Overcurrent(Condition),

    /// An overvoltage condition on [Channel] while the output was enabled.
    Overvoltage(Condition),

    /// The device selftest during the channel enabling sequence failed.
    /// See [output::FailReason] for details.
    Selftest(SelfTest),
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

    pub fn set(
        &mut self,
        reason: Option<Reason>,
        output_state: &mut [output::sm::StateMachine<output::Output<I2c1Proxy>>;
                 2],
    ) -> [Option<fugit::MillisDuration<u64>>; 2] {
        // only update if no reason yet or if clearing (remember first reason)
        if self.reason.is_none() || reason.is_none() {
            self.pin.set_state(reason.is_none().into());
            // disable both outputs if interlock is tripped
            self.reason = reason;
            if self.reason.is_some() {
                [
                    output_state[0].set_enable(false).unwrap(),
                    output_state[1].set_enable(false).unwrap(),
                ]
            } else {
                [None, None]
            }
        } else {
            [None, None]
        }
    }

    pub fn reason(&self) -> Option<Reason> {
        self.reason.clone()
    }
}

#[derive(Clone, Copy, Debug, Miniconf, Serialize, Deserialize)]
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

#[derive(Clone, Copy, Debug, Miniconf, Serialize, Deserialize)]
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
