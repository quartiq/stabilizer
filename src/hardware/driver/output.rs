///! Driver output state handling.
///!
///! Driver has two independent output channels.
///! Each channel can be in various states during operation, powerup and powerdown.
use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use idsp::iir;
use mcp230xx::{Mcp23008, Mcp230xx};
use miniconf::Miniconf;
use smlang::statemachine;

use super::{relay::Relay, Channel};

const LN_MAX_I_DEFAULT: f32 = 0.2; // default maximum current for the low noise channel in ampere

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
            iir: iir::IIR::new(0., 0.0, LN_MAX_I_DEFAULT),
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

/// Driver [Output].
/// An Output consits of the output [Relay]s and an IIR to implement a current ramp
/// during an enabling sequence and a current hold during powerdown.
pub struct Output<I2C: WriteRead + Write + 'static> {
    iir: iir::IIR<f32>,
    relay: Relay<I2C>,
}

impl<I2C, E> Output<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // 1 A/s current ramp-up
    const RAMP_STEP: f32 = 1e-3;
    const RAMP_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(1);

    pub fn new(
        gpio: &'static spin::Mutex<Mcp230xx<I2C, Mcp23008>>,
        channel: Channel,
    ) -> Self {
        Output {
            iir: iir::IIR::new(0., 0.0, f32::MAX),
            relay: Relay::new(gpio, channel),
        }
    }
}
#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Delay(pub fugit::MillisDuration<u64>);

pub mod sm {
    use super::*;

    statemachine! {
        transitions: {
            // Enable sequence
            *Disabled + Enable / engage_k0 = EnableWaitK0,
            EnableWaitK0 + Tick / disengage_k1 = EnableWaitK1,
            EnableWaitK1 + Tick = RampCurrent,
            RampCurrent + Tick / increment_current = RampCurrent,
            RampCurrent + RampDone = Enabled,

            // Abort transitions
            EnableWaitK0 | EnableWaitK1 | RampCurrent + Disable = Abort,
            Abort + Tick / engage_k1_and_hold_iir = DisableWaitK1,

            // Disable sequence
            Enabled + Disable / engage_k1_and_hold_iir = DisableWaitK1,
            DisableWaitK1 + Tick / disengage_k0_and_reset_iir = DisableWaitK0,
            DisableWaitK0 + Tick = Disabled,

            Disabled + Disable = Disabled,
            Enabled + Enable = Enabled
        }
    }
}

impl<I2C, E> sm::StateMachineContext for Output<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    fn increment_current(&mut self) {
        self.iir.y_offset += Output::<I2C>::RAMP_STEP;
    }

    fn engage_k0(&mut self) {
        self.relay.engage_k0();
    }

    fn disengage_k0_and_reset_iir(&mut self) {
        self.iir.y_offset = 0.;
        self.iir.ba = [0., 0., 0., 0., 0.];
        self.relay.disengage_k0();
    }

    fn disengage_k1(&mut self) {
        self.relay.disengage_k1();
    }

    fn engage_k1_and_hold_iir(&mut self) {
        self.iir.y_offset = 0.;
        self.iir.ba = [0., 0., 0., 1., 0.];
        self.relay.engage_k1();
    }
}

impl<I2C, E> sm::StateMachine<Output<I2C>>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Start enabling sequence. Returns `Some(relay delay)` or `None` if we started an Abort
    /// or an error if SM is in a state where we can't enable.
    pub fn set_enable(
        &mut self,
        enable: bool,
    ) -> Result<Option<fugit::MillisDuration<u64>>, sm::Error> {
        let event = match enable {
            true => sm::Events::Enable,
            false => sm::Events::Disable,
        };
        if *self.process_event(event)? != sm::States::Abort {
            Ok(Some(Relay::<I2C>::K0_DELAY)) // engage K0 first
        } else {
            Ok(None)
        }
    }

    /// Handle an event that happens during the enabling/disabling sequence.
    pub fn handle_tick(
        &mut self,
        target: &f32,
    ) -> Option<fugit::MillisDuration<u64>> {
        match *self.state() {
            sm::States::EnableWaitK0
            | sm::States::DisableWaitK1
            | sm::States::EnableWaitK1
            | sm::States::DisableWaitK0
            | sm::States::Abort => {
                self.process_event(sm::Events::Tick).unwrap();
            }
            // handle the ramp
            sm::States::RampCurrent => {
                if self.context().iir.y_offset
                    >= (*target - Output::<I2C>::RAMP_STEP)
                {
                    self.process_event(sm::Events::RampDone).unwrap();
                } else {
                    self.process_event(sm::Events::Tick).unwrap();
                }
            }
            _ => (),
        };
        match *self.state() {
            sm::States::DisableWaitK0 => Some(Relay::<I2C>::K0_DELAY),
            sm::States::DisableWaitK1 => Some(Relay::<I2C>::K1_DELAY),
            sm::States::EnableWaitK0 => Some(Relay::<I2C>::K0_DELAY),
            sm::States::EnableWaitK1 => Some(Relay::<I2C>::K1_DELAY),
            sm::States::RampCurrent => Some(Output::<I2C>::RAMP_DELAY),
            _ => None,
        }
    }

    pub fn iir(&mut self) -> &iir::IIR<f32> {
        &self.context().iir
    }

    pub fn is_enabled(&mut self) -> bool {
        *self.state() == sm::States::Enabled
    }
}
