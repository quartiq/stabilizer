///! Driver output state handling.
///!
///! Driver has two independent output channels.
///! Each channel can be in various states during operation, powerup and powerdown.
use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use idsp::iir;
use mcp230xx::{Mcp23008, Mcp230xx};
use smlang::statemachine;

use super::{relay::Relay, Channel};

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
    const RAMP_STEP: f32 = 1.0; // will be 1 mA current steps
    const RAMP_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(10); // will be current steps every 1 ms

    pub fn new(
        gpio: &'static spin::Mutex<Mcp230xx<I2C, Mcp23008>>,
        channel: Channel,
    ) -> Self {
        Output {
            iir: iir::IIR::new(0., -i16::MAX as _, i16::MAX as _), // Todo: sensible defaults for driver (this is for stabilizer now)
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
            EnableWaitK1 + Tick / increment_current = RampCurrent,
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
    fn delay(&self) -> Option<fugit::MillisDuration<u64>> {
        match *self.state() {
            sm::States::DisableWaitK0 => Some(Relay::<I2C>::K0_DELAY),
            sm::States::DisableWaitK1 => Some(Relay::<I2C>::K1_DELAY),
            sm::States::EnableWaitK0 => Some(Relay::<I2C>::K0_DELAY),
            sm::States::EnableWaitK1 => Some(Relay::<I2C>::K1_DELAY),
            sm::States::RampCurrent => Some(Output::<I2C>::RAMP_DELAY),
            _ => None,
        }
    }
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
    pub fn handle_output_event(
        &mut self,
        iir: &iir::IIR<f32>,
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
                if self.context().iir.y_offset >= iir.y_offset {
                    self.process_event(sm::Events::RampDone).unwrap();
                } else {
                    self.process_event(sm::Events::Tick).unwrap();
                }
            }
            _ => (),
        };
        self.delay()
    }

    pub fn iir(&mut self) -> &iir::IIR<f32> {
        &self.context().iir
    }

    pub fn is_enabled(&mut self) -> bool {
        *self.state() == sm::States::Enabled
    }
}
