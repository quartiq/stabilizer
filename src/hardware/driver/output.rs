///! Driver output state handling.
use crate::hardware::I2c1Proxy;
use idsp::iir;
use smlang::statemachine;

use super::relay;

pub struct Output {
    ramp_iir: iir::IIR<f32>,
}

impl Output {
    // 1 A/s current ramp-up
    const RAMP_STEP: f32 = 1.0; // will be 1 mA current steps
    const RAMP_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(10); // will be current steps every 1 ms

    pub fn new() -> Self {
        Output {
            ramp_iir: iir::IIR::new(0., -i16::MAX as _, i16::MAX as _), // Todo: sensible defaults for driver (this is for stabilizer now)
        }
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *Disabled + Enable = EnableWaitRelay,
            EnableWaitRelay + RelayDone = RampCurrent,
            EnableWaitRelay + Enable = EnableWaitRelay,
            RampCurrent + Enable = RampCurrent,
            RampCurrent + CurrentStep / increment_current = RampCurrent,
            RampCurrent + CurrentFinal = Enabled,
            Enabled + Disable / reset_iir = DisableWaitRelay,
            DisableWaitRelay + RelayDone  = Disabled,
            DisableWaitRelay + Disable = DisableWaitRelay,
            Disabled + Disable = Disabled,
            Enabled + Enable = Enabled
        }
    }
}

impl sm::StateMachineContext for Output {
    fn increment_current(&mut self) {
        self.ramp_iir.y_offset += Output::RAMP_STEP;
    }

    fn reset_iir(&mut self) {
        self.ramp_iir.y_offset = 0.;
    }
}

#[derive(Debug)]
pub enum Error {
    Output(sm::Error),
    Relay(relay::sm::Error),
}

impl From<sm::Error> for Error {
    fn from(err: sm::Error) -> Error {
        Error::Output(err)
    }
}

impl From<relay::sm::Error> for Error {
    fn from(err: relay::sm::Error) -> Error {
        Error::Relay(err)
    }
}

impl sm::StateMachine<Output> {
    /// Start enabling sequence. Returns `relay delay` or an error is a SM is in an illegal state.
    pub fn enable(
        &mut self,
        relay: &mut relay::sm::StateMachine<relay::Relay<I2c1Proxy>>,
    ) -> Result<fugit::MillisDuration<u64>, Error> {
        self.process_event(sm::Events::Enable)?;
        relay.enable().map_err(|err| err.into())
    }

    /// Start disabling sequence. Returns `relay delay` or an error is a SM is in an illegal state.
    pub fn disable(
        &mut self,
        relay: &mut relay::sm::StateMachine<relay::Relay<I2c1Proxy>>,
    ) -> Result<fugit::MillisDuration<u64>, Error> {
        self.process_event(sm::Events::Disable)?;
        relay.disable().map_err(|err| err.into())
    }

    /// Handle realays done. Returns `Some(ramp delay)` if output is enabling and `None` if it is disabling.
    pub fn relay_done(&mut self) -> Option<fugit::MillisDuration<u64>> {
        self.process_event(sm::Events::RelayDone).unwrap();
        if *self.state() == sm::States::RampCurrent {
            Some(Output::RAMP_DELAY)
        } else {
            None
        }
    }

    /// Handle current ramp step. Returns `Some(ramp delay)` unless the ramp is done.
    /// In this case returns `None`.
    pub fn handle_ramp(
        &mut self,
        iir: iir::IIR<f32>,
    ) -> Option<fugit::MillisDuration<u64>> {
        if self.context().ramp_iir.y_offset >= iir.y_offset {
            self.process_event(sm::Events::CurrentFinal).unwrap();
            None
        } else {
            self.process_event(sm::Events::CurrentStep).unwrap();
            Some(Output::RAMP_DELAY)
        }
    }

    pub fn iir(&mut self) -> &iir::IIR<f32> {
        &self.context().ramp_iir
    }

    pub fn is_enabled(&mut self) -> bool {
        *self.state() == sm::States::Enabled
    }
}
