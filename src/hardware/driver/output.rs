///! Driver output state handling.
use idsp::iir;
use smlang::statemachine;

// use super::Channel;

pub struct Output {
    ramp_iir: iir::IIR<f32>,
}

impl Output {
    // 1 A/s current ramp-up
    const RAMP_STEP: f32 = 1e-3; // 1 mA current steps
    const RAMP_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(1); // current steps every 1 ms 

    pub fn new() -> Self {
        Output {
            ramp_iir: iir::IIR::new(0., 0., 0.1), // Todo: sensible defaults
        }
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *Disabled + Enable = EnableWaitRelays,
            EnableWaitRelay + RelaysDone = RampCurrent,
            RampCurrent + CurrentStep / increment_current = RampCurrent,
            RampCurrent + CurrentFinal = Enabled,
            Enabled + Disable / reset_iir = DisableWaitRelays,
            DisableWaitRelays + RelaysDone  = Disabled,
            Disabled + Disable = Disabled,
            Enabled + Enable = Enabled
        }
    }
}

impl sm::StateMachineContext for Output {
    fn increment_current(&mut self) {
        self.ramp_iir.y_offset = self.ramp_iir.y_offset + Output::RAMP_STEP;
    }

    fn reset_iir(&mut self) -> () {
        self.ramp_iir.y_offset = 0.;
    }
}

impl sm::StateMachine<Output> {
    pub fn enable(&mut self) -> Result<(), sm::Error> {
        self.process_event(sm::Events::Enable)?;
        Ok(())
    }

    pub fn disable(&mut self) -> Result<(), sm::Error> {
        self.process_event(sm::Events::Disable)?;
        Ok(())
    }

    /// Handle realays done. Returns `ramp delay`.
    pub fn relays_done(&mut self) -> fugit::MillisDuration<u64> {
        self.process_event(sm::Events::RelaysDone).unwrap();
        Output::RAMP_DELAY
    }

    /// Handle current ramp step. Returns Some(`ramp delay`) unless the ramp is done.
    /// In this case returns `None`.
    pub fn handle_ramp(
        &mut self,
        iir: iir::IIR<f32>,
    ) -> Option<fugit::MillisDuration<u64>> {
        if iir.y_offset >= self.context().ramp_iir.y_offset {
            self.process_event(sm::Events::CurrentFinal).unwrap();
            None
        } else {
            self.process_event(sm::Events::CurrentStep).unwrap();
            Some(Output::RAMP_DELAY)
        }
    }

    pub fn ramp_iir(&mut self) -> &iir::IIR<f32> {
        &self.context().ramp_iir
    }
}
