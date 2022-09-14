///! Driver output state handling.
use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use idsp::iir;
use mcp230xx::{Level, Mcp23008, Mcp230xx};
use smlang::statemachine;

use super::{relay::Relay, Channel};

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

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            // Enable sequence
            *Disabled + Enable / engage_k0 = EnableWaitK0,
            EnableWaitK0 + RelayDone / disengage_k1 = EnableWaitK1,
            EnableWaitK1 + RelayDone = RampCurrent,
            RampCurrent + CurrentStep / increment_current = RampCurrent,
            RampCurrent + CurrentFinal = Enabled,

            // Abort transitions
            EnableWaitK0 | EnableWaitK1 | RampCurrent + Disable = Abort,
            Abort + RelayDone / engage_k1_and_hold_iir = DisableWaitK1,

            // Disable sequence
            Enabled + Disable / engage_k1_and_hold_iir = DisableWaitK1,
            DisableWaitK1 + RelayDone / disengage_k0_and_reset_iir = DisableWaitK0,
            DisableWaitK0 + RelayDone = Disabled,

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

    // set K0 to upper position (note that "upper" and "lower" refer to the schematic)
    fn engage_k0(&mut self) {
        let mut mcp = self.relay.gpio.try_lock().unwrap();
        // set flipflop data pin
        // mcp.set_gpio(self.relay.k0_d.into(), Level::High).unwrap();
        // set flipflop clock input low to prepare rising edge
        // mcp.set_gpio(self.relay.k0_cp.into(), Level::Low).unwrap();
        // set flipflop clock input high to generate rising edge
        // mcp.set_gpio(self.relay.k0_cp.into(), Level::High).unwrap();
    }

    // set K0 to lower position
    fn disengage_k0_and_reset_iir(&mut self) {
        self.iir.y_offset = 0.;
        let mut mcp = self.relay.gpio.try_lock().unwrap();
        // mcp.set_gpio(self.relay.k0_d.into(), Level::High).unwrap();
        // mcp.set_gpio(self.relay.k0_cp.into(), Level::Low).unwrap();
        // mcp.set_gpio(self.relay.k0_cp.into(), Level::High).unwrap();
    }

    // set K1 to upper position
    fn disengage_k1(&mut self) {
        let mut mcp = self.relay.gpio.try_lock().unwrap();
        // set en high and en _n low in order to engage K1
        // mcp.set_gpio(self.relay.k1_en.into(), Level::Low).unwrap();
        // mcp.set_gpio(self.relay.k1_en_n.into(), Level::High)
        // .unwrap();
    }

    // set K1 to lower position and output current to zero
    fn engage_k1_and_hold_iir(&mut self) {
        let mut mcp = self.relay.gpio.try_lock().unwrap();
        // mcp.set_gpio(self.relay.k1_en.into(), Level::High).unwrap();
        // mcp.set_gpio(self.relay.k1_en_n.into(), Level::Low).unwrap();
    }
}

impl<I2C, E> sm::StateMachine<Output<I2C>>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Start enabling sequence. Returns `Some(relay delay)` or None if we started an Abort
    /// or an error if SM is in a state where we can't enable.
    pub fn enable(
        &mut self,
    ) -> Result<Option<fugit::MillisDuration<u64>>, sm::Error> {
        self.process_event(sm::Events::Enable)?;
        if *self.state() != sm::States::Abort {
            Ok(Some(Relay::<I2C>::K0_DELAY)) // engage K0 first
        } else {
            Ok(None)
        }
    }

    /// Start disabling sequence. Returns `Some(relay delay)` or None if we started an Abort
    /// or an error if SM is in a state where we can't disable.
    pub fn disable(
        &mut self,
    ) -> Result<Option<fugit::MillisDuration<u64>>, sm::Error> {
        self.process_event(sm::Events::Disable)?;
        if *self.state() != sm::States::Abort {
            Ok(Some(Relay::<I2C>::K1_DELAY)) // engage K1 first
        } else {
            Ok(None)
        }
    }

    /// Handle an event that happens during the enabling/disabling sequence.
    pub fn handle_output_event(
        &mut self,
        iir: iir::IIR<f32>,
    ) -> Option<fugit::MillisDuration<u64>> {
        match *self.state() {
            // engage K1 second
            sm::States::EnableWaitK0 => {
                self.process_event(sm::Events::RelayDone).unwrap();
                Some(Relay::<I2C>::K1_DELAY)
            }
            // disengage K0 second
            sm::States::DisableWaitK1 => {
                self.process_event(sm::Events::RelayDone).unwrap();
                Some(Relay::<I2C>::K0_DELAY)
            }
            // start ramp
            sm::States::EnableWaitK1 => {
                self.process_event(sm::Events::RelayDone).unwrap();
                Some(Output::<I2C>::RAMP_DELAY)
            }
            // powerdown finished, no need to wait
            sm::States::DisableWaitK0 => {
                self.process_event(sm::Events::RelayDone).unwrap();
                None
            }
            // handle the ramp
            sm::States::RampCurrent => {
                if self.context().iir.y_offset >= iir.y_offset {
                    self.process_event(sm::Events::CurrentFinal).unwrap();
                    None
                } else {
                    self.process_event(sm::Events::CurrentStep).unwrap();
                    Some(Output::<I2C>::RAMP_DELAY)
                }
            }
            // first event after aborting => engage K1 to be safe in all cases
            sm::States::Abort => {
                self.process_event(sm::Events::RelayDone).unwrap();
                Some(Output::<I2C>::RAMP_DELAY)
            }
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
