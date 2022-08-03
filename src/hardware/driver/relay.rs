use core::fmt::Debug;

///! Driver relays driver
///!
///! There are 2 relays at the driver output:
///!    - one that shorts the output to ground
///!    - one that connects the current source/sink to the output
///!
///! The relays are controlled via an I2C io-expander.
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mcp23017::{Level, Pin, MCP23017};

use super::Channel;
use smlang::statemachine;

#[derive(Debug, Copy, Clone)]
pub enum RelayError {
    /// Indicates that the I2C expander IC is in use
    Mcp23008InUse,
}

// Driver low noise output relays pins
#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum LnRelayPin {
    K1_EN_N,
    K1_EN,
    K0_D,
    K0_CP,
}

impl From<LnRelayPin> for Pin {
    fn from(pin: LnRelayPin) -> Self {
        match pin {
            LnRelayPin::K1_EN_N => Self::A0,
            LnRelayPin::K1_EN => Self::A1,
            LnRelayPin::K0_D => Self::A2,
            LnRelayPin::K0_CP => Self::A3,
        }
    }
}

// Driver high power output relays pins
#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum HpRelayPin {
    K1_EN_N,
    K1_EN,
    K0_D,
    K0_CP,
}

impl From<HpRelayPin> for Pin {
    fn from(pin: HpRelayPin) -> Self {
        match pin {
            HpRelayPin::K1_EN_N => Self::A4,
            HpRelayPin::K1_EN => Self::A5,
            HpRelayPin::K0_D => Self::A6,
            HpRelayPin::K0_CP => Self::A7,
        }
    }
}

pub struct Relay<'a, I2C: WriteRead + Write> {
    mutex: &'a spin::Mutex<MCP23017<I2C>>,
    ch: Channel,
}

impl<'a, I2C, E> Relay<'a, I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    pub fn new(mutex: &'a spin::Mutex<MCP23017<I2C>>, ch: Channel) -> Self {
        Relay { mutex, ch }
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *Disabled + Enable / engage_k0 = EnableWaitK0,
            EnableWaitK0 + RelayDone / engage_k1 = EnableWaitK1,
            EnableWaitK1 + RelayDone = Enabled,
            Enabled + Disable / disengage_k1 = DisableWaitK1,
            DisableWaitK1 + RelayDone / disengage_k0 = DisableWaitK0,
            DisableWaitK0 + RelayDone = Disabled
        }
    }
}

impl<'a, I2C, E> sm::StateMachineContext for Relay<'a, I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // K0 to upper
    fn engage_k0(&mut self) -> () {
        let mut mcp = self
            .mutex
            .try_lock()
            .ok_or(RelayError::Mcp23008InUse)
            .unwrap(); // panic here if in use
        if self.ch == Channel::LowNoise {
            mcp.write_pin(LnRelayPin::K1_EN.into(), Level::High)
                .unwrap(); // dummy write for now
        } else {
            mcp.write_pin(HpRelayPin::K1_EN.into(), Level::High)
                .unwrap(); // dummy write for now
        }
        todo!()
    }

    // K0 to lower
    fn disengage_k0(&mut self) -> () {
        todo!()
    }

    // K1 to upper
    fn engage_k1(&mut self) -> () {
        todo!()
    }

    // K1 to lower
    fn disengage_k1(&mut self) -> () {
        todo!()
    }
}

impl<'a, I2C, E> sm::StateMachine<Relay<'a, I2C>>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    pub fn enable(&mut self) {
        self.process_event(sm::Events::Enable).unwrap();
    }

    pub fn disable(&mut self) {
        self.process_event(sm::Events::Disable).unwrap();
    }

    pub fn handle_relay_done(&mut self) {
        self.process_event(sm::Events::RelayDone).unwrap();
    }
}
