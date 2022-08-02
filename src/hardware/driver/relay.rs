use core::fmt::Debug;

///! Driver relays driver
///!
///! There are 2 relays at the driver output:
///!    - one that shorts the output to ground
///!    - one that connects the current source/sink to the output
///!
///! The relays are controlled via an I2C io-expander.
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mcp23017::Pin;

use super::OutputChannelIdx;
use smlang::statemachine;

// Driver low power output relays pins
#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum LpRelayPin {
    K1_EN_N,
    K1_EN,
    K0_D,
    K0_CP,
}

impl From<LpRelayPin> for Pin {
    fn from(pin: LpRelayPin) -> Self {
        match pin {
            LpRelayPin::K1_EN_N => Self::A0,
            LpRelayPin::K1_EN => Self::A1,
            LpRelayPin::K0_D => Self::A2,
            LpRelayPin::K0_CP => Self::A3,
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

pub struct Relay<I2C: WriteRead + Write> {
    mcp23017: mcp23017::MCP23017<I2C>,
}

impl<I2C, E> Relay<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    pub fn new(i2c: I2C) -> Self {
        let mcp23017 =
            mcp23017::MCP23017::new_default(i2c, mcp23017::Variant::MCP23008)
                .unwrap();
        Relay { mcp23017 }
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *Disabled + Enable / enable = EnableWaitK0,
            EnableWaitK0 + RelayDone / enable_k1 = EnableWaitK1,
            EnableWaitK1 + RelayDone = Enabled,
            Enabled + Disable / disable = DisableWaitK1,
            DisableWaitK1 + RelayDone / disable_k0 = DisableWaitK0,
            DisableWaitK0 + RelayDone = Disabled
        }
    }
}

impl<I2C, E> sm::StateMachineContext for Relay<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // K0 to upper
    fn enable(&mut self) -> () {
        todo!()
    }

    // K0 to lower
    fn disable_k0(&mut self) -> () {
        todo!()
    }

    // K1 to upper
    fn enable_k1(&mut self) -> () {
        todo!()
    }

    // K1 to lower
    fn disable(&mut self) -> () {
        todo!()
    }
}

impl<I2C, E> sm::StateMachine<Relay<I2C>>
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
