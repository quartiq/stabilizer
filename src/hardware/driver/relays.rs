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

// Driver relays pins
#[allow(non_camel_case_types, clippy::upper_case_acronyms, dead_code)]
#[derive(Debug, Copy, Clone, PartialEq)]
enum RelayPin {
    HP_K1_EN_N,
    HP_K1_EN,
    HP_K0_D,
    HP_K0_CP,
    LP_K1_EN_N,
    LP_K1_EN,
    LP_K0_D,
    LP_K0_CP,
}

impl From<RelayPin> for Pin {
    fn from(pin: RelayPin) -> Self {
        match pin {
            RelayPin::HP_K1_EN_N => Self::A0,
            RelayPin::HP_K1_EN => Self::A1,
            RelayPin::HP_K0_D => Self::A2,
            RelayPin::HP_K0_CP => Self::A3,
            RelayPin::LP_K1_EN_N => Self::A4,
            RelayPin::LP_K1_EN => Self::A5,
            RelayPin::LP_K0_D => Self::A6,
            RelayPin::LP_K0_CP => Self::A7,
        }
    }
}

// The three possible states the tow relays on driver can be.
// The output can never be open and not grounded.
pub enum RelayState {
    ShuntedGrounded, // Driver output stage connected to shunt reistor. Output grounded.
    ConnectedGrounded, // Driver output stage connected to output and grounded.
    Connected, // Driver output stage connected to output and grounding relais open.
}

pub struct Relay<I2C: WriteRead + Write> {
    mcp23017: mcp23017::MCP23017<I2C>,
    state: RelayState,
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
        Relay {
            mcp23017,
            state: RelayState::ShuntedGrounded,
        }
    }
}

pub mod sm {
    use super::*;
    statemachine! {
        transitions: {
            *ShuntedGrounded + Connect / connect = ConnectedGrounded,
            ConnectedGrounded + OpenK1 / open_k1 = Connected,
            Connected + CloseK1 / close_k1 = ConnectedGrounded,
            ConnectedGrounded + Shunt / shunt = ShuntedGrounded
        }
    }
}

impl<I2C, E> sm::StateMachineContext for Relay<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    fn connect(&mut self) -> () {
        todo!()
    }

    fn shunt(&mut self) -> () {
        todo!()
    }

    fn open_k1(&mut self) -> () {
        todo!()
    }

    fn close_k1(&mut self) -> () {
        todo!()
    }
}
