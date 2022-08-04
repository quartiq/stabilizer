///! Driver relays driver
///!
///! There are 2 relays at the driver output:
///!    - one that shorts the output to ground
///!    - one that connects the current source/sink to the output
///!
///! The relays are controlled via an I2C io-expander.
/// hide mutex
/// just pins as member variables for Relay
use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mcp230xx::mcp23008::{Level, Pin, MCP23008};

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
enum RelayPin {
    LN_K1_EN_N,
    LN_K1_EN,
    LN_K0_D,
    LN_K0_CP,
    HP_K1_EN_N,
    HP_K1_EN,
    HP_K0_D,
    HP_K0_CP,
}

impl From<RelayPin> for Pin {
    fn from(pin: RelayPin) -> Self {
        match pin {
            RelayPin::LN_K1_EN_N => Self::Gp0,
            RelayPin::LN_K1_EN => Self::Gp1,
            RelayPin::LN_K0_D => Self::Gp2,
            RelayPin::LN_K0_CP => Self::Gp3,
            RelayPin::HP_K1_EN_N => Self::Gp4,
            RelayPin::HP_K1_EN => Self::Gp5,
            RelayPin::HP_K0_D => Self::Gp6,
            RelayPin::HP_K0_CP => Self::Gp7,
        }
    }
}

pub struct Relay<'a, I2C: WriteRead + Write> {
    mutex: &'a spin::Mutex<MCP23008<I2C>>,
    k1_en_n: RelayPin,
    k1_en: RelayPin,
    k1_d: RelayPin,
    k1_cp: RelayPin,
}

impl<'a, I2C, E> Relay<'a, I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    pub fn new(mutex: &'a spin::Mutex<MCP23008<I2C>>, ch: Channel) -> Self {
        let (k1_en_n, k1_en, k1_d, k1_cp) = if ch == Channel::LowNoise {
            (
                RelayPin::LN_K1_EN_N,
                RelayPin::LN_K1_EN,
                RelayPin::LN_K0_D,
                RelayPin::LN_K0_CP,
            )
        } else {
            (
                RelayPin::HP_K1_EN_N,
                RelayPin::HP_K1_EN,
                RelayPin::HP_K0_D,
                RelayPin::HP_K0_CP,
            )
        };
        Relay {
            mutex,
            k1_en_n,
            k1_en,
            k1_d,
            k1_cp,
        }
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
        mcp.write_pin(self.k1_en.into(), Level::High).unwrap();
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

/// A SharedMcp can provide ownership of one of two sets of relays on Driver.
/// Each set consists of two relays that control the state of a Driver output channel.
/// The relays are controlled by toggeling pins on an MCP23008 I2C IO expander on the Driver board.
/// Both sets use the same MCP23008 chip, hence the SharedMcp.
pub struct SharedMcp<I2C> {
    mutex: spin::Mutex<MCP23008<I2C>>,
}

impl<I2C, E> SharedMcp<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    /// Construct a new shared MCP23008.
    ///
    /// # Args
    /// * `mcp` - The MCP23008 peripheral to share.
    pub fn new(mcp: MCP23008<I2C>) -> Self {
        Self {
            mutex: spin::Mutex::new(mcp),
        }
    }

    /// Allocate a set of relay pins on the MCP23008 and get the [Relay]
    ///
    /// # Args
    /// * `ch` - The Driver channel the relay pins are used for.
    ///
    /// # Returns
    /// An instantiated [Relay] whose ownership can be transferred to other drivers.
    pub fn obtain_relay(&self, ch: Channel) -> Relay<'_, I2C> {
        Relay::new(&self.mutex, ch)
    }
}
