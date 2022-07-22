use core::fmt::Debug;

///! Driver relays driver
///!
///! There are 2 relays at the driver output:
///!    - one that shorts the output to ground
///!    - one that connects the current source/sink to the output
///!
///! The relays are controlled via an I2C io-expander.
use embedded_hal::blocking::i2c::{Write, WriteRead};

pub struct Relays<I2C: WriteRead + Write> {
    mcp23017: mcp23017::MCP23017<I2C>,
}

impl<I2C, E> Relays<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    pub fn new(i2c: I2C) -> Self {
        let mcp23017 = mcp23017::MCP23017::default(i2c).unwrap();
        Relays { mcp23017 }
    }
}
