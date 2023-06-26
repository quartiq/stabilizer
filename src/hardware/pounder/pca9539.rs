use bit_field::BitField;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mcp230xx::{Direction, Level, Mcp23017};

/// Defines errors
#[derive(Debug, Copy, Clone)]
pub enum Error<E> {
    /// Underlying bus error
    BusError(E),
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::BusError(error)
    }
}

/// PCA9539/TCA9539 is a 16-pin I2C I/O Expander with I2C Interface.
#[derive(Clone, Copy, Debug)]
pub struct Pca9539<I2C> {
    i2c: I2C,
    address: u8,
}

///
impl<I2C, E> Pca9539<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
{
    const DEFAULT_ADDRESS: u8 = 0x74;

    /// Creates an expander with the default configuration.
    pub fn new_default(i2c: I2C) -> Result<Self, Error<E>> {
        Self::new(i2c, Self::DEFAULT_ADDRESS)
    }

    /// Creates an expander with specific address.
    pub fn new(i2c: I2C, address: u8) -> Result<Self, Error<E>> {
        Ok(Self { i2c, address })
    }

    /// Return the I2C address
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Read an 8 bit register
    pub fn read(&mut self, addr: u8) -> Result<u8, E> {
        let mut data = [0u8];
        self.i2c.write_read(self.address, &[addr], &mut data)?;
        Ok(data[0])
    }

    /// Write an 8 bit register
    pub fn write(&mut self, addr: u8, data: u8) -> Result<(), E> {
        self.i2c.write(self.address, &[addr, data])
    }

    /// Get a single bit in a register
    pub fn bit(&mut self, addr: u8, bit: usize) -> Result<bool, E> {
        let data = self.read(addr)?;
        Ok(data.get_bit(bit))
    }

    /// Set a single bit in a register
    pub fn set_bit(
        &mut self,
        addr: u8,
        bit: usize,
        value: bool,
    ) -> Result<(), E> {
        let mut data = self.read(addr)?;
        data.set_bit(bit, value);
        self.write(addr, data)
    }

    pub fn set_level(&mut self, pin: Mcp23017, level: Level) -> Result<(), E> {
        let pin = pin as u8;
        self.set_bit(0x02 | (pin >> 3), pin as usize & 7, level == Level::High)
    }

    pub fn set_direction(
        &mut self,
        pin: Mcp23017,
        direction: Direction,
    ) -> Result<(), E> {
        let pin = pin as u8;
        self.set_bit(
            0x06 | (pin >> 3),
            pin as usize & 7,
            direction == Direction::Input,
        )
    }
}
