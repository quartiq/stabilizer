///! Driver relays driver
///!
///! There are 2 relays at the driver output:
///!    - one that shorts the output to ground
///!    - one that connects the current source/sink to the output
///!
///! See [Relay] documentation for details about relay control.
///!
///! The relays are controlled via an MCP23008 I2C io expander.
use core::fmt::Debug;
use embedded_hal::blocking::i2c::{Write, WriteRead};
use mcp230xx::{Level, Mcp23008, Mcp230xx};

use super::Channel;

// Driver output relays pins.
#[allow(non_camel_case_types)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum RelayPin {
    LN_K1_EN_N,
    LN_K1_EN,
    LN_K0_D,
    LN_K0_CP,
    HP_K1_EN_N,
    HP_K1_EN,
    HP_K0_D,
    HP_K0_CP,
}

impl From<RelayPin> for Mcp23008 {
    fn from(pin: RelayPin) -> Self {
        match pin {
            RelayPin::LN_K0_CP => Self::P0,
            RelayPin::LN_K0_D => Self::P1,
            RelayPin::LN_K1_EN => Self::P2,
            RelayPin::LN_K1_EN_N => Self::P3,
            RelayPin::HP_K0_CP => Self::P4,
            RelayPin::HP_K0_D => Self::P5,
            RelayPin::HP_K1_EN => Self::P6,
            RelayPin::HP_K1_EN_N => Self::P7,
        }
    }
}

/// Driver output [Relay] type that controlls the set of two relays that is present on both of
/// the driver output channels.
/// [Relay] contains a mutex of a shared MCP23008, an I2C io expander chip with 8 GPIOs.
/// 4 of those GPIOs are used to controll one set of relays (aka one [Relay]):
///     - `k1_en_n` and `k1_en` differentially control the state of K1
///     - `k0_d` is that data input of a flipflop which controls the state of K0
///     - `k0_cp` is the clock input if the flipflop. Note that there is additional circuitry
///        which ensures this clock input cannot be driven if K1 is not in the correct state.
///        See Driver schematic for exact details.
pub struct Relay<I2C: WriteRead + Write + 'static> {
    gpio: &'static spin::Mutex<Mcp230xx<I2C, Mcp23008>>,
    k1_en_n: RelayPin,
    k1_en: RelayPin,
    k0_d: RelayPin,
    k0_cp: RelayPin,
}

impl<I2C, E> Relay<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // RL2B Operating Time and Release Time
    pub const K0_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(30);
    // RL1B Operating Time and Release Time
    pub const K1_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(30);

    /// Construct a new [Relay].
    ///
    /// # Args
    /// * `gpio`   - mutex of a shared MCP23008
    /// * `ccdr`    - core clocks to construct a `delay`
    /// * `channel` - Driver channel to construct the [Relay] for
    pub fn new(
        gpio: &'static spin::Mutex<Mcp230xx<I2C, Mcp23008>>,
        channel: Channel,
    ) -> Self {
        let (k1_en_n, k1_en, k0_d, k0_cp) = if channel == Channel::LowNoise {
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
        let mut mcp = gpio.try_lock().unwrap();

        // don't perform I2C transactions to MCP23008 on the headboard if it is intentionally not connected
        #[cfg(feature = "ai_artiq_laser_module")]
        {
            // set all pins to be outputs
            mcp.write(mcp230xx::Register::IODIR.into(), 0).unwrap();
            // set GPIOs to default position
            mcp.set_gpio(k1_en.into(), Level::Low).unwrap();
            mcp.set_gpio(k1_en_n.into(), Level::High).unwrap();
            mcp.set_gpio(k0_d.into(), Level::Low).unwrap();
            // toggle flipflop once to set a known output
            mcp.set_gpio(k0_cp.into(), Level::Low).unwrap();
            mcp.set_gpio(k0_cp.into(), Level::High).unwrap();
        }

        Relay {
            gpio,
            k1_en_n,
            k1_en,
            k0_d,
            k0_cp,
        }
    }

    // set K0 to upper position (note that "upper" and "lower" refer to the schematic)
    pub fn engage_k0(&mut self) {
        let mut mcp = self.gpio.try_lock().unwrap();
        // set flipflop data pin
        mcp.set_gpio(self.k0_d.into(), Level::High).unwrap();
        // set flipflop clock input low to prepare rising edge
        mcp.set_gpio(self.k0_cp.into(), Level::Low).unwrap();
        // set flipflop clock input high to generate rising edge
        mcp.set_gpio(self.k0_cp.into(), Level::High).unwrap();
    }

    // set K0 to lower position
    pub fn disengage_k0(&mut self) {
        let mut mcp = self.gpio.try_lock().unwrap();
        mcp.set_gpio(self.k0_d.into(), Level::Low).unwrap();
        mcp.set_gpio(self.k0_cp.into(), Level::Low).unwrap();
        mcp.set_gpio(self.k0_cp.into(), Level::High).unwrap();
    }

    // set K1 to upper position
    pub fn disengage_k1(&mut self) {
        let mut mcp = self.gpio.try_lock().unwrap();
        // set en high and en _n low in order to engage K1
        mcp.set_gpio(self.k1_en.into(), Level::Low).unwrap();
        mcp.set_gpio(self.k1_en_n.into(), Level::High).unwrap();
    }

    // set K1 to lower position and output current to zero
    pub fn engage_k1(&mut self) {
        let mut mcp = self.gpio.try_lock().unwrap();
        mcp.set_gpio(self.k1_en.into(), Level::High).unwrap();
        mcp.set_gpio(self.k1_en_n.into(), Level::Low).unwrap();
    }
}
