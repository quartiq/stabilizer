///! Driver DAC11001 driver
/// 
use core::fmt::Debug;


pub struct Dac11001 {
    
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
        // set GPIOs to default position
        mcp.set_gpio(k1_en.into(), Level::Low).unwrap();
        mcp.set_gpio(k1_en_n.into(), Level::High).unwrap();
        mcp.set_gpio(k0_d.into(), Level::Low).unwrap();
        // toggle flipflop once to set a known output
        mcp.set_gpio(k0_cp.into(), Level::Low).unwrap();
        mcp.set_gpio(k0_cp.into(), Level::High).unwrap();

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
        mcp.set_gpio(self.k0_d.into(), Level::High).unwrap();
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
