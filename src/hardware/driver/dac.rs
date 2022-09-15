use embedded_hal::blocking::spi::Transfer;
///! Driver DAC11001 driver
use stm32h7xx_hal::gpio;

pub struct Dac<SPI: Transfer<u8>> {
    spi: SPI,
    sync_n: gpio::ErasedPin<gpio::Output>,
}

impl<SPI> Dac<SPI>
where
    SPI: Transfer<u8>,
{
    pub fn new(spi: SPI, sync_n: gpio::ErasedPin<gpio::Output>) -> Self {
        Dac { spi, sync_n }
    }

    // pub fn calibrate()
}
