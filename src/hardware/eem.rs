pub use embedded_hal;
pub use stm32h7xx_hal as hal;

pub struct Gpio {
    pub lvds4: hal::gpio::gpiod::PD1<hal::gpio::Input>,
    pub lvds5: hal::gpio::gpiod::PD2<hal::gpio::Input>,
    pub lvds6: hal::gpio::gpiod::PD3<hal::gpio::Output>,
    pub lvds7: hal::gpio::gpiod::PD4<hal::gpio::Output>,
}

pub struct Urukul {
    pub spi: hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled, u32>,
    pub cs: [hal::gpio::ErasedPin<hal::gpio::Output>; 3],
    pub io_update: hal::gpio::ErasedPin<hal::gpio::Output>,
}

impl Urukul {
    pub fn init(&mut self) {}
}

pub enum Eem {
    Gpio(Gpio),
    Urukul(Urukul),
}
