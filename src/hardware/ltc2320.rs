
///! LTC2320 Driver
///!
use super::hal::{
    gpio::{self, gpiob, gpioc, gpioe},
    prelude::*,
    rcc, stm32,
    xspi::{Qspi, XspiExt},
    device::QUADSPI
};

pub struct Ltc2320Pins {
    pub spi: (
        gpiob::PB2<gpio::Alternate<9>>,
        gpioe::PE7<gpio::Alternate<10>>,
        gpioe::PE8<gpio::Alternate<10>>,
        gpioe::PE9<gpio::Alternate<10>>,
        gpioe::PE10<gpio::Alternate<10>>,
    ),
    pub cnv: gpioc::PC11<gpio::PushPull>,
}

pub struct Ltc2320 {
    qspi: Qspi<QUADSPI>,
    cnv: gpioc::PC11<gpio::PushPull>,
}

impl Ltc2320 {
    pub fn new(
        clocks: &rcc::CoreClocks,
        qspi_rec: rcc::rec::Qspi,
        qspi_peripheral: stm32::QUADSPI,
        pins: Ltc2320Pins,
    ) -> Self {
        let mut qspi =
            qspi_peripheral.bank2(pins.spi, 3u32.MHz(), clocks, qspi_rec);

        Self {
            qspi,
            cnv: pins.cnv,
        }
    }
}
