use core::ptr;

///! LTC2320 Driver
///!
use super::super::hal::{
    device::QUADSPI,
    gpio::{self, gpiob, gpioc, gpioe},
    hal::blocking::delay::DelayUs,
    prelude::*,
    rcc, stm32,
    xspi::{Qspi, QspiError, QspiMode, XspiExt},
};

pub struct Ltc2320Pins {
    pub spi: (
        gpiob::PB2<gpio::Alternate<9>>,
        gpioe::PE7<gpio::Alternate<10>>,
        gpioe::PE8<gpio::Alternate<10>>,
        gpioe::PE9<gpio::Alternate<10>>,
        gpioe::PE10<gpio::Alternate<10>>,
    ),
    pub cnv: gpioc::PC11<gpio::Output<gpio::PushPull>>,
}

pub struct Ltc2320 {
    qspi: Qspi<QUADSPI>,
    cnv: gpioc::PC11<gpio::Output<gpio::PushPull>>,
}

impl Ltc2320 {
    pub fn new(
        clocks: &rcc::CoreClocks,
        qspi_rec: rcc::rec::Qspi,
        qspi_peripheral: stm32::QUADSPI,
        mut pins: Ltc2320Pins,
    ) -> Self {
        let mut qspi =
            qspi_peripheral.bank2(pins.spi, 3u32.MHz(), clocks, qspi_rec);

        qspi.configure_mode(QspiMode::OneBit).unwrap();

        qspi.is_busy().unwrap();

        qspi.inner_mut().ccr.modify(|_, w| unsafe {
            w.dcyc()
                .bits(0) // set nr dummy cycles to 0 (disable dummy phase)
                .abmode()
                .bits(0) // disable alternate-bytes phase
                .admode()
                .bits(1) // enable address phase
                .imode()
                .bits(0) // disable instruction phase
                .fmode()
                .bits(0b01)
        });

        pins.cnv.set_low();

        Self {
            qspi,
            cnv: pins.cnv,
        }
    }

    pub fn convert(&mut self) -> u8 {
        self.cnv.set_high();
        cortex_m::asm::delay(100);
        self.cnv.set_low();
        cortex_m::asm::delay(500);
        self.cnv.set_low();
        let mut data: [u8; 1] = [0; 1];
        self.qspi.read(0, &mut data).unwrap();
        data[0]
    }
}
