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
        self.read(&mut data).unwrap();
        data[0]
    }

    pub fn read(&mut self, dest: &mut [u8]) -> Result<(), QspiError> {
        self.qspi.is_busy()?;

        // Clear the transfer complete flag.
        self.qspi.inner_mut().fcr.write(|w| w.ctcf().set_bit());

        // Write the length that should be read.
        self.qspi
            .inner_mut()
            .dlr
            .write(|w| unsafe { w.dl().bits(dest.len() as u32 - 1) });

        // Read data from the FIFO in a byte-wise manner.
        let mut dummy: [u8; 1] = [0; 1];
        unsafe {
            for location in &mut dummy {
                *location = ptr::read_volatile(
                    &self.qspi.inner().dr as *const _ as *const u8,
                );
            }
        }
        log::info!("pos1 {:#b}", self.qspi.inner().sr.read().bits());

       
        self.qspi.inner_mut().ccr.modify(|_, w| unsafe {
            w.admode().bits(0b0) // disable adress mode
        });

        // Write the address to force the read to start.
        // self.qspi.inner_mut().ar.write(|w| unsafe { w.address().bits(0x00) });

        cortex_m::asm::delay(1000000);
        log::info!("pos2 {:#b}", self.qspi.inner().sr.read().bits());

        // Write the address to force the read to start.
        // self.qspi.inner_mut().ar.write(|w| unsafe { w.address().bits(0x01) });

        // self.qspi.is_busy()?;

        // // set fmode to indirect read
        // self.qspi.inner_mut().ccr.modify(|_, w| unsafe {
        //     w.instruction().bits(0b1) // set some inst
        // });

        // Wait for the transaction to complete
        // while self.qspi.inner().sr.read().tcf().bit_is_clear() {
        //     log::info!("{:#b}", self.qspi.inner().sr.read().bits())
        // }

        // // clear fmode
        // self.qspi.inner_mut().ccr.modify(|_, w| unsafe {
        //     w.fmode().bits(0b00) // set to indirect write mode
        // });

        // Check for underflow on the FIFO.
        if (self.qspi.inner().sr.read().flevel().bits() as usize) < dest.len() {
            return Err(QspiError::Underflow);
        }

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in dest {
                *location = ptr::read_volatile(
                    &self.qspi.inner().dr as *const _ as *const u8,
                );
            }
        }
        Ok(())
    }
}
