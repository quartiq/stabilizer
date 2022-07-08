///! LTC2320 Driver
///!
///! QSPI bug (2.4.3):
///! https://www.st.com/resource/en/errata_sheet/es0392-stm32h742xig-and-stm32h743xig-device-limitations-stmicroelectronics.pdf
use super::super::hal::{
    device::QUADSPI,
    gpio::{self, gpiob, gpioc, gpioe},
    hal::blocking::delay::DelayUs,
    pac,
    prelude::*,
    rcc, stm32,
    time::NanoSeconds,
    timer::{self, Timer},
    xspi::{Qspi, QspiError, QspiMode, XspiExt},
};
use core::ptr;

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
    timer: Timer<stm32::TIM7>,
}

impl Ltc2320 {
    const N_BYTES: usize = 17; // Number of bytes to be transfered (one byte extra to account for address phase)
    const TCONV: u32 = 450; // minimum conversion time according to datasheet
    pub fn new(
        clocks: &rcc::CoreClocks,
        qspi_rec: rcc::rec::Qspi,
        qspi_peripheral: stm32::QUADSPI,
        timer_rec: rcc::rec::Tim7,
        timer_peripheral: stm32::TIM7,
        mut pins: Ltc2320Pins,
    ) -> Self {
        let mut qspi =
            qspi_peripheral.bank2(pins.spi, 3u32.MHz(), clocks, qspi_rec);
        qspi.configure_mode(QspiMode::OneBit).unwrap();
        qspi.is_busy().unwrap(); // panic if qspi busy
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

        qspi.inner_mut().cr.modify(|_, w| unsafe {
            w.ftie()
                .bit(true) // enable fifo threshold interrupt
                .fthres()
                .bits(Ltc2320::N_BYTES as u8) // set fifo threshold to number of bytes
        });

        pins.cnv.set_low();
        let mut timer = timer_peripheral.timer(400.MHz(), timer_rec, clocks);
        timer.pause();
        timer.reset_counter();
        // Todo: Understand how interrupt prio works with RTIC
        unsafe {
            pac::NVIC::unmask(pac::interrupt::TIM7);
            pac::NVIC::unmask(pac::interrupt::QUADSPI)
        }
        timer.listen(timer::Event::TimeOut);
        Self {
            qspi,
            cnv: pins.cnv,
            timer,
        }
    }

    /// set nCNV low and setup timer to wait for 450 ns
    pub fn start_conversion(&mut self) {
        self.cnv.set_low();
        self.timer
            .start(NanoSeconds::from_ticks(Ltc2320::TCONV).into_rate())
    }

    /// start qspi read of ADC data
    pub fn start_readout(&mut self) {
        self.qspi.begin_read(0, Ltc2320::N_BYTES).unwrap();
    }

    /// set nCNV high, readout qspi buffer, bitshuffle
    pub fn retrieve_data(&mut self, data: &mut [u16]) {
        self.cnv.set_high(); // TCNVH: has to be high for at least 30 ns (8 cycles)

        let mut buffer = [0u8; Ltc2320::N_BYTES];

        // Read data from the FIFO in a byte-wise manner.
        unsafe {
            for location in &mut buffer {
                *location = ptr::read_volatile(
                    &self.qspi.inner().dr as *const _ as *const u8,
                );
            }
        }
        // Todo: This is temporary. Unshuffle bits here.
        for (i, sample) in data.iter_mut().enumerate() {
            *sample =
                ((buffer[2 * i + 1] as u16) << 8) | buffer[2 * i + 2] as u16;
        }
    }
}
