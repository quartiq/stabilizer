///! LTC2320 Driver
///!
///! QSPI bug (2.4.3):
///! https://www.st.com/resource/en/errata_sheet/es0392-stm32h742xig-and-stm32h743xig-device-limitations-stmicroelectronics.pdf
///!
///! This driver is intended to be used in the following manner:
///! 1. Trigger a new LTC2320 conversion with start_conversion(). This sets nCNV low and waits until TCONV has passed
///!    and starts the qspi readout.
///! 2. Call handle_transfer_done() in the QSPI ISR to retrieve the ADC data and set nCNV high again.
///!
///! Only works under the following condition:
///! Conversions are not restarted faster than (T_readout + TCONV + TCNVH + readout/irq CPU overhead).
///!
///! The driver will always return an error if conversions are re-started too quickly and timings are not met.
///! The user has to ensure that the QSPI speed and number of lanes are adequate for the sampling period.
use super::super::hal::{
    device::QUADSPI,
    gpio::{self, gpiob, gpioc, gpioe},
    rcc, stm32,
    time::MegaHertz,
    xspi::{Event, Qspi, QspiError, QspiMode, XspiExt},
};
use core::ptr;

#[derive(Copy, Clone, Debug)]
pub enum Error<E> {
    TimerBusyError,
    QspiError(E),
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Error<E> {
        Error::QspiError(err)
    }
}

pub struct Ltc2320Pins {
    #[allow(clippy::complexity)]
    pub qspi: (
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
    // Number of cycles to wait for an equivalent time of TCONV.
    // The asm-delay crate can only do ms and us delays but TCONV < 1 us.
    tconv_delay_cycles: u32,
}

impl Ltc2320 {
    const N_BYTES: usize = 16; // Number of bytes to be transfered.
    const TCONV: f32 = 450e-9; // minimum conversion time in seconds according to datasheet
    /// The QSPI frequency for LTC2320 readout.
    const DRIVER_QSPI_FREQUENCY: MegaHertz = MegaHertz::MHz(1); // Use slow 1 MHz for now. This will change.
    pub fn new(
        clocks: &rcc::CoreClocks,
        qspi_rec: rcc::rec::Qspi,
        qspi_peripheral: stm32::QUADSPI,
        mut pins: Ltc2320Pins,
    ) -> Self {
        let mut qspi = qspi_peripheral.bank2(
            pins.qspi,
            Ltc2320::DRIVER_QSPI_FREQUENCY.convert(),
            clocks,
            qspi_rec,
        );
        qspi.configure_mode(QspiMode::OneBit).unwrap();
        qspi.is_busy().unwrap(); // panic if qspi busy
                                 // qspi.setup_extended() could do the same but is a private function and can therefore not be used.
                                 // qspi.begin_read_extended() uses setup_extended() but also starts the read right away.
        qspi.inner_mut().ccr.modify(|_, w| unsafe {
            w.dcyc()
                .bits(0) // set nr dummy cycles to 0 (disable dummy phase)
                .abmode()
                .bits(0) // disable alternate-bytes phase
                .admode()
                .bits(1) // enable address phase
                .adsize()
                .bits(0b01) // set to 16 bit address to gain readback data alignment
                .imode()
                .bits(0) // disable instruction phase
                .fmode()
                .bits(0b01) // indirect read mode
        });
        qspi.listen(Event::Complete);
        pins.cnv.set_high();
        // calculate the number of cycles equivalent to TCONV
        let tconv_delay_cycles =
            (clocks.c_ck().to_Hz() as f32 * Ltc2320::TCONV) as u32;
        Self {
            qspi,
            cnv: pins.cnv,
            tconv_delay_cycles,
        }
    }

    /// Set nCNV low, wait TCONV and start QSPI transfer.
    pub fn start_conversion(&mut self) -> Result<(), Error<QspiError>> {
        self.cnv.set_low();
        cortex_m::asm::delay(self.tconv_delay_cycles);
        self.qspi
            .begin_read(0, Ltc2320::N_BYTES)
            .map_err(|err| err.into())
    }

    /// Set nCNV high, readout QSPI buffer, bitshuffle.
    pub fn handle_transfer_done(&mut self, data: &mut [u16]) {
        // TCNVH: has to be high for at least 30 ns.
        // Given at normal MCU clockspeeds since this function takes > 10 cycles.
        self.cnv.set_high();
        self.qspi.inner_mut().fcr.modify(
            |_, w| w.ctcf().bit(true), // clear transfer complete flag
        );
        // Read data from the FIFO.
        // Should ultimately be 32 bit reads into a bytemuck::cast_slice_mut(data).
        unsafe {
            for location in data {
                *location = ptr::read_volatile(
                    &self.qspi.inner().dr as *const _ as *const u16,
                );
            }
        }
    }
}
