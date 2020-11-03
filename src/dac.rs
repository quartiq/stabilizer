use super::hal;
use heapless::consts;

pub struct DacOutputs {
    dac0_spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
    dac1_spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
    outputs: heapless::spsc::Queue<(u16, u16), consts::U32>,
    timer: hal::timer::Timer<hal::stm32::TIM3>,
}

impl DacOutputs {
    pub fn new(
        dac0_spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
        dac1_spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
        mut timer: hal::timer::Timer<hal::stm32::TIM3>,
    ) -> Self {
        dac0_spi.inner().cr1.modify(|_, w| w.cstart().started());
        dac1_spi.inner().cr1.modify(|_, w| w.cstart().started());
        timer.pause();
        timer.reset_counter();
        timer.clear_irq();
        timer.listen(hal::timer::Event::TimeOut);

        Self {
            dac0_spi,
            dac1_spi,
            outputs: heapless::spsc::Queue::new(),
            timer,
        }
    }

    pub fn push(&mut self, dac0_value: u16, dac1_value: u16) {
        self.outputs.enqueue((dac0_value, dac1_value)).unwrap();
        self.timer.resume();
    }

    pub fn update(&mut self) {
        self.timer.clear_irq();
        match self.outputs.dequeue() {
            Some((dac0, dac1)) => self.write(dac0, dac1),
            None => {
                self.timer.pause();
                self.timer.reset_counter();
                self.timer.clear_irq();
            }
        };
    }

    pub fn write(&mut self, dac0_value: u16, dac1_value: u16) {
        unsafe {
            core::ptr::write_volatile(
                &self.dac0_spi.inner().txdr as *const _ as *mut u16,
                dac0_value,
            );

            core::ptr::write_volatile(
                &self.dac1_spi.inner().txdr as *const _ as *mut u16,
                dac1_value,
            );
        }
    }
}
