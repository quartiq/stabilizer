use super::hal;
use heapless::consts;

pub struct Dac0Output {
    outputs: heapless::spsc::Queue<u16, consts::U32>,
    spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
    timer: hal::timer::Timer<hal::stm32::TIM3>,
}

impl Dac0Output {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
        mut timer: hal::timer::Timer<hal::stm32::TIM3>,
    ) -> Self {
        spi.inner().cr1.modify(|_, w| w.cstart().started());
        timer.pause();
        timer.reset_counter();
        timer.clear_irq();
        timer.listen(hal::timer::Event::TimeOut);

        Self {
            spi,
            outputs: heapless::spsc::Queue::new(),
            timer,
        }
    }

    pub fn push(&mut self, value: u16) {
        self.outputs.enqueue(value).unwrap();
        self.timer.resume();
    }

    pub fn update(&mut self) {
        self.timer.clear_irq();
        match self.outputs.dequeue() {
            Some(value) => self.write(value),
            None => self.timer.pause(),
        }
    }

    pub fn write(&mut self, value: u16) {
        unsafe {
            core::ptr::write_volatile(
                &self.spi.inner().txdr as *const _ as *mut u16,
                value,
            );
        }
    }
}

pub struct Dac1Output {
    outputs: heapless::spsc::Queue<u16, consts::U32>,
    spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
    timer: hal::timer::Timer<hal::stm32::TIM4>,
}

impl Dac1Output {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
        mut timer: hal::timer::Timer<hal::stm32::TIM4>,
    ) -> Self {
        spi.inner().cr1.modify(|_, w| w.cstart().started());
        timer.pause();
        timer.reset_counter();
        timer.clear_irq();
        timer.listen(hal::timer::Event::TimeOut);

        Self {
            spi,
            outputs: heapless::spsc::Queue::new(),
            timer,
        }
    }

    pub fn push(&mut self, value: u16) {
        self.outputs.enqueue(value).unwrap();
        self.timer.resume();
    }

    pub fn update(&mut self) {
        self.timer.clear_irq();
        match self.outputs.dequeue() {
            Some(value) => self.write(value),
            None => self.timer.pause(),
        }
    }

    pub fn write(&mut self, value: u16) {
        unsafe {
            core::ptr::write_volatile(
                &self.spi.inner().txdr as *const _ as *mut u16,
                value,
            );
        }
    }
}
