use super::hal;
use heapless::consts;

pub struct Dac0Output {
    outputs: heapless::spsc::Queue<u16, consts::U32>,
    spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
}

impl Dac0Output {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
    ) -> Self {
        spi.inner().cr1.modify(|_, w| w.cstart().started());
        Self {
            spi,
            outputs: heapless::spsc::Queue::new(),
        }
    }

    pub fn push(&mut self, value: u16) {
        self.outputs.enqueue(value).unwrap();
    }

    pub fn update(&mut self) {
        match self.outputs.dequeue() {
            Some(value) => self.write(value),
            None => {}
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
}

impl Dac1Output {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
    ) -> Self {
        spi.inner().cr1.modify(|_, w| w.cstart().started());

        Self {
            spi,
            outputs: heapless::spsc::Queue::new(),
        }
    }

    pub fn push(&mut self, value: u16) {
        self.outputs.enqueue(value).unwrap();
    }

    pub fn update(&mut self) {
        match self.outputs.dequeue() {
            Some(value) => self.write(value),
            None => {}
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
