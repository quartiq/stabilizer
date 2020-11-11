///! Stabilizer DAC output control
///!
///! Stabilizer output DACs do not currently rely on DMA requests for generating output.
///! Instead, the DACs utilize an internal queue for storing output codes. A timer then periodically
///! generates an interrupt which triggers an update of the DACs via a write over SPI.
use super::hal;
use heapless::consts;

/// Controller structure for managing the DAC outputs.
pub struct DacOutputs {
    dac0_spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
    dac1_spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
    timer: hal::timer::Timer<hal::stm32::TIM3>,

    // The queue is provided a default length of 32 updates, but this queue can be updated by the
    // end user to be larger if necessary.
    outputs: heapless::spsc::Queue<(u16, u16), consts::U32>,
}

impl DacOutputs {
    /// Construct a new set of DAC output controls
    ///
    /// # Args
    /// * `dac0_spi` - The SPI interface to the DAC0 output.
    /// * `dac1_spi` - The SPI interface to the DAC1 output.
    /// * `timer` - The timer used to generate periodic events for updating the DACs.
    pub fn new(
        mut dac0_spi: hal::spi::Spi<hal::stm32::SPI4, hal::spi::Enabled, u16>,
        mut dac1_spi: hal::spi::Spi<hal::stm32::SPI5, hal::spi::Enabled, u16>,
        mut timer: hal::timer::Timer<hal::stm32::TIM3>,
    ) -> Self {
        // Start the DAC SPI interfaces in infinite transaction mode. CS is configured in
        // auto-suspend mode.
        dac0_spi.inner().cr1.modify(|_, w| w.cstart().started());
        dac1_spi.inner().cr1.modify(|_, w| w.cstart().started());

        dac0_spi.listen(hal::spi::Event::Error);
        dac1_spi.listen(hal::spi::Event::Error);

        // Stop the timer and begin listening for timeouts. Timeouts will be used as a means to
        // generate new DAC outputs.
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

    /// Push a set of new DAC output codes to the internal queue.
    ///
    /// # Note
    /// The earlier DAC output codes will be generated within 1 update cycle of the codes. This is a
    /// fixed latency currently.
    ///
    /// This function will panic if too many codes are written.
    ///
    /// # Args
    /// * `dac0_value` - The value to enqueue for a DAC0 update.
    /// * `dac1_value` - The value to enqueue for a DAC1 update.
    pub fn push(&mut self, dac0_value: u16, dac1_value: u16) {
        self.outputs.enqueue((dac0_value, dac1_value)).unwrap();
        self.timer.resume();
    }

    /// Update the DAC codes with the next set of values in the internal queue.
    ///
    /// # Note
    /// This is intended to be called from the TIM3 update ISR.
    ///
    /// If the last value in the queue is used, the timer is stopped.
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

    /// Write immediate values to the DAC outputs.
    ///
    /// # Note
    /// The DACs will be updated as soon as the SPI transfer completes, which will be nominally
    /// 320nS after this function call.
    ///
    /// # Args
    /// * `dac0_value` - The output code to write to DAC0.
    /// * `dac1_value` - The output code to write to DAC1.
    pub fn write(&mut self, dac0_value: u16, dac1_value: u16) {
        // In order to optimize throughput and minimize latency, the DAC codes are written directly
        // into the SPI TX FIFO. No error checking is conducted. Errors are handled via interrupts
        // instead.
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
