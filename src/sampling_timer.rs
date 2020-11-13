///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;

pub use hal::stm32::tim2::ccmr2_input::CC4S_A;

/// The timer used for managing ADC sampling.
pub struct SamplingTimer {
    timer: hal::timer::Timer<hal::stm32::TIM2>,
    channels: Option<TimerChannels>,
}

impl SamplingTimer {
    /// Construct the sampling timer.
    pub fn new(mut timer: hal::timer::Timer<hal::stm32::TIM2>) -> Self {
        timer.pause();

        Self {
            timer,
            channels: Some(TimerChannels::new()),
        }
    }

    /// Get the timer capture/compare channels.
    pub fn channels(&mut self) -> TimerChannels {
        self.channels.take().unwrap()
    }

    /// Start the sampling timer.
    pub fn start(&mut self) {
        self.timer.reset_counter();
        self.timer.resume();
    }
}

/// The capture/compare channels for the sampling timer.
///
/// # Note
/// This should not be instantiated directly.
pub struct TimerChannels {
    pub ch1: Timer2Channel1,
    pub ch2: Timer2Channel2,
    pub ch3: Timer2Channel3,
    pub ch4: Timer2Channel4,
}

impl TimerChannels {
    fn new() -> Self {
        Self {
            ch1: Timer2Channel1 {},
            ch2: Timer2Channel2 {},
            ch3: Timer2Channel3 {},
            ch4: Timer2Channel4 {},
        }
    }
}

/// Representation of CH1 of TIM2.
pub struct Timer2Channel1 {}

impl Timer2Channel1 {
    /// Allow CH1 to generate DMA requests.
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc1de().set_bit());
    }

    /// Operate CH1 as an output-compare.
    ///
    /// # Args
    /// * `value` - The value to compare the sampling timer's counter against.
    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr1.write(|w| w.ccr().bits(value));
        regs.ccmr1_output()
            .modify(|_, w| unsafe { w.cc1s().bits(0) });
    }
}

/// Representation of CH2 of TIM2.
pub struct Timer2Channel2 {}

impl Timer2Channel2 {
    /// Allow CH2 to generate DMA requests.
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc2de().set_bit());
    }

    /// Operate CH2 as an output-compare.
    ///
    /// # Args
    /// * `value` - The value to compare the sampling timer's counter against.
    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr2.write(|w| w.ccr().bits(value));
        regs.ccmr1_output()
            .modify(|_, w| unsafe { w.cc2s().bits(0) });
    }
}

/// Representation of CH3 of TIM2.
pub struct Timer2Channel3 {}

impl Timer2Channel3 {
    /// Allow CH4 to generate DMA requests.
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc3de().set_bit());
    }

    /// Operate CH2 as an output-compare.
    ///
    /// # Args
    /// * `value` - The value to compare the sampling timer's counter against.
    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr3.write(|w| w.ccr().bits(value));
        regs.ccmr2_output()
            .modify(|_, w| unsafe { w.cc3s().bits(0) });
    }
}

/// Representation of CH4 of TIM2.
pub struct Timer2Channel4 {}

impl Timer2Channel4 {
    /// Allow CH4 to generate DMA requests.
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc4de().set_bit());
    }

    /// Operate CH2 as an output-compare.
    ///
    /// # Args
    /// * `value` - The value to compare the sampling timer's counter against.
    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr4.write(|w| w.ccr().bits(value));
        regs.ccmr2_output()
            .modify(|_, w| unsafe { w.cc4s().bits(0) });
    }
}
