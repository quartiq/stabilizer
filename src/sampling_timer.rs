use super::hal;

use hal::dma::{dma::DMAReq, traits::TargetAddress, PeripheralToMemory};
pub use hal::stm32::tim2::ccmr2_input::CC4S_A;

pub struct SamplingTimer {
    timer: hal::timer::Timer<hal::stm32::TIM2>,
    channels: Option<TimerChannels>,
}

impl SamplingTimer {
    pub fn new(mut timer: hal::timer::Timer<hal::stm32::TIM2>) -> Self {
        timer.pause();

        Self {
            timer,
            channels: Some(TimerChannels::new()),
        }
    }

    pub fn channels(&mut self) -> TimerChannels {
        self.channels.take().unwrap()
    }

    pub fn start(&mut self) {
        self.timer.reset_counter();
        self.timer.resume();
    }
}

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

pub struct Timer2Channel1 {}

impl Timer2Channel1 {
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc1de().set_bit());
    }

    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr1.write(|w| w.ccr().bits(value));
        regs.ccmr1_output()
            .modify(|_, w| unsafe { w.cc1s().bits(0) });
    }
}

pub struct Timer2Channel2 {}

impl Timer2Channel2 {
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc2de().set_bit());
    }

    pub fn to_output_compare(&self, value: u32) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        assert!(value <= regs.arr.read().bits());
        regs.ccr2.write(|w| w.ccr().bits(value));
        regs.ccmr1_output()
            .modify(|_, w| unsafe { w.cc2s().bits(0) });
    }
}

pub struct Timer2Channel3 {}

pub struct Timer2Channel4 {}

unsafe impl TargetAddress<PeripheralToMemory> for Timer2Channel4 {
    type MemSize = u16;

    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_CH4 as u8);

    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        &regs.dmar as *const _ as u32
    }
}

impl Timer2Channel4 {
    pub fn listen_dma(&self) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.dier.modify(|_, w| w.cc4de().set_bit());
    }

    pub fn to_input_capture(&self, trig: CC4S_A) {
        let regs = unsafe { &*hal::stm32::TIM2::ptr() };
        regs.ccmr2_input().modify(|_, w| w.cc4s().variant(trig));

        // Update the DMA control burst regs to point to CCR4.
        regs.dcr
            .modify(|_, w| unsafe { w.dbl().bits(1).dba().bits(16) });
    }
}
