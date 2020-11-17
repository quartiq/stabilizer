use crate::hrtimer::HighResTimerE;
use stm32h7xx_hal as hal;

pub struct DdsOutput {
    profiles: heapless::spsc::Queue<[u32; 4], heapless::consts::U32>,
    update_timer: hal::timer::Timer<hal::stm32::TIM3>,
    io_update_trigger: HighResTimerE,
}

impl DdsOutput {
    pub fn new(
        mut timer: hal::timer::Timer<hal::stm32::TIM3>,
        io_update_trigger: HighResTimerE,
    ) -> Self {
        timer.pause();
        timer.reset_counter();
        timer.listen(hal::timer::Event::TimeOut);

        Self {
            update_timer: timer,
            io_update_trigger,
            profiles: heapless::spsc::Queue::new(),
        }
    }

    pub fn update_handler(&mut self) {
        match self.profiles.dequeue() {
            Some(profile) => self.write_profile(profile),
            None => self.update_timer.pause(),
        }
    }

    pub fn push(&mut self, profile: [u32; 4]) {
        self.profiles.enqueue(profile).unwrap();
        self.update_timer.resume();
    }

    fn write_profile(&mut self, profile: [u32; 4]) {
        let regs = unsafe { &*hal::stm32::QUADSPI::ptr() };
        unsafe {
            core::ptr::write_volatile(
                &regs.dr as *const _ as *mut u32,
                profile[0],
            );
            core::ptr::write_volatile(
                &regs.dr as *const _ as *mut u32,
                profile[1],
            );
            core::ptr::write_volatile(
                &regs.dr as *const _ as *mut u32,
                profile[2],
            );
            core::ptr::write_volatile(
                &regs.dr as *const _ as *mut u32,
                profile[3],
            );
        }

        // Trigger the IO_update signal generating timer to asynchronous create the IO_Update pulse.
        self.io_update_trigger.trigger();
    }
}
