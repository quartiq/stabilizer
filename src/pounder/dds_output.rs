use super::QspiInterface;
use crate::hrtimer::HighResTimerE;
use stm32h7xx_hal as hal;

pub struct DdsOutput {
    _qspi: QspiInterface,
    io_update_trigger: HighResTimerE,
}

impl DdsOutput {
    pub fn new(_qspi: QspiInterface, io_update_trigger: HighResTimerE) -> Self {
        Self {
            _qspi,
            io_update_trigger,
        }
    }

    pub fn write_profile(&mut self, profile: [u32; 4]) {
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
