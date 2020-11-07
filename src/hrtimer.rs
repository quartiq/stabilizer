use crate::hal;
use hal::rcc::{CoreClocks, ResetEnable, rec};

pub enum Channel {
    One,
    Two,
}

struct HighResTimerE {
    master: hal::stm32::HRTIM_MASTER,
    timer: hal::stm32::HRTIM_TIME,
    common: hal::stm32::HRTIM_COMMON,

    clocks: CoreClocks,
}

impl HighResTimerE {
    pub fn new(timer_regs: hal::stm32::HRTIM_TIME, clocks: CoreClocks, prec: rec::Hrtim) -> Self {
        let master = unsafe { &*hal::stm32::HRTIM_MASTER::ptr() };
        let common = unsafe { &*hal::stm32::HRTIM_COMMON::ptr() };
        prec.reset().enable();

        Self { master, timer: timer_regs, common, clocks }
    }

    pub fn configure_single_shot(&mut self, channel: Channel, set_duration: f32, set_offset: f32) {
        // Disable the timer before configuration.
        self.master.mcr.modify(|_, w| w.tecen().clear_bit());

        // Configure the desired timer for single shot mode with set and reset of the specified
        // channel at the desired durations. The HRTIM is on APB2 (D2 domain), and the kernel clock
        // is the APB bus clock.
        let minimum_duration = set_duration + set_offset;

        let source_frequency = self.clocks.timy_ker_ck;
        let source_cycles = minimum_duration * source_frequency;

        // Determine the clock divider, which may be 1, 2, or 4. We will choose a clock divider that
        // allows us the highest resolution per tick, so lower dividers are favored.
        let divider = if source_cycles < 0xFFDF {
            1
        } else if (source_cycles / 2) < 0xFFDF {
            2
        } else if (source_cycles / 4) < 0xFFDF {
            4
        } else {
            panic!("Unattainable timing parameters!");
        };

        // The period register must be greater than or equal to 3 cycles.
        assert!((source_cycles / divider) > 2);

        // We now have the prescaler and the period registers. Configure the timer.
        self.timer.timecr.modify(|_, w| unsafe{w.ck_pscx().bits(divider)})
        self.timer.perer.write(|w| unsafe{w.per().bits(source_cycles / divider)});

        // Configure the comparator 1 level.
        self.timer.cmpe1r.write(|w| unsafe{w.cmp1().bits(set_offset * source_frequency)});

        // Configure the set/reset signals.
        // Set on compare with CMP1, reset upon reaching PER
        match channel {
            Channel::One => {
                self.timer.sete1r().write(|w| w.cmp1().set_bit());
                self.timer.resete1r().write(|w| w.per().set_bit());
            },
            Channel::Two => {
                self.timer.sete2r().write(|w| w.cmp1().set_bit());
                self.timer.resete2r().write(|w| w.per().set_bit());
            },
        }

        // Enable the timer now that it is configured.
        self.master.mcr.modify(|_, w| w.tecen().set_bit());
    }

    pub fn trigger(&mut self) {
        // Generate a reset event to force the timer to start counting.
        self.common.cr2.write(|w| w.terst().set_bit());
    }
}
