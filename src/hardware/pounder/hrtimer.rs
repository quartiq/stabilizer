///! The HRTimer (High Resolution Timer) is used to generate IO_Update pulses to the Pounder DDS.
use stm32h7xx_hal::{
    self as hal,
    rcc::{rec, CoreClocks, ResetEnable},
};

/// A HRTimer output channel.
#[allow(dead_code)]
pub enum Channel {
    One,
    Two,
}

/// The high resolution timer. Currently, only Timer E is supported.
pub struct HighResTimerE {
    master: hal::stm32::HRTIM_MASTER,
    timer: hal::stm32::HRTIM_TIME,
    common: hal::stm32::HRTIM_COMMON,

    clocks: CoreClocks,
}

impl HighResTimerE {
    /// Construct a new high resolution timer for generating IO_update signals.
    pub fn new(
        timer_regs: hal::stm32::HRTIM_TIME,
        master_regs: hal::stm32::HRTIM_MASTER,
        common_regs: hal::stm32::HRTIM_COMMON,
        clocks: CoreClocks,
        prec: rec::Hrtim,
    ) -> Self {
        prec.reset().enable();

        Self {
            master: master_regs,
            timer: timer_regs,
            common: common_regs,
            clocks,
        }
    }

    /// Configure the timer to operate in single-shot mode.
    ///
    /// # Note
    /// This will configure the timer to generate a single pulse on an output channel. The timer
    /// will only count up once and must be `trigger()`'d after / configured.
    ///
    /// The output will be asserted from `set_offset` to `set_offset` + `set_duration` in the count.
    ///
    /// # Args
    /// * `channel` - The timer output channel to configure.
    /// * `set_duration` - The duration that the output should be asserted for.
    /// * `set_offset` - The first time at which the output should be asserted.
    pub fn configure_single_shot(
        &mut self,
        channel: Channel,
        set_duration: f32,
        set_offset: f32,
    ) {
        // Disable the timer before configuration.
        self.master.mcr.modify(|_, w| w.tecen().clear_bit());

        // Configure the desired timer for single shot mode with set and reset of the specified
        // channel at the desired durations. The HRTIM is on APB2 (D2 domain), and the kernel clock
        // is the APB bus clock.
        let minimum_duration = set_duration + set_offset;

        let source_frequency: u32 = self.clocks.timy_ker_ck().to_Hz();
        let source_cycles =
            (minimum_duration * source_frequency as f32) as u32 + 1;

        // Determine the clock divider, which may be 1, 2, or 4. We will choose a clock divider that
        // allows us the highest resolution per tick, so lower dividers are favored.
        let setting: u8 = if source_cycles < 0xFFDF {
            1
        } else if (source_cycles / 2) < 0xFFDF {
            2
        } else if (source_cycles / 4) < 0xFFDF {
            3
        } else {
            panic!("Unattainable timing parameters!");
        };

        let divider = 1 << (setting - 1);

        // The period register must be greater than or equal to 3 cycles.
        let period = (source_cycles / divider as u32) as u16;
        assert!(period > 2);

        // We now have the prescaler and the period registers. Configure the timer.
        // Note(unsafe): The prescaler is guaranteed to be greater than or equal to 4 (minimum
        // allowed value) due to the addition. The setting is always 1, 2, or 3, which represents
        // all valid values.
        self.timer
            .timecr
            .modify(|_, w| unsafe { w.ck_pscx().bits(setting + 4) });

        // Note(unsafe): The period register is guaranteed to be a 16-bit value, which will fit in
        // this register.
        self.timer.perer.write(|w| unsafe { w.perx().bits(period) });

        // Configure the comparator 1 level.
        let offset = (set_offset * source_frequency as f32) as u16;
        // Note(unsafe): The offset is always a 16-bit value, so is always valid for values >= 3, as
        // specified by the datasheet.
        assert!(offset >= 3);
        self.timer
            .cmp1er
            .write(|w| unsafe { w.cmp1x().bits(offset) });

        // Configure the set/reset signals.
        // Set on compare with CMP1, reset upon reaching PER
        match channel {
            Channel::One => {
                self.timer.sete1r.write(|w| w.cmp1().set_bit());
                self.timer.rste1r.write(|w| w.per().set_bit());
                self.common.oenr.write(|w| w.te1oen().set_bit());
            }
            Channel::Two => {
                self.timer.sete2r.write(|w| w.cmp1().set_bit());
                self.timer.rste2r.write(|w| w.per().set_bit());
                self.common.oenr.write(|w| w.te2oen().set_bit());
            }
        }

        // Enable the timer now that it is configured.
        self.master.mcr.modify(|_, w| w.tecen().set_bit());
    }

    /// Generate a single trigger of the timer to start the output pulse generation.
    pub fn trigger(&mut self) {
        // Generate a reset event to force the timer to start counting.
        self.common.cr2.write(|w| w.terst().set_bit());
    }
}
