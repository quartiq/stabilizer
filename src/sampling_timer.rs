///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;
pub use hal::stm32::tim2::ccmr2_input::CC4S_A;

/// The timer used for managing ADC sampling.
pub struct SamplingTimer {
    timer: hal::timer::Timer<hal::stm32::TIM2>,
    channels: Option<tim2::Channels>,
}

impl SamplingTimer {
    /// Construct the sampling timer.
    pub fn new(mut timer: hal::timer::Timer<hal::stm32::TIM2>) -> Self {
        timer.pause();

        Self {
            timer,
            // Note(unsafe): Once these channels are taken, we guarantee that we do not modify any
            // of the underlying timer channel registers, as ownership of the channels is now
            // provided through the associated channel structures. We additionally guarantee this
            // can only be called once because there is only one Timer2 and this resource takes
            // ownership of it once instantiated.
            channels: unsafe { Some(tim2::Channels::new()) },
        }
    }

    /// Get the timer capture/compare channels.
    pub fn channels(&mut self) -> tim2::Channels {
        self.channels.take().unwrap()
    }

    /// Start the sampling timer.
    pub fn start(&mut self) {
        self.timer.reset_counter();
        self.timer.resume();
    }
}

macro_rules! timer_channel {
    ($name:ident, $TY:ty, ($ccxde:expr, $ccrx:expr, $ccmrx_output:expr, $ccxs:expr)) => {
        pub struct $name {}

        paste::paste! {
            impl $name {
                /// Construct a new timer channel.
                ///
                /// Note(unsafe): This function must only be called once. Once constructed, the
                /// constructee guarantees to never modify the timer channel.
                unsafe fn new() -> Self {
                    Self {}
                }

                /// Allow CH4 to generate DMA requests.
                pub fn listen_dma(&self) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.dier.modify(|_, w| w.[< $ccxde >]().set_bit());
                }

                /// Operate CH2 as an output-compare.
                ///
                /// # Args
                /// * `value` - The value to compare the sampling timer's counter against.
                pub fn to_output_compare(&self, value: u32) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    assert!(value <= regs.arr.read().bits());
                    regs.[< $ccrx >].write(|w| w.ccr().bits(value));
                    regs.[< $ccmrx_output >]()
                        .modify(|_, w| unsafe { w.[< $ccxs >]().bits(0) });
                }
            }
        }
    };
}

pub mod tim2 {
    use stm32h7xx_hal as hal;

    /// The channels representing the timer.
    pub struct Channels {
        pub ch1: Channel1,
        pub ch2: Channel2,
        pub ch3: Channel3,
        pub ch4: Channel4,
    }

    impl Channels {
        /// Construct a new set of channels.
        ///
        /// Note(unsafe): This is only safe to call once.
        pub unsafe fn new() -> Self {
            Self {
                ch1: Channel1::new(),
                ch2: Channel2::new(),
                ch3: Channel3::new(),
                ch4: Channel4::new(),
            }
        }
    }

    timer_channel!(
        Channel1,
        hal::stm32::TIM2,
        (cc1de, ccr1, ccmr1_output, cc1s)
    );
    timer_channel!(
        Channel2,
        hal::stm32::TIM2,
        (cc2de, ccr2, ccmr1_output, cc1s)
    );
    timer_channel!(
        Channel3,
        hal::stm32::TIM2,
        (cc3de, ccr3, ccmr2_output, cc3s)
    );
    timer_channel!(
        Channel4,
        hal::stm32::TIM2,
        (cc4de, ccr4, ccmr2_output, cc4s)
    );
}
