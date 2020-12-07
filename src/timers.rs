///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;

macro_rules! timer_channels {
    ($name:ident, $TY:ident) => {
        paste::paste! {

            /// The timer used for managing ADC sampling.
            pub struct $name {
                timer: hal::timer::Timer<hal::stm32::[< $TY >]>,
                channels: Option<[< $TY:lower >]::Channels>,
            }

            impl $name {
                /// Construct the sampling timer.
                pub fn new(mut timer: hal::timer::Timer<hal::stm32::[< $TY>]>) -> Self {
                    timer.pause();

                    Self {
                        timer,
                        // Note(unsafe): Once these channels are taken, we guarantee that we do not modify any
                        // of the underlying timer channel registers, as ownership of the channels is now
                        // provided through the associated channel structures. We additionally guarantee this
                        // can only be called once because there is only one Timer2 and this resource takes
                        // ownership of it once instantiated.
                        channels: unsafe { Some([< $TY:lower >]::Channels::new()) },
                    }
                }

                /// Get the timer capture/compare channels.
                pub fn channels(&mut self) -> [< $TY:lower >]::Channels {
                    self.channels.take().unwrap()
                }

                #[allow(dead_code)]
                pub fn get_prescaler(&self) -> u16 {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.psc.read().psc().bits() + 1
                }

                #[allow(dead_code)]
                pub fn set_prescaler(&mut self, prescaler: u16) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    assert!(prescaler >= 1);
                    regs.psc.write(|w| w.psc().bits(prescaler - 1));
                }

                #[allow(dead_code)]
                pub fn get_period(&self) -> u32 {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.arr.read().arr().bits()
                }

                #[allow(dead_code)]
                pub fn set_period(&mut self, period: u32) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.arr.write(|w| w.arr().bits(period));
                }

                /// Start the timer.
                pub fn start(mut self) {
                    // Force a refresh of the frequency settings.
                    self.timer.apply_freq();

                    self.timer.reset_counter();
                    self.timer.resume();
                }
            }

            pub mod [< $TY:lower >] {
                pub use hal::stm32::tim2::ccmr1_input::{CC1S_A, CC2S_A};
                pub use hal::stm32::tim2::ccmr2_input::{CC3S_A, CC4S_A};

                use stm32h7xx_hal as hal;
                use hal::dma::{traits::TargetAddress, PeripheralToMemory, dma::DMAReq};
                use hal::stm32::$TY;

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

                timer_channels!(1, $TY, ccmr1);
                timer_channels!(2, $TY, ccmr1);
                timer_channels!(3, $TY, ccmr2);
                timer_channels!(4, $TY, ccmr2);
            }
        }
    };

    ($index:expr, $TY:ty, $ccmrx:expr) => {
        paste::paste! {
            pub struct [< Channel $index >] {}

            pub struct [< Channel $index InputCapture>] {}

            impl [< Channel $index >] {
                /// Construct a new timer channel.
                ///
                /// Note(unsafe): This function must only be called once. Once constructed, the
                /// constructee guarantees to never modify the timer channel.
                unsafe fn new() -> Self {
                    Self {}
                }

                /// Allow the channel to generate DMA requests.
                #[allow(dead_code)]
                pub fn listen_dma(&self) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.dier.modify(|_, w| w.[< cc $index de >]().set_bit());
                }

                /// Operate the channel as an output-compare.
                ///
                /// # Args
                /// * `value` - The value to compare the sampling timer's counter against.
                #[allow(dead_code)]
                pub fn to_output_compare(&self, value: u32) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    assert!(value <= regs.arr.read().bits());
                    regs.[< ccr $index >].write(|w| w.ccr().bits(value));
                    regs.[< $ccmrx _output >]()
                        .modify(|_, w| unsafe { w.[< cc $index s >]().bits(0) });
                }

                /// Operate the channel in input-capture mode.
                ///
                /// # Args
                /// * `input` - The input source for the input capture event.
                #[allow(dead_code)]
                pub fn to_input_capture(self, input: hal::stm32::tim2::[< $ccmrx _input >]::[< CC $index S_A >]) -> [< Channel $index InputCapture >]{
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.[< $ccmrx _input >]().modify(|_, w| w.[< cc $index s>]().variant(input));

                    [< Channel $index InputCapture >] {}
                }
            }

            unsafe impl TargetAddress<PeripheralToMemory> for [< Channel $index InputCapture >] {
                type MemSize = u16;

                const REQUEST_LINE: Option<u8> = Some(DMAReq::[< $TY _CH $index >]as u8);

                fn address(&self) -> u32 {
                    let regs = unsafe { &*<$TY>::ptr() };
                    &regs.[<ccr $index >] as *const _ as u32
                }
            }
        }
    };
}

timer_channels!(SamplingTimer, TIM2);
timer_channels!(TimestampTimer, TIM5);
