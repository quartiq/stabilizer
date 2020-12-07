///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;

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

macro_rules! timer_channels {
    ($TY:ty) => {
        paste::paste! {
            pub mod [< $TY:lower >] {
                pub use hal::stm32::[< $TY:lower >]::ccmr1_input::{CC1S_A, CC2S_A};
                pub use hal::stm32::[< $TY:lower >]::ccmr2_input::{CC3S_A, CC4S_A};

                use stm32h7xx_hal as hal;
                use hal::dma::{traits::TargetAddress, PeripheralToMemory, dma::DMAReq};
                use hal::stm32::TIM2;

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
                pub fn listen_dma(&self) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.dier.modify(|_, w| w.[< cc $index de >]().set_bit());
                }

                /// Operate the channel as an output-compare.
                ///
                /// # Args
                /// * `value` - The value to compare the sampling timer's counter against.
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
                pub fn to_input_capture(self, input: hal::stm32::[<$TY:lower>]::[< $ccmrx _input >]::[< CC $index S_A >]) -> [< Channel $index InputCapture >]{
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

timer_channels!(TIM2);
