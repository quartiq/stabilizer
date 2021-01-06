///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;

macro_rules! timer_channels {
    ($name:ident, $TY:ident, u32) => {
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

                /// Get the period of the timer.
                #[allow(dead_code)]
                pub fn get_period(&self) -> u32 {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.arr.read().arr().bits()
                }

                /// Manually set the period of the timer.
                #[allow(dead_code)]
                pub fn set_period_ticks(&mut self, period: u32) {
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
            /// A capture/compare channel of the timer.
            pub struct [< Channel $index >] {}

            /// A capture channel of the timer.
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
                pub fn into_input_capture(self, input: hal::stm32::tim2::[< $ccmrx _input >]::[< CC $index S_A >]) -> [< Channel $index InputCapture >]{
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.[< $ccmrx _input >]().modify(|_, w| w.[< cc $index s>]().variant(input));

                    [< Channel $index InputCapture >] {}
                }
            }

            impl [< Channel $index InputCapture >] {
                /// Get the latest capture from the channel.
                #[allow(dead_code)]
                pub fn latest_capture(&mut self) -> Result<Option<u32>, ()> {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    let sr = regs.sr.read();

                    let result = if sr.[< cc $index if >]().bit_is_set() {
                        // Read the capture value. Reading the captured value clears the flag in the
                        // status register automatically.
                        let ccx = regs.[< ccr $index >].read();
                        Some(ccx.ccr().bits())
                    } else {
                        None
                    };

                    // Read SR again to check for a potential over-capture. If there is an
                    // overcapture, return an error.
                    if regs.sr.read().[< cc $index of >]().bit_is_clear() {
                        Ok(result)
                    } else {
                        regs.sr.modify(|_, w| w.[< cc $index of >]().clear_bit());
                        Err(())
                    }
                }

                /// Allow the channel to generate DMA requests.
                #[allow(dead_code)]
                pub fn listen_dma(&self) {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.dier.modify(|_, w| w.[< cc $index de >]().set_bit());
                }

                /// Enable the input capture to begin capturing timer values.
                #[allow(dead_code)]
                pub fn enable(&mut self) {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.ccer.modify(|_, w| w.[< cc $index e >]().set_bit());
                }

                /// Check if an over-capture event has occurred.
                #[allow(dead_code)]
                pub fn check_overcapture(&self) -> bool {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.sr.read().[< cc $index of >]().bit_is_set()
                }
            }

            // Note(unsafe): This manually implements DMA support for input-capture channels. This
            // is safe as it is only completed once per channel and each DMA request is allocated to
            // each channel as the owner.
            unsafe impl TargetAddress<PeripheralToMemory> for [< Channel $index InputCapture >] {
                type MemSize = u32;

                const REQUEST_LINE: Option<u8> = Some(DMAReq::[< $TY _CH $index >]as u8);

                fn address(&self) -> usize {
                    let regs = unsafe { &*<$TY>::ptr() };
                    &regs.[<ccr $index >] as *const _ as usize
                }
            }
        }
    };
}

timer_channels!(SamplingTimer, TIM2, u32);
timer_channels!(TimestampTimer, TIM5, u32);
