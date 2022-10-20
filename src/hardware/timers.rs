///! The sampling timer is used for managing ADC sampling and external reference timestamping.
use super::hal;
use num_enum::TryFromPrimitive;

use hal::stm32::{
    // TIM1 and TIM8 have identical registers.
    tim1 as __tim8,
    tim2 as __tim2,
    // TIM2 and TIM5 have identical registers.
    tim2 as __tim5,
    tim3 as __tim3,
};

/// The event that should generate an external trigger from the peripheral.
#[allow(dead_code)]
pub enum TriggerGenerator {
    Reset = 0b000,
    Enable = 0b001,
    Update = 0b010,
    ComparePulse = 0b011,
    Ch1Compare = 0b100,
    Ch2Compare = 0b101,
    Ch3Compare = 0b110,
    Ch4Compare = 0b111,
}

/// Selects the trigger source for the timer peripheral.
#[allow(dead_code)]
pub enum TriggerSource {
    Trigger0 = 0,
    Trigger1 = 0b01,
    Trigger2 = 0b10,
    Trigger3 = 0b11,
}

/// Prescalers for externally-supplied reference clocks.
#[allow(dead_code)]
#[derive(TryFromPrimitive)]
#[repr(u8)]
pub enum Prescaler {
    Div1 = 0b00,
    Div2 = 0b01,
    Div4 = 0b10,
    Div8 = 0b11,
}

/// Optional slave operation modes of a timer.
#[allow(dead_code)]
pub enum SlaveMode {
    Disabled = 0,
    Trigger = 0b0110,
}

/// Optional input capture preconditioning filter configurations.
#[allow(dead_code)]
pub enum InputFilter {
    Div1N1 = 0b0000,
    Div1N8 = 0b0011,
}

macro_rules! timer_channels {
    ($name:ident, $TY:ident, $size:ty) => {
        paste::paste! {

            /// The timer used for managing ADC sampling.
            pub struct $name {
                timer: hal::timer::Timer<hal::stm32::[< $TY >]>,
                channels: Option<[< $TY:lower >]::Channels>,
                update_event: Option<[< $TY:lower >]::UpdateEvent>,
            }

            impl $name {
                /// Construct the sampling timer.
                #[allow(dead_code)]
                pub fn new(mut timer: hal::timer::Timer<hal::stm32::[< $TY>]>) -> Self {
                    timer.pause();

                    Self {
                        timer,
                        // Note(unsafe): Once these channels are taken, we guarantee that we do not
                        // modify any of the underlying timer channel registers, as ownership of the
                        // channels is now provided through the associated channel structures. We
                        // additionally guarantee this can only be called once because there is only
                        // one Timer2 and this resource takes ownership of it once instantiated.
                        channels: unsafe { Some([< $TY:lower >]::Channels::new()) },
                        update_event: unsafe { Some([< $TY:lower >]::UpdateEvent::new()) },
                    }
                }

                /// Get the timer capture/compare channels.
                #[allow(dead_code)]
                pub fn channels(&mut self) -> [< $TY:lower >]::Channels {
                    self.channels.take().unwrap()
                }

                /// Get the timer update event.
                #[allow(dead_code)]
                pub fn update_event(&mut self) -> [< $TY:lower >]::UpdateEvent {
                    self.update_event.take().unwrap()
                }

                /// Get the period of the timer.
                #[allow(dead_code)]
                pub fn get_period(&self) -> $size {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.arr.read().arr().bits()
                }

                /// Manually set the period of the timer.
                #[allow(dead_code)]
                pub fn set_period_ticks(&mut self, period: $size) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.arr.write(|w| w.arr().bits(period));

                    // Force the new period to take effect immediately.
                    self.timer.apply_freq();
                }

                /// Clock the timer from an external source.
                ///
                /// # Note:
                /// * Currently, only an external source applied to ETR is supported.
                ///
                /// # Args
                /// * `prescaler` - The prescaler to use for the external source.
                #[allow(dead_code)]
                pub fn set_external_clock(&mut self, prescaler: Prescaler) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    regs.smcr.modify(|_, w| w.etps().bits(prescaler as u8).ece().set_bit());

                    // Clear any other prescaler configuration.
                    regs.psc.write(|w| w.psc().bits(0));
                }

                /// Start the timer.
                #[allow(dead_code)]
                pub fn start(&mut self) {
                    // Force a refresh of the frequency settings.
                    self.timer.apply_freq();
                    self.timer.reset_counter();

                    self.timer.resume();
                }

                /// Configure the timer peripheral to generate a trigger based on the provided
                /// source.
                #[allow(dead_code)]
                pub fn generate_trigger(&mut self, source: TriggerGenerator) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    // Note(unsafe) The TriggerGenerator enumeration is specified such that this is
                    // always in range.
                    regs.cr2.modify(|_, w| w.mms().bits(source as u8));

                }

                /// Select a trigger source for the timer peripheral.
                #[allow(dead_code)]
                pub fn set_trigger_source(&mut self, source: TriggerSource) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    // Note(unsafe) The TriggerSource enumeration is specified such that this is
                    // always in range.
                    regs.smcr.modify(|_, w| unsafe { w.ts().bits(source as u8) } );
                }

                #[allow(dead_code)]
                pub fn set_slave_mode(&mut self, source: TriggerSource, mode: SlaveMode) {
                    let regs = unsafe { &*hal::stm32::$TY::ptr() };
                    // Note(unsafe) The TriggerSource and SlaveMode enumerations are specified such
                    // that they are always in range.
                    regs.smcr.modify(|_, w| unsafe { w.sms().bits(mode as u8).ts().bits(source as u8) } );
                }
            }

            pub mod [< $TY:lower >] {
                use stm32h7xx_hal as hal;
                use hal::dma::{traits::TargetAddress, PeripheralToMemory, dma::DMAReq};
                use hal::stm32::$TY;

                pub struct UpdateEvent {}

                impl UpdateEvent {
                    /// Create a new update event
                    ///
                    /// # Safety
                    /// This is only safe to call once.
                    #[allow(dead_code)]
                    pub unsafe fn new() -> Self {
                        Self {}
                    }

                    /// Enable DMA requests upon timer updates.
                    #[allow(dead_code)]
                    pub fn listen_dma(&self) {
                        // Note(unsafe): We perform only atomic operations on the timer registers.
                        let regs = unsafe { &*<$TY>::ptr() };
                        regs.dier.modify(|_, w| w.ude().set_bit());
                    }

                    /// Trigger a DMA request manually
                    #[allow(dead_code)]
                    pub fn trigger(&self) {
                        let regs = unsafe { &*<$TY>::ptr() };
                        regs.egr.write(|w| w.ug().set_bit());
                    }
                }

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
                    /// # Safety
                    /// This is only safe to call once.
                    #[allow(dead_code)]
                    pub unsafe fn new() -> Self {
                        Self {
                            ch1: Channel1::new(),
                            ch2: Channel2::new(),
                            ch3: Channel3::new(),
                            ch4: Channel4::new(),
                        }
                    }
                }

                timer_channels!(1, $TY, ccmr1, $size);
                timer_channels!(2, $TY, ccmr1, $size);
                timer_channels!(3, $TY, ccmr2, $size);
                timer_channels!(4, $TY, ccmr2, $size);
            }
        }
    };

    ($index:expr, $TY:ty, $ccmrx:expr, $size:ty) => {
        paste::paste! {
            pub use super::[< __ $TY:lower >]::[< $ccmrx _input >]::[< CC $index S_A>] as [< CaptureSource $index >];

            /// A capture/compare channel of the timer.
            pub struct [< Channel $index >] {}

            /// A capture channel of the timer.
            pub struct [< Channel $index InputCapture>] {}

            impl [< Channel $index >] {
                /// Construct a new timer channel.
                ///
                /// Note(unsafe): This function must only be called once. Once constructed, the
                /// constructee guarantees to never modify the timer channel.
                #[allow(dead_code)]
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
                pub fn to_output_compare(&self, value: $size) {
                    let regs = unsafe { &*<$TY>::ptr() };
                    let arr = regs.arr.read().bits() as $size;
                    assert!(value <= arr);
                    regs.ccr[$index - 1].write(|w| w.ccr().bits(value));
                    regs.[< $ccmrx _output >]()
                        .modify(|_, w| unsafe { w.[< cc $index s >]().bits(0) });
                }

                /// Operate the channel in input-capture mode.
                ///
                /// # Args
                /// * `input` - The input source for the input capture event.
                #[allow(dead_code)]
                pub fn into_input_capture(self, input: [< CaptureSource $index >]) -> [< Channel $index InputCapture >]{
                    let regs = unsafe { &*<$TY>::ptr() };

                    regs.[< $ccmrx _input >]().modify(|_, w| w.[< cc $index s>]().variant(input));

                    [< Channel $index InputCapture >] {}
                }
            }

            impl [< Channel $index InputCapture >] {
                /// Get the latest capture from the channel.
                #[allow(dead_code)]
                pub fn latest_capture(&mut self) -> Result<Option<$size>, Option<$size>> {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };

                    if regs.sr.read().[< cc $index if >]().bit_is_set() {
                        // Read the capture value. Reading the captured value clears the flag in the
                        // status register automatically.
                        let result = regs.ccr[$index - 1].read().ccr().bits();

                        // Read SR again to check for a potential over-capture. Return an error in
                        // that case.
                        let sr = regs.sr.read();
                        if sr.[< cc $index of >]().bit_is_set() {
                            // NOTE(unsafe) write-back is safe
                            regs.sr.write(|w| unsafe { w.bits(sr.bits()) }.[< cc $index of >]().clear_bit());
                            Err(Some(result))
                        } else {
                            Ok(Some(result))
                        }
                    } else {
                        Ok(None)
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
                    // Read the latest input capture to clear any pending data in the register.
                    let _ = self.latest_capture();

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

                /// Configure the input capture input pre-filter.
                ///
                /// # Args
                /// * `filter` - The desired input filter stage configuration. Defaults to disabled.
                #[allow(dead_code)]
                pub fn configure_filter(&mut self, filter: super::InputFilter) {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    regs.[< $ccmrx _input >]().modify(|_, w| w.[< ic $index f >]().bits(filter as u8));
                }

                /// Configure the input capture prescaler.
                ///
                /// # Args
                /// * `psc` - Prescaler exponent.
                #[allow(dead_code)]
                pub fn configure_prescaler(&mut self, prescaler: super::Prescaler) {
                    // Note(unsafe): This channel owns all access to the specific timer channel.
                    // Only atomic operations on completed on the timer registers.
                    let regs = unsafe { &*<$TY>::ptr() };
                    // Note(unsafe): Enum values are all valid.
                    #[allow(unused_unsafe)]
                    regs.[< $ccmrx _input >]().modify(|_, w| unsafe {
                        w.[< ic $index psc >]().bits(prescaler as u8)});
                }
            }

            // Note(unsafe): This manually implements DMA support for input-capture channels. This
            // is safe as it is only completed once per channel and each DMA request is allocated to
            // each channel as the owner.
            unsafe impl TargetAddress<PeripheralToMemory> for [< Channel $index InputCapture >] {
                type MemSize = $size;

                const REQUEST_LINE: Option<u8> = Some(DMAReq::[< $TY:camel Ch $index >]as u8);

                fn address(&self) -> usize {
                    let regs = unsafe { &*<$TY>::ptr() };
                    &regs.ccr[$index - 1] as *const _ as usize
                }
            }
        }
    };
}

timer_channels!(SamplingTimer, TIM2, u32);
timer_channels!(ShadowSamplingTimer, TIM3, u16);

timer_channels!(TimestampTimer, TIM5, u32);
timer_channels!(PounderTimestampTimer, TIM8, u16);
