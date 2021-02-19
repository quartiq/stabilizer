///! Module for all hardware-specific setup of Stabilizer
use stm32h7xx_hal as hal;

#[cfg(feature = "semihosting")]
use panic_semihosting as _;

#[cfg(not(any(feature = "nightly", feature = "semihosting")))]
use panic_halt as _;

mod adc;
mod afe;
mod configuration;
mod cycle_counter;
mod dac;
pub mod design_parameters;
mod digital_input_stamper;
mod eeprom;
pub mod pounder;
mod timers;

pub use adc::{Adc0Input, Adc1Input};
pub use afe::Gain as AfeGain;
pub use cycle_counter::CycleCounter;
pub use dac::{Dac0Output, Dac1Output};
pub use digital_input_stamper::InputStamper;
pub use pounder::DdsOutput;

// Type alias for the analog front-end (AFE) for ADC0.
pub type AFE0 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiof::PF2<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiof::PF5<hal::gpio::Output<hal::gpio::PushPull>>,
>;

// Type alias for the analog front-end (AFE) for ADC1.
pub type AFE1 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiod::PD15<hal::gpio::Output<hal::gpio::PushPull>>,
>;

pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    'static,
    hal::ethernet::EthernetDMA<'static>,
>;

pub use configuration::{setup, PounderDevices, StabilizerDevices};

#[inline(never)]
#[panic_handler]
#[cfg(all(feature = "nightly", not(feature = "semihosting")))]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high()); // FP_LED_1, FP_LED_3
    #[cfg(feature = "nightly")]
    core::intrinsics::abort();
    #[cfg(not(feature = "nightly"))]
    unsafe {
        core::intrinsics::abort();
    }
}

#[cortex_m_rt::exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[cortex_m_rt::exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
