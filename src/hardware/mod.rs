///! Module for all hardware-specific setup of Stabilizer
use stm32h7xx_hal as hal;

// Re-export for the DigitalInputs below:
pub use embedded_hal::digital::v2::InputPin;

#[cfg(feature = "semihosting")]
use panic_semihosting as _;

mod adc;
mod afe;
mod configuration;
mod cycle_counter;
mod dac;
pub mod design_parameters;
mod digital_input_stamper;
mod eeprom;
pub mod pounder;
mod system_timer;
mod timers;

pub use adc::{Adc0Input, Adc1Input, AdcCode};
pub use afe::Gain as AfeGain;
pub use cycle_counter::CycleCounter;
pub use dac::{Dac0Output, Dac1Output, DacCode};
pub use digital_input_stamper::InputStamper;
pub use pounder::DdsOutput;
pub use system_timer::SystemTimer;

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

// Type alias for digital input 0 (DI0).
pub type DigitalInput0 =
    hal::gpio::gpiog::PG9<hal::gpio::Input<hal::gpio::Floating>>;

// Type alias for digital input 1 (DI1).
pub type DigitalInput1 =
    hal::gpio::gpioc::PC15<hal::gpio::Input<hal::gpio::Floating>>;

pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    'static,
    hal::ethernet::EthernetDMA<'static>,
>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

pub use configuration::{setup, PounderDevices, StabilizerDevices};

#[inline(never)]
#[panic_handler]
#[cfg(all(not(feature = "semihosting")))]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    // Turn on both red LEDs, FP_LED_1, FP_LED_3
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high());
    loop {
        // Halt
        core::sync::atomic::compiler_fence(
            core::sync::atomic::Ordering::SeqCst,
        );
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
