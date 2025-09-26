//! Module for all hardware-specific setup of Stabilizer

pub use embedded_hal_02;
use embedded_hal_compat::{Forward, markers::ForwardOutputPin};
use hal::{
    flash::{LockedFlashBank, UnlockedFlashBank},
    gpio::{self, ErasedPin, Input, Output},
};
pub use stm32h7xx_hal as hal;

use platform::{ApplicationMetadata, AsyncFlash, UnlockFlash};

pub mod adc;
pub mod afe;
pub mod cpu_temp_sensor;
pub mod dac;
mod eeprom;
pub mod input_stamper;
pub mod net;
pub mod pounder;
pub mod setup;
pub mod shared_adc;
pub mod timers;

// Type alias for the analog front-end
pub type Pgia = afe::ProgrammableGainAmplifier<
    Forward<ErasedPin<Output>, ForwardOutputPin>,
>;

pub type UsbBus = hal::usb_hs::UsbBus<hal::usb_hs::USB2>;

// Type alias for the USB device.
pub type UsbDevice = usb_device::device::UsbDevice<'static, UsbBus>;

pub struct Gpio {
    pub lvds4: gpio::gpiod::PD1<Input>,
    pub lvds5: gpio::gpiod::PD2<Input>,
    pub lvds6: gpio::gpiod::PD3<Output>,
    pub lvds7: gpio::gpiod::PD4<Output>,
}

pub type Urukul = urukul::Urukul<
    'static,
    Forward<hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled>>,
    Forward<ErasedPin<Output>, ForwardOutputPin>,
>;

pub enum Eem {
    Gpio(Gpio),
    Urukul(Urukul),
    None,
}

// Type alias for digital input 0 (DI0).
pub type DigitalInput0 = hal::gpio::gpiog::PG9<hal::gpio::Input>;

// Type alias for digital input 1 (DI1).
pub type DigitalInput1 = hal::gpio::gpioc::PC15<hal::gpio::Input>;

/// System timer (RTIC Monotonic) tick frequency
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
rtic_monotonics::systick_monotonic!(Systick, MONOTONIC_FREQUENCY);
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

pub type I2c1Proxy = shared_bus::I2cProxy<
    'static,
    shared_bus::AtomicCheckMutex<hal::i2c::I2c<hal::stm32::I2C1>>,
>;

pub type SerialPort = usbd_serial::SerialPort<
    'static,
    UsbBus,
    &'static mut [u8],
    &'static mut [u8],
>;

pub type SerialTerminal<C> = serial_settings::Runner<
    'static,
    platform::SerialSettingsPlatform<C, AsyncFlash<Flash>, SerialPort>,
>;

pub struct Flash(LockedFlashBank);

impl embedded_storage::nor_flash::ErrorType for Flash {
    type Error =
        <LockedFlashBank as embedded_storage::nor_flash::ErrorType>::Error;
}

impl embedded_storage::nor_flash::ReadNorFlash for Flash {
    const READ_SIZE: usize = LockedFlashBank::READ_SIZE;

    fn capacity(&self) -> usize {
        self.0.capacity()
    }

    fn read(
        &mut self,
        offset: u32,
        bytes: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.0.read(offset, bytes)
    }
}

impl UnlockFlash for Flash {
    type Unlocked<'a> = UnlockedFlashBank<'a>;
    fn unlock(&mut self) -> Self::Unlocked<'_> {
        self.0.unlocked()
    }
}

mod build_info {
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

/// Construct the global metadata.
///
/// # Note
/// This may only be called once.
///
/// # Args
/// * `hardware_version` - The hardware version detected.
///
/// # Returns
/// A reference to the global metadata.
pub fn metadata(version: &'static str) -> &'static ApplicationMetadata {
    cortex_m::singleton!(: ApplicationMetadata = ApplicationMetadata {
        firmware_version: build_info::GIT_VERSION.unwrap_or("Unspecified"),
        rust_version: build_info::RUSTC_VERSION,
        profile: build_info::PROFILE,
        git_dirty: build_info::GIT_DIRTY.unwrap_or(false),
        features: build_info::FEATURES_STR,
        hardware_version: version,
        panic_info: panic_persist::get_panic_message_utf8().unwrap_or("None"),
    })
    .unwrap()
}

#[derive(strum::IntoStaticStr)]
pub enum HardwareVersion {
    Rev1_0,
    Rev1_1,
    Rev1_2,
    Rev1_3,
    Unknown(u8),
}

impl From<&[bool]> for HardwareVersion {
    fn from(bits: &[bool]) -> Self {
        match bits.iter().rev().fold(0, |v, b| (v << 1) | *b as u8) {
            0b000 => HardwareVersion::Rev1_0,
            0b001 => HardwareVersion::Rev1_1,
            0b010 => HardwareVersion::Rev1_2,
            0b011 => HardwareVersion::Rev1_3,
            other => HardwareVersion::Unknown(other),
        }
    }
}

impl core::fmt::Display for HardwareVersion {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            HardwareVersion::Rev1_0 => write!(f, "v1.0"),
            HardwareVersion::Rev1_1 => write!(f, "v1.1"),
            HardwareVersion::Rev1_2 => write!(f, "v1.2"),
            HardwareVersion::Rev1_3 => write!(f, "v1.3"),
            HardwareVersion::Unknown(other) => {
                write!(f, "Unknown ({:#b})", other)
            }
        }
    }
}

#[inline(never)]
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    use core::{
        fmt::Write,
        sync::atomic::{AtomicBool, Ordering},
    };
    use cortex_m::asm;
    use rtt_target::{ChannelMode, UpChannel};

    cortex_m::interrupt::disable();

    // Recursion protection
    static PANICKED: AtomicBool = AtomicBool::new(false);
    while PANICKED.load(Ordering::Relaxed) {
        asm::bkpt();
    }
    PANICKED.store(true, Ordering::Relaxed);

    // Turn on both red LEDs, FP_LED_1, FP_LED_3
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high());

    // Analogous to panic-rtt-target
    if let Some(mut channel) = unsafe { UpChannel::conjure(0) } {
        channel.set_mode(ChannelMode::BlockIfFull);
        writeln!(channel, "{info}").ok();
    }

    panic_persist::report_panic_info(info);

    // Abort
    asm::udf();
    // Halt
    // loop { core::sync::atomic::compiler_fence(Ordering::SeqCst); }
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[cortex_m_rt::exception]
unsafe fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
