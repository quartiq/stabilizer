//! Module for all hardware-specific setup of Stabilizer

pub use embedded_hal_02;
pub use stm32h7xx_hal as hal;

pub mod adc;
pub mod afe;
pub mod cpu_temp_sensor;
pub mod dac;
pub mod delay;
pub mod design_parameters;
pub mod eem;
mod eeprom;
pub mod flash;
pub mod input_stamper;
pub mod metadata;
pub mod platform;
pub mod pounder;
pub mod setup;
pub mod shared_adc;
pub mod signal_generator;
pub mod timers;

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

pub type UsbBus = stm32h7xx_hal::usb_hs::UsbBus<stm32h7xx_hal::usb_hs::USB2>;

// Type alias for the USB device.
pub type UsbDevice = usb_device::device::UsbDevice<'static, UsbBus>;

pub struct Gpio {
    pub lvds4: hal::gpio::gpiod::PD1<hal::gpio::Input>,
    pub lvds5: hal::gpio::gpiod::PD2<hal::gpio::Input>,
    pub lvds6: hal::gpio::gpiod::PD3<hal::gpio::Output>,
    pub lvds7: hal::gpio::gpiod::PD4<hal::gpio::Output>,
}

pub type Urukul = urukul::Urukul<
    'static,
    embedded_hal_compat::Forward<
        hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled>,
    >,
    embedded_hal_compat::Forward<
        hal::gpio::ErasedPin<hal::gpio::Output>,
        embedded_hal_compat::markers::ForwardOutputPin,
    >,
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

// Number of TX descriptors in the ethernet descriptor ring.
const TX_DESRING_CNT: usize = 4;

// Number of RX descriptors in the ethernet descriptor ring.
const RX_DESRING_CNT: usize = 4;

pub type NetworkStack = smoltcp_nal::NetworkStack<
    'static,
    hal::ethernet::EthernetDMA<TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type NetworkManager = smoltcp_nal::shared::NetworkManager<
    'static,
    hal::ethernet::EthernetDMA<TX_DESRING_CNT, RX_DESRING_CNT>,
    SystemTimer,
>;

pub type EthernetPhy = hal::ethernet::phy::LAN8742A<hal::ethernet::EthernetMAC>;

/// System timer (RTIC Monotonic) tick frequency
pub const MONOTONIC_FREQUENCY: u32 = 1_000;
rtic_monotonics::systick_monotonic!(Systick, MONOTONIC_FREQUENCY);
pub type SystemTimer = mono_clock::MonoClock<u32, MONOTONIC_FREQUENCY>;

pub type I2c1 = hal::i2c::I2c<hal::stm32::I2C1>;
pub type I2c1Proxy =
    shared_bus::I2cProxy<'static, shared_bus::AtomicCheckMutex<I2c1>>;

pub type SerialPort = usbd_serial::SerialPort<
    'static,
    crate::hardware::UsbBus,
    &'static mut [u8],
    &'static mut [u8],
>;

pub type SerialTerminal<C, const Y: usize> = serial_settings::Runner<
    'static,
    crate::settings::SerialSettingsPlatform<C>,
    Y,
>;

pub enum HardwareVersion {
    Rev1_0,
    Rev1_1,
    Rev1_2,
    Rev1_3,
    Unknown(u8),
}

impl From<u8> for HardwareVersion {
    fn from(bitfield: u8) -> Self {
        match bitfield {
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

impl serde::Serialize for HardwareVersion {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        use core::fmt::Write;

        let mut version_string: heapless::String<32> = heapless::String::new();
        write!(&mut version_string, "{}", self).unwrap();
        serializer.serialize_str(&version_string)
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
