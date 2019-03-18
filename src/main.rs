#![no_std]
#![no_main]
#![feature(asm)]
// Enable returning `!`
#![feature(never_type)]
#[allow(unused_extern_crates)]

#[cfg(not(feature = "semihosting"))]
extern crate panic_abort;
#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

extern crate cortex_m;
extern crate cortex_m_rt;
extern crate stm32h7;

#[macro_use]
extern crate log;

use cortex_m_rt::{entry, exception};
// use core::fmt::Write;
use stm32h7::{stm32h7x3 as stm32};

#[cfg(not(feature = "semihosting"))]
fn init_log() {}

#[cfg(feature = "semihosting")]
fn init_log() {
    use log::LevelFilter;
    use cortex_m_log::log::{Logger, init};
    use cortex_m_log::printer::semihosting::{InterruptOk, hio::HStdout};
    static mut LOGGER: Option<Logger<InterruptOk<HStdout>>> = None;
    let logger = Logger {
        inner: InterruptOk::<_>::stdout().expect("semihosting stdout"),
        level: LevelFilter::Info,
    };
    let logger = unsafe {
        LOGGER.get_or_insert(logger)
    };

    init(logger).expect("set logger");
}

// Pull in build information (from `built` crate)
mod build_info {
    #![allow(dead_code)]
    include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

#[entry]
fn main() -> ! {
    init_log();
    info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);
    info!("Built on {}", build_info::BUILT_TIME_UTC);

    let mut cp = cortex_m::Peripherals::take().unwrap();
    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();
    let mut dp = stm32::Peripherals::take().unwrap();
    /*
let clocks = dp.RCC.constrain()
        .cfgr
        .sysclk(84.mhz())
        .hclk(84.mhz())
        .pclk1(16.mhz())
        .pclk2(32.mhz())
        .freeze();
    let gpiod = dp.GPIOD.split();
    let mut led_fp0 = gpiod.pd5.into_push_pull_output();
*/

    cortex_m::interrupt::free(|_cs| {
    });
    loop {
        cortex_m::asm::wfi();
    }
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
