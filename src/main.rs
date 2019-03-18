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
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().unwrap();

    // go to VOS1 voltage scale high perf
    let pwr = dp.PWR;
    pwr.pwr_cr3.write(|w|
        w.sden().set_bit()
         .ldoen().set_bit()
         .bypass().clear_bit()
    );
    while pwr.pwr_csr1.read().actvosrdy().bit_is_clear() {}
    pwr.pwr_d3cr.write(|w| unsafe { w.vos().bits(0b11) });  // vos1
    while pwr.pwr_d3cr.read().vosrdy().bit_is_clear() {}

    let rcc = dp.RCC;

    // Reset all peripherals
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0)});
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0)});
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0)});

    rcc.ahb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb2rstr.write(|w| unsafe { w.bits(0)});
    rcc.apb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb2rstr.write(|w| unsafe { w.bits(0)});

    // do not reset the cpu
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0x7FFF_FFFF) });
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0)});
    rcc.apb3rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb3rstr.write(|w| unsafe { w.bits(0)});

    rcc.ahb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb4rstr.write(|w| unsafe { w.bits(0)});
    rcc.apb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb4rstr.write(|w| unsafe { w.bits(0)});

    // Ensure HSI is on and stable
    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsirdy().bit_is_clear() {}

    // Set system clock to HSI
    rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(0) });  // hsi
    while rcc.cfgr.read().sws().bits() != 0 {}

    // Clear registers to reset value
    rcc.cr.write(|w| w.hsion().set_bit());
    rcc.cfgr.reset();

    // Ensure HSE is on and stable
    rcc.cr.modify(|_, w| w.hseon().set_bit());
    while rcc.cr.read().hserdy().bit_is_clear() {}

    rcc.pllckselr.modify(|_, w| unsafe {
        w.pllsrc().bits(0b10)  // hse
         .divm1().bits(1)  // ref prescaler
         .divm2().bits(4)  // ref prescaler
    });
    // Configure PLL1: 8MHz /1 *100 /2 = 400 MHz
    rcc.pllcfgr.modify(|_, w| unsafe {
        w.pll1vcosel().clear_bit()  // 192-836 MHz VCO
         .pll1rge().bits(0b11)  // 8-16 MHz PFD
         .pll1fracen().clear_bit()
         .divp1en().set_bit()
         .pll2vcosel().set_bit()  // 150-420 MHz VCO
         .pll2rge().bits(0b01)  // 2-4 MHz PFD
         .pll2fracen().clear_bit()
         .divp2en().set_bit()
         .divq2en().set_bit()
    });
    rcc.pll1divr.write(|w| unsafe {
        w.divn1().bits(100 - 1)  // feebdack divider
         .divp1().bits(2 - 1)  // p output divider
    });
    rcc.cr.modify(|_, w| w.pll1on().set_bit());
    while rcc.cr.read().pll1rdy().bit_is_clear() {}

    // Configure PLL2: 8MHz /4 * 125 = 250 MHz
    rcc.pll2divr.write(|w| unsafe {
        w.divn1().bits(125 - 1)  // feebdack divider
         .divp1().bits(2 - 1)  // p output divider
         .divq1().bits(2 - 1)  // q output divider
    });
    rcc.cr.modify(|_, w| w.pll2on().set_bit());
    while rcc.cr.read().pll2rdy().bit_is_clear() {}

    // hclk 200 MHz, pclk 100 MHz
    rcc.d1cfgr.write(|w| unsafe {
        w.d1cpre().bits(0)  // sys_ck not divided
         .d1ppre().bits(0b100) // rcc_pclk3 = rcc_hclk3 / 2
         .hpre().bits(0b1000)  // rcc_hclk3 = sys_d1cpre_ck / 2
    });
    rcc.d2cfgr.write(|w| unsafe {
        w.d2ppre1().bits(0b100)  // rcc_pclk1 = rcc_hclk3 / 2
         .d2ppre2().bits(0b100) // rcc_pclk2 = rcc_hclk3 / 2
    });
    rcc.d3cfgr.write(|w| unsafe {
        w.d3ppre().bits(0b100)  // rcc_pclk4 = rcc_hclk3 / 2
    });

    let flash = dp.FLASH;
    // 2 wait states, 0b10 programming delay
    // 185-210 MHz
    flash.acr.write(|w| unsafe {
        w.wrhighfreq().bits(2)
         .latency().bits(2)
    });
    while flash.acr.read().latency().bits() != 2 {}

    // Set system clock to HSI
    rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(0b011) });  // pll1p
    while rcc.cfgr.read().sws().bits() != 0b011 {}

    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    init_log();
    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    info!("Built on {}", build_info::BUILT_TIME_UTC);
    // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

    // FP_LED0
    let gpiod = dp.GPIOD;
    rcc.ahb4enr.modify(|_, w| w.gpioden().set_bit());
    gpiod.otyper.modify(|_, w| w.ot5().push_pull());
    gpiod.moder.modify(|_, w| w.moder5().output());
    gpiod.odr.modify(|_, w| w.odr5().set_bit());

    // FP_LED1
    gpiod.otyper.modify(|_, w| w.ot6().push_pull());
    gpiod.moder.modify(|_, w| w.moder6().output());
    gpiod.odr.modify(|_, w| w.odr6().set_bit());

    // LED_FP2
    let gpiog = dp.GPIOG;
    rcc.ahb4enr.modify(|_, w| w.gpiogen().set_bit());
    gpiog.otyper.modify(|_, w| w.ot4().push_pull());
    gpiog.moder.modify(|_, w| w.moder4().output());
    gpiog.odr.modify(|_, w| w.odr4().set_bit());

    // LED_FP3
    gpiod.otyper.modify(|_, w| w.ot12().push_pull());
    gpiod.moder.modify(|_, w| w.moder12().output());
    gpiod.odr.modify(|_, w| w.odr12().set_bit());

    rcc.d2ccip1r.modify(|_, w| unsafe {
        w.spi123src().bits(1)  // pll2_p
         .spi45src().bits(1)  // pll2_q
    });
    rcc.d3ccipr.modify(|_, w| unsafe {
        w.spi6src().bits(1)  // pll2_q
    });

    // Set up peripheral clocks
    rcc.ahb1enr.modify(|_, w|
        w.dma1en().set_bit()
         .dma2en().set_bit()
    );
    rcc.apb1lenr.modify(|_, w|
        w.spi2en().set_bit()
         .spi3en().set_bit()
    );
    rcc.apb2enr.modify(|_, w|
        w.spi1en().set_bit()
         .spi4en().set_bit()
         .spi5en().set_bit()
    );
    rcc.apb4enr.modify(|_, w|
        w.spi6en().set_bit()
    );

    let gpioa = dp.GPIOA;
    rcc.ahb4enr.modify(|_, w| w.gpioaen().set_bit());

    // AFE0_A0,1: PG2,PG3
    gpiog.otyper.modify(|_, w|
        w.ot2().push_pull()
         .ot3().push_pull()
    );
    gpiog.moder.modify(|_, w|
        w.moder2().output()
         .moder3().output()
    );
    gpiod.odr.modify(|_, w|
        w.odr2().clear_bit()
         .odr3().clear_bit()
    );

    // SCK: PG11
    gpiog.moder.modify(|_, w| w.moder11().alternate());
    gpiog.otyper.modify(|_, w| w.ot11().push_pull());
    gpiog.afrh.modify(|_, w| w.afr11().af5());
    // MOSI: PD7
    // MISO: PA6
    gpioa.moder.modify(|_, w| w.moder6().alternate());
    gpioa.afrl.modify(|_, w| w.afr6().af5());
    // NSS: PG10
    gpiog.moder.modify(|_, w| w.moder10().alternate());
    gpiog.otyper.modify(|_, w| w.ot10().push_pull());
    gpiog.afrh.modify(|_, w| w.afr10().af5());

    let spi1 = dp.SPI1;
    spi1.cfg1.modify(|_, w| unsafe {
        // w.mbr().bits(0)  // clk/2
        w.mbr().bits(1)  // FIXME
         .dsize().bits(16 - 1)
         .fthvl().bits(1 - 1)  // one data
    });
    spi1.cfg2.modify(|_, w| unsafe {
        w.ssom().set_bit()  // ss deassert between frames during midi
         .ssoe().set_bit()  // ss output enable
         .ssiop().clear_bit()  // ss active low
         .ssm().clear_bit()  // PAD counts
         .cpol().clear_bit()
         .cpha().clear_bit()
         .lsbfrst().clear_bit()
         .master().set_bit()
         .sp().bits(0)  // motorola
         .comm().bits(0b10)  // simplex receiver
         .ioswp().clear_bit()
         .midi().bits(2)  // master inter data idle
         .mssi().bits(15)  // master SS idle
    });
    spi1.cr2.modify(|_, w| unsafe {
        w.tsize().bits(1)
    });
    spi1.cr1.write(|w| w.spe().set_bit());

    // cortex_m::interrupt::free(|_cs| { });
    loop {
        // spi1.cr1.write(|w| w.cstart().set_bit());
        spi1.cr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) });
        while spi1.sr.read().eot().bit_is_clear() {}
        spi1.ifcr.write(|w| w.eotc().set_bit());
        while spi1.sr.read().rxp().bit_is_set() {
           let a = spi1.rxdr.read().rxdr().bits() as i16;
           info!("adc {}", a);
        }
        // cortex_m::asm::wfi();
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
