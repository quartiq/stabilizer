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

use core::ptr;
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
    rcc.cr.modify(|_, w|
        w.hseon().set_bit()
         .hsebyp().clear_bit());
    while rcc.cr.read().hserdy().bit_is_clear() {}

    rcc.pllckselr.modify(|_, w| unsafe {
        w.pllsrc().bits(0b10)  // hse
         .divm1().bits(1)  // ref prescaler
         .divm2().bits(1)  // ref prescaler
    });
    // Configure PLL1: 8MHz /1 *100 /2 = 400 MHz
    rcc.pllcfgr.modify(|_, w| unsafe {
        w.pll1vcosel().clear_bit()  // 192-836 MHz VCO
         .pll1rge().bits(0b11)  // 8-16 MHz PFD
         .pll1fracen().clear_bit()
         .divp1en().set_bit()
         .pll2vcosel().set_bit()  // 150-420 MHz VCO
         .pll2rge().bits(0b11)  // 8-16 MHz PFD
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

    // Configure PLL2: 8MHz /1 *25 / 2 = 100 MHz
    rcc.pll2divr.write(|w| unsafe {
        w.divn1().bits(25 - 1)  // feebdack divider
         .divp1().bits(2 - 1)  // p output divider
         .divq1().bits(2 - 1)  // q output divider
    });
    rcc.cr.modify(|_, w| w.pll2on().set_bit());
    while rcc.cr.read().pll2rdy().bit_is_clear() {}

    // hclk 200 MHz, pclk 100 MHz
    let dapb = 0b100;
    rcc.d1cfgr.write(|w| unsafe {
        w.d1cpre().bits(0)  // sys_ck not divided
         .hpre().bits(0b1000)  // rcc_hclk3 = sys_d1cpre_ck / 2
         .d1ppre().bits(dapb) // rcc_pclk3 = rcc_hclk3 / 2
    });
    rcc.d2cfgr.write(|w| unsafe {
        w.d2ppre1().bits(dapb)  // rcc_pclk1 = rcc_hclk3 / 2
         .d2ppre2().bits(dapb) // rcc_pclk2 = rcc_hclk3 / 2

    });
    rcc.d3cfgr.write(|w| unsafe {
        w.d3ppre().bits(dapb)  // rcc_pclk4 = rcc_hclk3 / 2
    });

    let flash = dp.FLASH;
    // 2 wait states, 0b10 programming delay
    // 185-210 MHz
    flash.acr.write(|w| unsafe {
        w.wrhighfreq().bits(2)
         .latency().bits(2)
    });
    while flash.acr.read().latency().bits() != 2 {}

    // Set system clock to pll1_p
    rcc.cfgr.modify(|_, w| unsafe { w.sw().bits(0b011) });  // pll1p
    while rcc.cfgr.read().sws().bits() != 0b011 {}

    cp.SCB.enable_icache();
    cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    init_log();
    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    // info!("Built on {}", build_info::BUILT_TIME_UTC);
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

    rcc.d1ccipr.write(|w| unsafe {
        w.ckpersrc().bits(1)  // hse_ck
    });
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
    gpiog.odr.modify(|_, w|
        w.odr2().clear_bit()
         .odr3().clear_bit()
    );

    // SCK: PG11
    gpiog.moder.modify(|_, w| w.moder11().alternate());
    gpiog.otyper.modify(|_, w| w.ot11().push_pull());
    gpiog.ospeedr.modify(|_, w| w.ospeedr11().very_high_speed());
    gpiog.afrh.modify(|_, w| w.afr11().af5());
    // MOSI: PD7
    // MISO: PA6
    gpioa.moder.modify(|_, w| w.moder6().alternate());
    gpioa.afrl.modify(|_, w| w.afr6().af5());
    // NSS: PG10
    gpiog.moder.modify(|_, w| w.moder10().alternate());
    gpiog.otyper.modify(|_, w| w.ot10().push_pull());
    gpiog.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
    gpiog.afrh.modify(|_, w| w.afr10().af5());

    let spi1 = dp.SPI1;
    spi1.cfg1.modify(|_, w| unsafe {
        w.mbr().bits(0)  // clk/2
         .dsize().bits(16 - 1)
         .fthvl().bits(1 - 1)  // one data
    });
    spi1.cfg2.modify(|_, w| unsafe {
        w.afcntr().set_bit()
         .ssom().set_bit()  // ss deassert between frames during midi
         .ssoe().set_bit()  // ss output enable
         .ssiop().clear_bit()  // ss active low
         .ssm().clear_bit()  // PAD counts
         .cpol().set_bit()
         .cpha().set_bit()
         .lsbfrst().clear_bit()
         .master().set_bit()
         .sp().bits(0)  // motorola
         .comm().bits(0b10)  // simplex receiver
         .ioswp().clear_bit()
         .midi().bits(2)  // master inter data idle
         .mssi().bits(11)  // master SS idle
    });
    spi1.cr2.modify(|_, w| unsafe {
        w.tsize().bits(1)
    });
    spi1.cr1.write(|w| w.spe().set_bit());

    let gpiob = dp.GPIOB;
    rcc.ahb4enr.modify(|_, w| w.gpioben().set_bit());
    // SCK: PB10
    gpiob.moder.modify(|_, w| w.moder10().alternate());
    gpiob.otyper.modify(|_, w| w.ot10().push_pull());
    gpiob.ospeedr.modify(|_, w| w.ospeedr10().very_high_speed());
    gpiob.afrh.modify(|_, w| w.afr10().af5());
    // MOSI: PB15
    gpiob.moder.modify(|_, w| w.moder15().alternate());
    gpiob.otyper.modify(|_, w| w.ot15().push_pull());
    gpiob.ospeedr.modify(|_, w| w.ospeedr15().very_high_speed());
    gpiob.afrh.modify(|_, w| w.afr15().af5());
    // MISO: PB14
    // NSS: PB9
    gpiob.moder.modify(|_, w| w.moder9().alternate());
    gpiob.otyper.modify(|_, w| w.ot9().push_pull());
    gpiob.ospeedr.modify(|_, w| w.ospeedr9().very_high_speed());
    gpiob.afrh.modify(|_, w| w.afr9().af5());

    let gpioe = dp.GPIOE;
    rcc.ahb4enr.modify(|_, w| w.gpioeen().set_bit());
    // DAC0_LDAC: PE11
    gpioe.moder.modify(|_, w| w.moder11().output());
    gpioe.otyper.modify(|_, w| w.ot11().push_pull());
    gpioe.odr.modify(|_, w| w.odr11().clear_bit());
    // DAC_CLR: PE12
    gpioe.moder.modify(|_, w| w.moder12().output());
    gpioe.otyper.modify(|_, w| w.ot12().push_pull());
    gpioe.odr.modify(|_, w| w.odr12().set_bit());

    let spi2 = dp.SPI2;
    rcc.apb1lrstr.write(|w| w.spi2rst().set_bit());
    rcc.apb1lrstr.write(|w| w.spi2rst().clear_bit());
    rcc.apb1lenr.modify(|_, w| w.spi2en().set_bit());

    spi2.cfg1.modify(|_, w| unsafe {
        w.mbr().bits(0)  // clk/2
         .dsize().bits(16 - 1)
         .fthvl().bits(1 - 1)  // one data
    });
    spi2.cfg2.modify(|_, w| unsafe {
        w.afcntr().set_bit()
         .ssom().set_bit()  // ss deassert between frames during midi
         .ssoe().set_bit()  // ss output enable
         .ssiop().clear_bit()  // ss active low
         .ssm().clear_bit()  // PAD counts
         .cpol().clear_bit()
         .cpha().clear_bit()
         .lsbfrst().clear_bit()
         .master().set_bit()
         .sp().bits(0)  // motorola
         .comm().bits(0b01)  // simplex transmitter
         .ioswp().clear_bit()
         .midi().bits(1)  // master inter data idle
         .mssi().bits(0)  // master SS idle
    });
    spi2.cr2.modify(|_, w| unsafe {
        w.tsize().bits(0)
    });
    spi2.cr1.write(|w| w.spe().set_bit());
    spi2.cr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) });

    loop {
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();

        // at least one SCK between EOT and CSTART
        spi1.cr1.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 9)) });
        while spi1.sr.read().eot().bit_is_clear() {}
        spi1.ifcr.write(|w| w.eotc().set_bit());
        if spi1.sr.read().rxp().bit_is_clear() {
            continue;
        }
        let a = spi1.rxdr.read().rxdr().bits() as i16;
        let d = (a as u16) ^ 0x8000;

        if spi2.sr.read().txp().bit_is_clear() {
            continue;
        }
        let txdr = &spi2.txdr as *const _ as *mut u16;
        unsafe { ptr::write_volatile(txdr, d) };
        // at least one SCK between EOT and CSTART
        while spi2.sr.read().txc().bit_is_clear() {}

        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();

        info!("dac adc {:#x} cr1 {:#x} sr {:#x} cfg1 {:#x} cr2 {:#x}",
            a,
            spi2.cr1.read().bits(), spi2.sr.read().bits(),
            spi2.cfg1.read().bits(), spi2.cr2.read().bits(),
        );
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
