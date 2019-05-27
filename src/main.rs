#![no_std]
#![no_main]
#![feature(asm)]
// Enable returning `!`
#![feature(never_type)]

#[cfg(not(feature = "semihosting"))]
extern crate panic_abort;
#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

#[macro_use]
extern crate log;

use core::ptr;
use core::cell::RefCell;
use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use core::fmt::Write;
use cortex_m_rt::{entry, exception};
use stm32h7::stm32h7x3::{self as stm32, Peripherals, CorePeripherals, interrupt};
use cortex_m::interrupt::Mutex;
use heapless::{String, Vec, consts::*};

use smoltcp as net;

use serde::{Serialize, Deserialize};
use serde_json_core::{ser::to_string, de::from_slice};

mod eth;

mod iir;
use iir::*;

#[cfg(not(feature = "semihosting"))]
fn init_log() {}

#[cfg(feature = "semihosting")]
fn init_log() {
    use log::LevelFilter;
    use cortex_m_log::log::{Logger, init};
    use cortex_m_log::printer::semihosting::{InterruptOk, hio::HStdout};
    static mut LOGGER: Option<Logger<InterruptOk<HStdout>>> = None;
    let logger = Logger {
        inner: InterruptOk::<_>::stdout().unwrap(),
        level: LevelFilter::Info,
    };
    let logger = unsafe {
        LOGGER.get_or_insert(logger)
    };

    init(logger).unwrap();
}

// Pull in build information (from `built` crate)
mod build_info {
    #![allow(dead_code)]
    // include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

fn pwr_setup(pwr: &stm32::PWR) {
    // go to VOS1 voltage scale for high perf
    pwr.cr3.write(|w|
        w.scuen().set_bit()
         .ldoen().set_bit()
         .bypass().clear_bit()
    );
    while pwr.csr1.read().actvosrdy().bit_is_clear() {}
    pwr.d3cr.write(|w| unsafe { w.vos().bits(0b11) });  // vos1
    while pwr.d3cr.read().vosrdy().bit_is_clear() {}
}

fn rcc_reset(rcc: &stm32::RCC) {
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
}

fn rcc_pll_setup(rcc: &stm32::RCC, flash: &stm32::FLASH) {
    // Switch to HSI to mess with HSE
    rcc.cr.modify(|_, w| w.hsion().on());
    while rcc.cr.read().hsirdy().is_not_ready() {}
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}
    rcc.cr.write(|w| w.hsion().on());
    rcc.cfgr.reset();

    // Ensure HSE is on and stable
    rcc.cr.modify(|_, w|
        w.hseon().on()
         .hsebyp().not_bypassed());
    while !rcc.cr.read().hserdy().is_ready() {}

    rcc.pllckselr.modify(|_, w|
        w.pllsrc().hse()
         .divm1().bits(1)  // ref prescaler
         .divm2().bits(1)  // ref prescaler
    );
    // Configure PLL1: 8MHz /1 *100 /2 = 400 MHz
    rcc.pllcfgr.modify(|_, w|
        w.pll1vcosel().wide_vco()  // 192-836 MHz VCO
         .pll1rge().range8()  // 8-16 MHz PFD
         .pll1fracen().reset()
         .divp1en().enabled()
         .pll2vcosel().medium_vco()  // 150-420 MHz VCO
         .pll2rge().range8()  // 8-16 MHz PFD
         .pll2fracen().reset()
         .divp2en().enabled()
         .divq2en().enabled()
    );
    rcc.pll1divr.write(|w| unsafe {
        w.divn1().bits(100 - 1)  // feebdack divider
         .divp1().div2()  // p output divider
    });
    rcc.cr.modify(|_, w| w.pll1on().on());
    while !rcc.cr.read().pll1rdy().is_ready() {}

    // Configure PLL2: 8MHz /1 *25 / 2 = 100 MHz
    rcc.pll2divr.write(|w| unsafe {
        w.divn1().bits(25 - 1)  // feebdack divider
         .divp1().bits(2 - 1)  // p output divider
         .divq1().bits(2 - 1)  // q output divider
    });
    rcc.cr.modify(|_, w| w.pll2on().on());
    while !rcc.cr.read().pll2rdy().is_ready() {}

    // hclk 200 MHz, pclk 100 MHz
    rcc.d1cfgr.write(|w|
        w.d1cpre().div1()  // sys_ck not divided
         .hpre().div2()  // rcc_hclk3 = sys_d1cpre_ck / 2
         .d1ppre().div2() // rcc_pclk3 = rcc_hclk3 / 2
    );
    rcc.d2cfgr.write(|w|
        w.d2ppre1().div2()  // rcc_pclk1 = rcc_hclk3 / 2
         .d2ppre2().div2() // rcc_pclk2 = rcc_hclk3 / 2
    );
    rcc.d3cfgr.write(|w|
        w.d3ppre().div2()  // rcc_pclk4 = rcc_hclk3 / 2
    );

    // 2 wait states, 0b10 programming delay
    // 185-210 MHz
    flash.acr.write(|w| unsafe {
        w.wrhighfreq().bits(2)
         .latency().bits(2)
    });
    while flash.acr.read().latency().bits() != 2 {}

    // CSI for I/O compensationc ell
    rcc.cr.modify(|_, w| w.csion().on());
    while !rcc.cr.read().csirdy().is_ready() {}

    // Set system clock to pll1_p
    rcc.cfgr.modify(|_, w| w.sw().pll1());
    while !rcc.cfgr.read().sws().is_pll1() {}

    rcc.d1ccipr.write(|w| w.ckpersel().hse());
    rcc.d2ccip1r.modify(|_, w|
        w.spi123sel().pll2_p()
         .spi45sel().pll2_q()
    );
    rcc.d3ccipr.modify(|_, w| w.spi6sel().pll2_q());
}

fn io_compensation_setup(syscfg: &stm32::SYSCFG) {
    syscfg.cccsr.modify(|_, w|
        w.en().set_bit()
         .cs().clear_bit()
         .hslv().clear_bit()
    );
    while syscfg.cccsr.read().ready().bit_is_clear() {}
}

fn gpio_setup(gpioa: &stm32::GPIOA, gpiob: &stm32::GPIOB, gpiod: &stm32::GPIOD,
              gpioe: &stm32::GPIOE, gpiof: &stm32::GPIOF, gpiog: &stm32::GPIOG) {
    // FP_LED0
    gpiod.otyper.modify(|_, w| w.ot5().push_pull());
    gpiod.moder.modify(|_, w| w.moder5().output());
    gpiod.odr.modify(|_, w| w.odr5().clear_bit());

    // FP_LED1
    gpiod.otyper.modify(|_, w| w.ot6().push_pull());
    gpiod.moder.modify(|_, w| w.moder6().output());
    gpiod.odr.modify(|_, w| w.odr6().clear_bit());

    // LED_FP2
    gpiog.otyper.modify(|_, w| w.ot4().push_pull());
    gpiog.moder.modify(|_, w| w.moder4().output());
    gpiog.odr.modify(|_, w| w.odr4().clear_bit());

    // LED_FP3
    gpiod.otyper.modify(|_, w| w.ot12().push_pull());
    gpiod.moder.modify(|_, w| w.moder12().output());
    gpiod.odr.modify(|_, w| w.odr12().clear_bit());

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

    // ADC0
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

    // DAC0
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

    // DAC0_LDAC: PE11
    gpioe.moder.modify(|_, w| w.moder11().output());
    gpioe.otyper.modify(|_, w| w.ot11().push_pull());
    gpioe.odr.modify(|_, w| w.odr11().clear_bit());

    // DAC_CLR: PE12
    gpioe.moder.modify(|_, w| w.moder12().output());
    gpioe.otyper.modify(|_, w| w.ot12().push_pull());
    gpioe.odr.modify(|_, w| w.odr12().set_bit());

    // AFE1_A0,1: PD14,PD15
    gpiod.otyper.modify(|_, w|
        w.ot14().push_pull()
         .ot15().push_pull()
    );
    gpiod.moder.modify(|_, w|
        w.moder14().output()
         .moder15().output()
    );
    gpiod.odr.modify(|_, w|
        w.odr14().clear_bit()
         .odr15().clear_bit()
    );

    // ADC1
    // SCK: PF6
    gpiof.moder.modify(|_, w| w.moder7().alternate());
    gpiof.otyper.modify(|_, w| w.ot7().push_pull());
    gpiof.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());
    gpiof.afrl.modify(|_, w| w.afr7().af5());
    // MOSI: PF9
    // MISO: PF7
    gpiof.moder.modify(|_, w| w.moder8().alternate());
    gpiof.afrh.modify(|_, w| w.afr8().af5());
    // NSS: PF8
    gpiof.moder.modify(|_, w| w.moder6().alternate());
    gpiof.otyper.modify(|_, w| w.ot6().push_pull());
    gpiof.ospeedr.modify(|_, w| w.ospeedr6().very_high_speed());
    gpiof.afrl.modify(|_, w| w.afr6().af5());

    // DAC1
    // SCK: PE2
    gpioe.moder.modify(|_, w| w.moder2().alternate());
    gpioe.otyper.modify(|_, w| w.ot2().push_pull());
    gpioe.ospeedr.modify(|_, w| w.ospeedr2().very_high_speed());
    gpioe.afrl.modify(|_, w| w.afr2().af5());
    // MOSI: PE6
    gpioe.moder.modify(|_, w| w.moder6().alternate());
    gpioe.otyper.modify(|_, w| w.ot6().push_pull());
    gpioe.ospeedr.modify(|_, w| w.ospeedr6().very_high_speed());
    gpioe.afrl.modify(|_, w| w.afr6().af5());
    // MISO: PE5
    // NSS: PE4
    gpioe.moder.modify(|_, w| w.moder4().alternate());
    gpioe.otyper.modify(|_, w| w.ot4().push_pull());
    gpioe.ospeedr.modify(|_, w| w.ospeedr4().very_high_speed());
    gpioe.afrl.modify(|_, w| w.afr4().af5());

    // DAC1_LDAC: PE15
    gpioe.moder.modify(|_, w| w.moder15().output());
    gpioe.otyper.modify(|_, w| w.ot15().push_pull());
    gpioe.odr.modify(|_, w| w.odr15().clear_bit());
}

// ADC0
fn spi1_setup(spi1: &stm32::SPI1) {
    spi1.cfg1.modify(|_, w|
        w.mbr().div4()
         .dsize().bits(16 - 1)
         .fthlv().one_frame()
    );
    spi1.cfg2.modify(|_, w|
        w.afcntr().controlled()
         .ssom().not_asserted()
         .ssoe().enabled()
         .ssiop().active_low()
         .ssm().disabled()
         .cpol().idle_high()
         .cpha().second_edge()
         .lsbfrst().msbfirst()
         .master().master()
         .sp().motorola()
         .comm().receiver()
         .ioswp().disabled()
         .midi().bits(0)
         .mssi().bits(6)
    );
    spi1.cr2.modify(|_, w| w.tsize().bits(1));
    spi1.cr1.write(|w| w.spe().set_bit());
}

// ADC1
fn spi5_setup(spi5: &stm32::SPI5) {
    spi5.cfg1.modify(|_, w|
        w.mbr().div4()
         .dsize().bits(16 - 1)
         .fthlv().one_frame()
    );
    spi5.cfg2.modify(|_, w|
        w.afcntr().controlled()
         .ssom().not_asserted()
         .ssoe().enabled()
         .ssiop().active_low()
         .ssm().disabled()
         .cpol().idle_high()
         .cpha().second_edge()
         .lsbfrst().msbfirst()
         .master().master()
         .sp().motorola()
         .comm().receiver()
         .ioswp().disabled()
         .midi().bits(0)
         .mssi().bits(6)
    );
    spi5.cr2.modify(|_, w| w.tsize().bits(1));
    spi5.cr1.write(|w| w.spe().set_bit());
}

// DAC0
fn spi2_setup(spi2: &stm32::SPI2) {
    spi2.cfg1.modify(|_, w|
        w.mbr().div2()
         .dsize().bits(16 - 1)
         .fthlv().one_frame()
    );
    spi2.cfg2.modify(|_, w|
        w.afcntr().controlled()
         .ssom().not_asserted()
         .ssoe().enabled()
         .ssiop().active_low()
         .ssm().disabled()
         .cpol().idle_low()
         .cpha().first_edge()
         .lsbfrst().msbfirst()
         .master().master()
         .sp().motorola()
         .comm().transmitter()
         .ioswp().disabled()
         .midi().bits(0)
         .mssi().bits(0)
    );
    spi2.cr2.modify(|_, w| w.tsize().bits(0));
    spi2.cr1.write(|w| w.spe().enabled());
    spi2.cr1.modify(|_, w| w.cstart().started());
}

// DAC1
fn spi4_setup(spi4: &stm32::SPI4) {
    spi4.cfg1.modify(|_, w|
        w.mbr().div2()
         .dsize().bits(16 - 1)
         .fthlv().one_frame()
    );
    spi4.cfg2.modify(|_, w|
        w.afcntr().controlled()
         .ssom().not_asserted()
         .ssoe().enabled()
         .ssiop().active_low()
         .ssm().disabled()
         .cpol().idle_low()
         .cpha().first_edge()
         .lsbfrst().msbfirst()
         .master().master()
         .sp().motorola()
         .comm().transmitter()
         .ioswp().disabled()
         .midi().bits(0)
         .mssi().bits(0)
    );
    spi4.cr2.modify(|_, w| w.tsize().bits(0));
    spi4.cr1.write(|w| w.spe().enabled());
    spi4.cr1.modify(|_, w| w.cstart().started());
}

fn tim2_setup(tim2: &stm32::TIM2) {
    tim2.psc.write(|w| unsafe { w.psc().bits(200 - 1) });  // from 200 MHz
    tim2.arr.write(|w| unsafe { w.bits(2 - 1) });  // µs
    tim2.dier.write(|w| w.ude().set_bit());
    tim2.egr.write(|w| w.ug().set_bit());
    tim2.cr1.modify(|_, w|
        w.dir().clear_bit()  // up
         .cen().set_bit());  // enable
}

fn dma1_setup(dma1: &stm32::DMA1, dmamux1: &stm32::DMAMUX1, ma: usize, pa0: usize, pa1: usize) {
    dma1.st[0].cr.modify(|_, w| w.en().clear_bit());
    while dma1.st[0].cr.read().en().bit_is_set() {}

    dma1.st[0].par.write(|w| unsafe { w.bits(pa0 as u32) });
    dma1.st[0].m0ar.write(|w| unsafe { w.bits(ma as u32) });
    dma1.st[0].ndtr.write(|w| unsafe { w.ndt().bits(1) });
    dmamux1.ccr[0].modify(|_, w| w.dmareq_id().tim2_up());
    dma1.st[0].cr.modify(|_, w| unsafe {
        w.pl().bits(0b01)  // medium
         .circ().set_bit()  // reload ndtr
         .msize().bits(0b10)  // 32
         .minc().clear_bit()
         .mburst().bits(0b00)
         .psize().bits(0b10)  // 32
         .pinc().clear_bit()
         .pburst().bits(0b00)
         .dbm().clear_bit()
         .dir().bits(0b01)  // memory_to_peripheral
         .pfctrl().clear_bit()  // dma is FC
    });
    dma1.st[0].fcr.modify(|_, w| w.dmdis().clear_bit());
    dma1.st[0].cr.modify(|_, w| w.en().set_bit());

    dma1.st[1].cr.modify(|_, w| w.en().clear_bit());
    while dma1.st[1].cr.read().en().bit_is_set() {}

    dma1.st[1].par.write(|w| unsafe { w.bits(pa1 as u32) });
    dma1.st[1].m0ar.write(|w| unsafe { w.bits(ma as u32) });
    dma1.st[1].ndtr.write(|w| unsafe { w.ndt().bits(1) });
    dmamux1.ccr[1].modify(|_, w| w.dmareq_id().tim2_up());
    dma1.st[1].cr.modify(|_, w| unsafe {
        w.pl().bits(0b01)  // medium
         .circ().set_bit()  // reload ndtr
         .msize().bits(0b10)  // 32
         .minc().clear_bit()
         .mburst().bits(0b00)
         .psize().bits(0b10)  // 32
         .pinc().clear_bit()
         .pburst().bits(0b00)
         .dbm().clear_bit()
         .dir().bits(0b01)  // memory_to_peripheral
         .pfctrl().clear_bit()  // dma is FC
    });
    dma1.st[1].fcr.modify(|_, w| w.dmdis().clear_bit());
    dma1.st[1].cr.modify(|_, w| w.en().set_bit());
}

type SpiPs = Option<(stm32::SPI1, stm32::SPI2, stm32::SPI4, stm32::SPI5)>;
static SPIP: Mutex<RefCell<SpiPs>> = Mutex::new(RefCell::new(None));

#[link_section = ".sram1.datspi"]
static mut DAT: u32 = 0x201;  // EN | CSTART

static TIME: AtomicU32 = AtomicU32::new(0);
static ETHERNET_PENDING: AtomicBool = AtomicBool::new(true);

#[link_section = ".sram3.eth"]
static mut ETHERNET: eth::Device = eth::Device::new();

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

macro_rules! create_socket {
    ($set:ident, $rx_storage:ident, $tx_storage:ident, $target:ident) => (
        let mut $rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut $tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_rx_buffer = net::socket::TcpSocketBuffer::new(&mut $rx_storage[..]);
        let tcp_tx_buffer = net::socket::TcpSocketBuffer::new(&mut $tx_storage[..]);
        let tcp_socket = net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
        let $target = $set.add(tcp_socket);
    )
}


#[entry]
fn main() -> ! {
    let mut cp = CorePeripherals::take().unwrap();
    let dp = Peripherals::take().unwrap();

    let rcc = dp.RCC;
    rcc_reset(&rcc);

    init_log();
    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    // info!("Built on {}", build_info::BUILT_TIME_UTC);
    // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

    pwr_setup(&dp.PWR);
    rcc_pll_setup(&rcc, &dp.FLASH);
    rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
    io_compensation_setup(&dp.SYSCFG);

    // 100 MHz
    cp.SYST.set_clock_source(cortex_m::peripheral::syst::SystClkSource::Core);
    cp.SYST.set_reload(cortex_m::peripheral::SYST::get_ticks_per_10ms()*200/10);
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();
    unsafe { cp.SCB.shpr[11].write(128); }  // systick exception priority

    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter();

    rcc.ahb4enr.modify(|_, w|
        w.gpioaen().set_bit()
         .gpioben().set_bit()
         .gpiocen().set_bit()
         .gpioden().set_bit()
         .gpioeen().set_bit()
         .gpiofen().set_bit()
         .gpiogen().set_bit()
    );
    gpio_setup(&dp.GPIOA, &dp.GPIOB, &dp.GPIOD, &dp.GPIOE, &dp.GPIOF, &dp.GPIOG);

    rcc.apb1lenr.modify(|_, w| w.spi2en().set_bit());
    let spi2 = dp.SPI2;
    spi2_setup(&spi2);

    rcc.apb2enr.modify(|_, w| w.spi4en().set_bit());
    let spi4 = dp.SPI4;
    spi4_setup(&spi4);

    rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());
    let spi1 = dp.SPI1;
    spi1_setup(&spi1);
    spi1.ier.write(|w| w.eotie().set_bit());

    rcc.apb2enr.modify(|_, w| w.spi5en().set_bit());
    let spi5 = dp.SPI5;
    spi5_setup(&spi5);
    // spi5.ier.write(|w| w.eotie().set_bit());

    rcc.ahb2enr.modify(|_, w|
        w
            .sram1en().set_bit()
            .sram2en().set_bit()
            .sram3en().set_bit()
    );
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    // init SRAM1 rodata can't load with sram1 disabled
    unsafe { DAT = 0x201 };  // EN | CSTART
    cortex_m::asm::dsb();
    let dat_addr = unsafe { &DAT as *const _ } as usize;
    cp.SCB.clean_dcache_by_address(dat_addr, 4);

    dma1_setup(&dp.DMA1, &dp.DMAMUX1, dat_addr,
               &spi1.cr1 as *const _ as usize,
               &spi5.cr1 as *const _ as usize);

    rcc.apb1lenr.modify(|_, w| w.tim2en().set_bit());

    // work around the SPI stall erratum
    let dbgmcu = dp.DBGMCU;
    dbgmcu.apb1lfz1.modify(|_, w| w.tim2().set_bit());

    unsafe {
        let t = 2e-6*2.;
        IIR_CH[0].set_pi(1., 0., 0.).unwrap();
        IIR_CH[0].set_x_offset(0.*SCALE);

        IIR_CH[1].set_pi(-0.1, -10.*t, 0.).unwrap();
        IIR_CH[1].set_x_offset(0.1*SCALE);
        IIR_CH[1].get_x_offset().unwrap();
    }

    eth::setup(&rcc, &dp.SYSCFG);
    eth::setup_pins(&dp.GPIOA, &dp.GPIOB, &dp.GPIOC, &dp.GPIOG);

    let device = unsafe { &mut ETHERNET };
    let hardware_addr = net::wire::EthernetAddress([0x10, 0xE2, 0xD5, 0x00, 0x03, 0x00]);
    unsafe { device.init(hardware_addr, &dp.ETHERNET_MAC, &dp.ETHERNET_DMA, &dp.ETHERNET_MTL) };
    let mut neighbor_cache_storage = [None; 8];
    let neighbor_cache = net::iface::NeighborCache::new(&mut neighbor_cache_storage[..]);
    let local_addr = net::wire::IpAddress::v4(10, 0, 16, 99);
    let mut ip_addrs = [net::wire::IpCidr::new(local_addr, 24)];
    let mut iface = net::iface::EthernetInterfaceBuilder::new(device)
                .ethernet_addr(hardware_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut ip_addrs[..])
                .finalize();
    let mut socket_set_entries: [_; 8] = Default::default();
    let mut sockets = net::socket::SocketSet::new(&mut socket_set_entries[..]);
    create_socket!(sockets, tcp_rx_storage0, tcp_tx_storage0, tcp_handle0);
    create_socket!(sockets, tcp_rx_storage0, tcp_tx_storage0, tcp_handle1);

    unsafe { eth::enable_interrupt(&dp.ETHERNET_DMA); }
    unsafe { cp.NVIC.set_priority(stm32::Interrupt::ETH, 196); }  // mid prio
    cp.NVIC.enable(stm32::Interrupt::ETH);

    tim2_setup(&dp.TIM2);

    unsafe { cp.NVIC.set_priority(stm32::Interrupt::SPI1, 0); }  // highest prio
    cortex_m::interrupt::free(|cs| {
        cp.NVIC.enable(stm32::Interrupt::SPI1);
        SPIP.borrow(cs).replace(Some((spi1, spi2, spi4, spi5)));
    });

    let mut last = 0;
    loop {
        // if ETHERNET_PENDING.swap(false, Ordering::Relaxed) { }
        let time = TIME.load(Ordering::Relaxed);
        {
            let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle0);
            if !(socket.is_open() || socket.is_listening()) {
                socket.listen(1234).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
            } else if last != time && socket.can_send() {
                last = time;
                let s = unsafe { Status{
                    t: time,
                    x0: IIR_STATE[0][0],
                    y0: IIR_STATE[0][2],
                    x1: IIR_STATE[1][0],
                    y1: IIR_STATE[1][2],
                }};
                send_response(socket, &s);
            }
        }
        {
            let socket = &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle1);
            if !(socket.is_open() || socket.is_listening()) {
                socket.listen(1235).unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
            } else {
                handle_command(socket);
            }
        }

        if !match iface.poll(&mut sockets, net::time::Instant::from_millis(time as i64)) {
            Ok(changed) => changed,
            Err(net::Error::Unrecognized) => true,
            Err(e) => { info!("iface poll error: {:?}", e); true }
        } {
            cortex_m::asm::wfi();
        }
    }
}

#[derive(Deserialize,Serialize)]
struct Request {
    channel: i8,
    iir: IIR,
}

#[derive(Serialize)]
struct Response<'a> {
    code: i32,
    message: &'a str,
}

fn send_response<T: Serialize>(socket: &mut net::socket::TcpSocket, msg: &T) {
    let mut u: String<U128> = to_string(msg).unwrap();
    u.push('\n').unwrap();
    socket.write_str(&u).unwrap();
}

fn handle_command(socket: &mut net::socket::TcpSocket) {
    let mut data: Vec<u8, U256> = Vec::new();
    let mut discard: bool = false;
    while socket.can_recv() {
        if socket.recv(|buf| {
            let (len, found) = match buf.iter().position(|&c| c as char == '\n') {
                Some(end) => (end + 1, true),
                None => (buf.len(), false),
            };
            if data.len() + len >= data.capacity() {
                discard = true;
                data.clear();
            } else if !discard && len > 0 {
                data.extend_from_slice(&buf[..len - 1]).unwrap();
            }
            (len, found)
        }).unwrap() {
            let resp = if discard {
                discard = false;
                Response{ code: 500, message: "command buffer overflow" }
            } else {
                match from_slice::<Request>(&data) {
                    Ok(request) => {
                        cortex_m::interrupt::free(|_| {
                            unsafe { IIR_CH[request.channel as usize] = request.iir; };
                        });
                        Response{ code: 200, message: "ok" }
                    },
                    Err(err) => {
                        warn!("parse error {}", err);
                        Response{ code: 550, message: "parse error" }
                    },
                }
            };
            send_response(socket, &resp);
            socket.close();
        }
    }
}

#[derive(Serialize)]
struct Status {
    t: u32,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32
}

const SCALE: f32 = ((1 << 15) - 1) as f32;
static mut IIR_STATE: [IIRState; 2] = [[0.; 5]; 2];
static mut IIR_CH: [IIR; 2] = [
    IIR{ ba: [0., 0., 0., 0., 0.], y_offset: 0.,
         y_min: -SCALE, y_max: SCALE }; 2];

// seems to slow it down
// #[link_section = ".data.spi1"]
#[interrupt]
fn SPI1() {
    #[cfg(feature = "bkpt")]
    cortex_m::asm::bkpt();
    cortex_m::interrupt::free(|cs| {
        let spip = SPIP.borrow(cs).borrow();
        let (spi1, spi2, spi4, spi5) = spip.as_ref().unwrap();

        let sr = spi1.sr.read();
        if sr.eot().bit_is_set() {
           spi1.ifcr.write(|w| w.eotc().set_bit());
        }
        if sr.rxp().bit_is_set() {
            let rxdr = &spi1.rxdr as *const _ as *const u16;
            let a = unsafe { ptr::read_volatile(rxdr) };
            let x0 = f32::from(a as i16);
            let y0 = unsafe { IIR_CH[0].update(&mut IIR_STATE[0], x0) };
            let d = y0 as i16 as u16 ^ 0x8000;
            let txdr = &spi2.txdr as *const _ as *mut u16;
            unsafe { ptr::write_volatile(txdr, d) };
        }

        let sr = spi5.sr.read();
        if sr.eot().bit_is_set() {
           spi5.ifcr.write(|w| w.eotc().set_bit());
        }
        if sr.rxp().bit_is_set() {
            let rxdr = &spi5.rxdr as *const _ as *const u16;
            let a = unsafe { ptr::read_volatile(rxdr) };
            let x0 = f32::from(a as i16);
            let y0 = unsafe { IIR_CH[1].update(&mut IIR_STATE[1], x0) };
            let d = y0 as i16 as u16 ^ 0x8000;
            let txdr = &spi4.txdr as *const _ as *mut u16;
            unsafe { ptr::write_volatile(txdr, d) };
        }
    });
    #[cfg(feature = "bkpt")]
    cortex_m::asm::bkpt();
}

#[interrupt]
fn ETH() {
    let dma = unsafe { &stm32::Peripherals::steal().ETHERNET_DMA };
    ETHERNET_PENDING.store(true, Ordering::Relaxed);
    unsafe { eth::interrupt_handler(dma) }
}

#[exception]
fn SysTick() {
    TIME.fetch_add(1, Ordering::Relaxed);
}

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
