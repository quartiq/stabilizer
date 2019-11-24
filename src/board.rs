use super::eth;
use super::i2c;
use super::pac;

fn pwr_setup(pwr: &pac::PWR) {
    // go to VOS1 voltage scale for high perf
    pwr.cr3
        .write(|w| w.scuen().set_bit().ldoen().set_bit().bypass().clear_bit());
    while pwr.csr1.read().actvosrdy().bit_is_clear() {}
    pwr.d3cr.write(|w| unsafe { w.vos().bits(0b11) }); // vos1
    while pwr.d3cr.read().vosrdy().bit_is_clear() {}
}

fn rcc_reset(rcc: &pac::RCC) {
    // Reset all peripherals
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb1rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1lrstr.write(|w| unsafe { w.bits(0) });
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb1hrstr.write(|w| unsafe { w.bits(0) });

    rcc.ahb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb2rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb2rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb2rstr.write(|w| unsafe { w.bits(0) });

    // do not reset the cpu
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0x7FFF_FFFF) });
    rcc.ahb3rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb3rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb3rstr.write(|w| unsafe { w.bits(0) });

    rcc.ahb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.ahb4rstr.write(|w| unsafe { w.bits(0) });
    rcc.apb4rstr.write(|w| unsafe { w.bits(0xFFFF_FFFF) });
    rcc.apb4rstr.write(|w| unsafe { w.bits(0) });
}

fn rcc_pll_setup(rcc: &pac::RCC, flash: &pac::FLASH) {
    // Switch to HSI to mess with HSE
    rcc.cr.modify(|_, w| w.hsion().on());
    while rcc.cr.read().hsirdy().is_not_ready() {}
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}
    rcc.cr.write(|w| w.hsion().on());
    rcc.cfgr.reset();

    // Ensure HSE is on and stable
    rcc.cr.modify(|_, w| w.hseon().on().hsebyp().not_bypassed());
    while !rcc.cr.read().hserdy().is_ready() {}

    rcc.pllckselr.modify(
        |_, w| {
            w.pllsrc()
                .hse()
                .divm1()
                .bits(1) // ref prescaler
                .divm2()
                .bits(1)
        }, // ref prescaler
    );
    // Configure PLL1: 8MHz /1 *100 /2 = 400 MHz
    rcc.pllcfgr.modify(|_, w| {
        w.pll1vcosel()
            .wide_vco() // 192-836 MHz VCO
            .pll1rge()
            .range8() // 8-16 MHz PFD
            .pll1fracen()
            .reset()
            .divp1en()
            .enabled()
            .pll2vcosel()
            .medium_vco() // 150-420 MHz VCO
            .pll2rge()
            .range8() // 8-16 MHz PFD
            .pll2fracen()
            .reset()
            .divp2en()
            .enabled()
            .divq2en()
            .enabled()
    });
    rcc.pll1divr.write(|w| unsafe {
        w.divn1()
            .bits(100 - 1) // feebdack divider
            .divp1()
            .div2() // p output divider
    });
    rcc.cr.modify(|_, w| w.pll1on().on());
    while !rcc.cr.read().pll1rdy().is_ready() {}

    // Configure PLL2: 8MHz /1 *25 / 2 = 100 MHz
    rcc.pll2divr.write(|w| unsafe {
        w.divn2()
            .bits(25 - 1) // feebdack divider
            .divp2()
            .bits(2 - 1) // p output divider
            .divq2()
            .bits(2 - 1) // q output divider
    });
    rcc.cr.modify(|_, w| w.pll2on().on());
    while !rcc.cr.read().pll2rdy().is_ready() {}

    // hclk 200 MHz, pclk 100 MHz
    rcc.d1cfgr.write(
        |w| {
            w.d1cpre()
                .div1() // sys_ck not divided
                .hpre()
                .div2() // rcc_hclk3 = sys_d1cpre_ck / 2
                .d1ppre()
                .div2()
        }, // rcc_pclk3 = rcc_hclk3 / 2
    );
    rcc.d2cfgr.write(
        |w| {
            w.d2ppre1()
                .div2() // rcc_pclk1 = rcc_hclk3 / 2
                .d2ppre2()
                .div2()
        }, // rcc_pclk2 = rcc_hclk3 / 2
    );
    rcc.d3cfgr.write(
        |w| w.d3ppre().div2(), // rcc_pclk4 = rcc_hclk3 / 2
    );

    // 2 wait states, 0b10 programming delay
    // 185-210 MHz
    flash
        .acr
        .write(|w| unsafe { w.wrhighfreq().bits(2).latency().bits(2) });
    while flash.acr.read().latency().bits() != 2 {}

    // CSI for I/O compensationc ell
    rcc.cr.modify(|_, w| w.csion().on());
    while !rcc.cr.read().csirdy().is_ready() {}

    // Set system clock to pll1_p
    rcc.cfgr.modify(|_, w| w.sw().pll1());
    while !rcc.cfgr.read().sws().is_pll1() {}

    rcc.d1ccipr.write(|w| w.ckpersel().hse());
    rcc.d2ccip1r
        .modify(|_, w| w.spi123sel().pll2_p().spi45sel().pll2_q());
    rcc.d3ccipr.modify(|_, w| w.spi6sel().pll2_q());
}

fn io_compensation_setup(syscfg: &pac::SYSCFG) {
    syscfg
        .cccsr
        .modify(|_, w| w.en().set_bit().cs().clear_bit().hslv().clear_bit());
    while syscfg.cccsr.read().ready().bit_is_clear() {}
}

fn gpio_setup(
    gpioa: &pac::GPIOA,
    gpiob: &pac::GPIOB,
    gpiod: &pac::GPIOD,
    gpioe: &pac::GPIOE,
    gpiof: &pac::GPIOF,
    gpiog: &pac::GPIOG,
) {
    // FP_LED0
    gpiod.otyper.modify(|_, w| w.ot5().push_pull());
    gpiod.moder.modify(|_, w| w.moder5().output());
    gpiod.odr.modify(|_, w| w.odr5().low());

    // FP_LED1
    gpiod.otyper.modify(|_, w| w.ot6().push_pull());
    gpiod.moder.modify(|_, w| w.moder6().output());
    gpiod.odr.modify(|_, w| w.odr6().low());

    // LED_FP2
    gpiog.otyper.modify(|_, w| w.ot4().push_pull());
    gpiog.moder.modify(|_, w| w.moder4().output());
    gpiog.odr.modify(|_, w| w.odr4().low());

    // LED_FP3
    gpiod.otyper.modify(|_, w| w.ot12().push_pull());
    gpiod.moder.modify(|_, w| w.moder12().output());
    gpiod.odr.modify(|_, w| w.odr12().low());

    // AFE0_A0,1: PG2,PG3
    gpiog
        .otyper
        .modify(|_, w| w.ot2().push_pull().ot3().push_pull());
    gpiog
        .moder
        .modify(|_, w| w.moder2().output().moder3().output());
    gpiog.odr.modify(|_, w| w.odr2().low().odr3().low());

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
    gpioe.odr.modify(|_, w| w.odr11().low());

    // DAC_CLR: PE12
    gpioe.moder.modify(|_, w| w.moder12().output());
    gpioe.otyper.modify(|_, w| w.ot12().push_pull());
    gpioe.odr.modify(|_, w| w.odr12().high());

    // AFE1_A0,1: PD14,PD15
    gpiod
        .otyper
        .modify(|_, w| w.ot14().push_pull().ot15().push_pull());
    gpiod
        .moder
        .modify(|_, w| w.moder14().output().moder15().output());
    gpiod.odr.modify(|_, w| w.odr14().low().odr15().low());

    // I2C2: SDA,SCL: PF0,PF1
    gpiof
        .moder
        .modify(|_, w| w.moder0().alternate().moder1().alternate());
    gpiof.afrl.modify(|_, w| w.afr0().af4().afr1().af4());
    gpiof
        .otyper
        .modify(|_, w| w.ot0().open_drain().ot1().open_drain());

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
    gpioe.odr.modify(|_, w| w.odr15().low());
}

// ADC0
fn spi1_setup(spi1: &pac::SPI1) {
    spi1.cfg1
        .modify(|_, w| w.mbr().div4().dsize().bits(16 - 1).fthlv().one_frame());
    spi1.cfg2.modify(|_, w| {
        w.afcntr()
            .controlled()
            .ssom()
            .not_asserted()
            .ssoe()
            .enabled()
            .ssiop()
            .active_low()
            .ssm()
            .disabled()
            .cpol()
            .idle_high()
            .cpha()
            .second_edge()
            .lsbfrst()
            .msbfirst()
            .master()
            .master()
            .sp()
            .motorola()
            .comm()
            .receiver()
            .ioswp()
            .disabled()
            .midi()
            .bits(0)
            .mssi()
            .bits(6)
    });
    spi1.cr2.modify(|_, w| w.tsize().bits(1));
    spi1.cr1.write(|w| w.spe().enabled());
}

// ADC1
fn spi5_setup(spi5: &pac::SPI5) {
    spi5.cfg1
        .modify(|_, w| w.mbr().div4().dsize().bits(16 - 1).fthlv().one_frame());
    spi5.cfg2.modify(|_, w| {
        w.afcntr()
            .controlled()
            .ssom()
            .not_asserted()
            .ssoe()
            .enabled()
            .ssiop()
            .active_low()
            .ssm()
            .disabled()
            .cpol()
            .idle_high()
            .cpha()
            .second_edge()
            .lsbfrst()
            .msbfirst()
            .master()
            .master()
            .sp()
            .motorola()
            .comm()
            .receiver()
            .ioswp()
            .disabled()
            .midi()
            .bits(0)
            .mssi()
            .bits(6)
    });
    spi5.cr2.modify(|_, w| w.tsize().bits(1));
    spi5.cr1.write(|w| w.spe().enabled());
}

// DAC0
fn spi2_setup(spi2: &pac::SPI2) {
    spi2.cfg1
        .modify(|_, w| w.mbr().div2().dsize().bits(16 - 1).fthlv().one_frame());
    spi2.cfg2.modify(|_, w| {
        w.afcntr()
            .controlled()
            .ssom()
            .not_asserted()
            .ssoe()
            .enabled()
            .ssiop()
            .active_low()
            .ssm()
            .disabled()
            .cpol()
            .idle_low()
            .cpha()
            .first_edge()
            .lsbfrst()
            .msbfirst()
            .master()
            .master()
            .sp()
            .motorola()
            .comm()
            .transmitter()
            .ioswp()
            .disabled()
            .midi()
            .bits(0)
            .mssi()
            .bits(0)
    });
    spi2.cr2.modify(|_, w| w.tsize().bits(0));
    spi2.cr1.write(|w| w.spe().enabled());
    spi2.cr1.modify(|_, w| w.cstart().started());
}

// DAC1
fn spi4_setup(spi4: &pac::SPI4) {
    spi4.cfg1
        .modify(|_, w| w.mbr().div2().dsize().bits(16 - 1).fthlv().one_frame());
    spi4.cfg2.modify(|_, w| {
        w.afcntr()
            .controlled()
            .ssom()
            .not_asserted()
            .ssoe()
            .enabled()
            .ssiop()
            .active_low()
            .ssm()
            .disabled()
            .cpol()
            .idle_low()
            .cpha()
            .first_edge()
            .lsbfrst()
            .msbfirst()
            .master()
            .master()
            .sp()
            .motorola()
            .comm()
            .transmitter()
            .ioswp()
            .disabled()
            .midi()
            .bits(0)
            .mssi()
            .bits(0)
    });
    spi4.cr2.modify(|_, w| w.tsize().bits(0));
    spi4.cr1.write(|w| w.spe().enabled());
    spi4.cr1.modify(|_, w| w.cstart().started());
}

fn tim2_setup(tim2: &pac::TIM2) {
    tim2.psc.write(|w| w.psc().bits(200 - 1)); // from 200 MHz
    tim2.arr.write(|w| unsafe { w.bits(2 - 1) }); // 2 µs
    tim2.dier.write(|w| w.ude().set_bit());
    tim2.egr.write(|w| w.ug().set_bit());
    tim2.cr1.modify(|_, w| w.dir().clear_bit()); // up
}

fn dma1_setup(
    dma1: &pac::DMA1,
    dmamux1: &pac::DMAMUX1,
    ma: usize,
    pa0: usize,
    pa1: usize,
) {
    dma1.st[0].cr.modify(|_, w| w.en().clear_bit());
    while dma1.st[0].cr.read().en().bit_is_set() {}

    dma1.st[0].par.write(|w| unsafe { w.bits(pa0 as u32) });
    dma1.st[0].m0ar.write(|w| unsafe { w.bits(ma as u32) });
    dma1.st[0].ndtr.write(|w| unsafe { w.ndt().bits(1) });
    dmamux1.ccr[0].modify(|_, w| w.dmareq_id().tim2_up());
    dma1.st[0].cr.modify(|_, w| unsafe {
        w.pl()
            .bits(0b01) // medium
            .circ()
            .set_bit() // reload ndtr
            .msize()
            .bits(0b10) // 32
            .minc()
            .clear_bit()
            .mburst()
            .bits(0b00)
            .psize()
            .bits(0b10) // 32
            .pinc()
            .clear_bit()
            .pburst()
            .bits(0b00)
            .dbm()
            .clear_bit()
            .dir()
            .bits(0b01) // memory_to_peripheral
            .pfctrl()
            .clear_bit() // dma is FC
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
        w.pl()
            .bits(0b01) // medium
            .circ()
            .set_bit() // reload ndtr
            .msize()
            .bits(0b10) // 32
            .minc()
            .clear_bit()
            .mburst()
            .bits(0b00)
            .psize()
            .bits(0b10) // 32
            .pinc()
            .clear_bit()
            .pburst()
            .bits(0b00)
            .dbm()
            .clear_bit()
            .dir()
            .bits(0b01) // memory_to_peripheral
            .pfctrl()
            .clear_bit() // dma is FC
    });
    dma1.st[1].fcr.modify(|_, w| w.dmdis().clear_bit());
    dma1.st[1].cr.modify(|_, w| w.en().set_bit());
}

#[link_section = ".sram1.datspi"]
static mut DAT: u32 = 0x201; // EN | CSTART

pub fn init() {
    let mut cp = unsafe { cortex_m::Peripherals::steal() };
    let dp = unsafe { pac::Peripherals::steal() };

    let rcc = dp.RCC;
    rcc_reset(&rcc);
    pwr_setup(&dp.PWR);
    rcc_pll_setup(&rcc, &dp.FLASH);
    rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
    io_compensation_setup(&dp.SYSCFG);

    cp.SCB.enable_icache();
    // TODO: ETH DMA coherence issues
    // cp.SCB.enable_dcache(&mut cp.CPUID);
    cp.DWT.enable_cycle_counter(); // japaric/cortex-m-rtfm#184

    rcc.ahb4enr.modify(|_, w| {
        w.gpioaen()
            .set_bit()
            .gpioben()
            .set_bit()
            .gpiocen()
            .set_bit()
            .gpioden()
            .set_bit()
            .gpioeen()
            .set_bit()
            .gpiofen()
            .set_bit()
            .gpiogen()
            .set_bit()
    });
    gpio_setup(
        &dp.GPIOA, &dp.GPIOB, &dp.GPIOD, &dp.GPIOE, &dp.GPIOF, &dp.GPIOG,
    );

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

    rcc.ahb2enr.modify(|_, w| {
        w.sram1en()
            .set_bit()
            .sram2en()
            .set_bit()
            .sram3en()
            .set_bit()
    });
    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
    // init SRAM1 rodata can't load with sram1 disabled
    unsafe { DAT = 0x201 }; // EN | CSTART
    cortex_m::asm::dsb();
    let dat_addr = unsafe { &DAT as *const _ } as usize;
    cp.SCB.clean_dcache_by_address(dat_addr, 4);

    dma1_setup(
        &dp.DMA1,
        &dp.DMAMUX1,
        dat_addr,
        &spi1.cr1 as *const _ as usize,
        &spi5.cr1 as *const _ as usize,
    );

    rcc.apb1lenr.modify(|_, w| w.tim2en().set_bit());

    // work around the SPI stall erratum
    let dbgmcu = dp.DBGMCU;
    dbgmcu.apb1lfz1.modify(|_, w| w.tim2().set_bit());

    tim2_setup(&dp.TIM2);

    rcc.apb1lenr.modify(|_, w| w.i2c2en().set_bit());
    i2c::setup(&dp.I2C2);

    rcc.apb4enr.modify(|_, w| w.syscfgen().set_bit());
    rcc.ahb1enr.modify(|_, w| {
        w.eth1macen()
            .set_bit()
            .eth1txen()
            .set_bit()
            .eth1rxen()
            .set_bit()
    });
    dp.SYSCFG
        .pmcr
        .modify(|_, w| unsafe { w.epis().bits(0b100) }); // RMII
    eth::setup_pins(&dp.GPIOA, &dp.GPIOB, &dp.GPIOC, &dp.GPIOG);

    // enable TIM2 this must be late to be able to handle the first ADC SPI
    // interrupt in time
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit());
}
