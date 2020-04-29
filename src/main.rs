#![deny(warnings)]
#![allow(clippy::missing_safety_doc)]
#![no_std]
#![no_main]
#![cfg_attr(feature = "nightly", feature(asm))]
// Enable returning `!`
#![cfg_attr(feature = "nightly", feature(never_type))]
#![cfg_attr(feature = "nightly", feature(core_intrinsics))]

#[inline(never)]
#[panic_handler]
#[cfg(all(feature = "nightly", not(feature = "semihosting")))]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    let gpiod = unsafe { &*pac::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high()); // FP_LED_1, FP_LED_3
    unsafe {
        core::intrinsics::abort();
    }
}

#[cfg(feature = "semihosting")]
extern crate panic_semihosting;

#[cfg(not(any(feature = "nightly", feature = "semihosting")))]
extern crate panic_halt;

#[macro_use]
extern crate log;

use nb;

// use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use asm_delay;
use rtfm::cyccnt::{Instant, U32Ext};
use cortex_m_rt::exception;
use cortex_m;
use stm32h7xx_hal as hal;
use stm32h7xx_hal::{
    prelude::*,
    stm32 as pac,
};

use embedded_hal::{
    digital::v2::OutputPin,
};

use stm32h7_ethernet as ethernet;
use smoltcp as net;

#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

mod eth;
mod pounder;
mod server;
mod afe;

mod iir;
use iir::*;

mod eeprom;

#[cfg(not(feature = "semihosting"))]
fn init_log() {}

#[cfg(feature = "semihosting")]
fn init_log() {
    use cortex_m_log::log::{init as init_log, Logger};
    use cortex_m_log::printer::semihosting::{hio::HStdout, InterruptOk};
    use log::LevelFilter;
    static mut LOGGER: Option<Logger<InterruptOk<HStdout>>> = None;
    let logger = Logger {
        inner: InterruptOk::<_>::stdout().unwrap(),
        level: LevelFilter::Info,
    };
    let logger = unsafe { LOGGER.get_or_insert(logger) };

    init_log(logger).unwrap();
}

// Pull in build information (from `built` crate)
mod build_info {
    #![allow(dead_code)]
    // include!(concat!(env!("OUT_DIR"), "/built.rs"));
}

pub struct NetStorage {
    ip_addrs: [net::wire::IpCidr; 1],
    neighbor_cache: [Option<(net::wire::IpAddress, net::iface::Neighbor)>; 8],
}

static mut NET_STORE: NetStorage = NetStorage {
    // Placeholder for the real IP address, which is initialized at runtime.
    ip_addrs: [net::wire::IpCidr::Ipv6(net::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX)],

    neighbor_cache: [None; 8],
};

const SCALE: f32 = ((1 << 15) - 1) as f32;

// static ETHERNET_PENDING: AtomicBool = AtomicBool::new(true);

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

type AFE1 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiof::PF2<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiof::PF5<hal::gpio::Output<hal::gpio::PushPull>>>;

type AFE2 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiod::PD15<hal::gpio::Output<hal::gpio::PushPull>>>;

#[rtfm::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        adc1: hal::spi::Spi<hal::stm32::SPI2>,
        dac1: hal::spi::Spi<hal::stm32::SPI4>,
        _afe1: AFE1,

        adc2: hal::spi::Spi<hal::stm32::SPI3>,
        dac2: hal::spi::Spi<hal::stm32::SPI5>,
        _afe2: AFE2,

        eeprom_i2c: hal::i2c::I2c<hal::stm32::I2C2>,

        dbg_pin: hal::gpio::gpioc::PC6<hal::gpio::Output<hal::gpio::PushPull>>,
        dac_pin: hal::gpio::gpiob::PB15<hal::gpio::Output<hal::gpio::PushPull>>,
        timer: hal::timer::Timer<hal::stm32::TIM2>,
        net_interface: net::iface::EthernetInterface<'static, 'static, 'static,
                                                     ethernet::EthernetDMA<'static>>,
        _eth_mac: ethernet::EthernetMAC,
        mac_addr: net::wire::EthernetAddress,

        pounder: pounder::PounderDevices<asm_delay::AsmDelay>,

        #[init([[0.; 5]; 2])]
        iir_state: [IIRState; 2],
        #[init([IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE }; 2])]
        iir_ch: [IIR; 2],
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        let dp = c.device;
        let mut cp = cortex_m::Peripherals::take().unwrap();

        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();

        let rcc = dp.RCC.constrain();
        let mut clocks = rcc
            //TODO: Re-enable HSE for Stabilizer platform.
//            .use_hse(8.mhz())
            .sysclk(400.mhz())
            .hclk(200.mhz())
            .per_ck(100.mhz())
            .pll2_p_ck(100.mhz())
            .pll2_q_ck(100.mhz())
            .freeze(vos, &dp.SYSCFG);

        // Enable SRAM3 for the ethernet descriptor ring.
        clocks.rb.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        clocks.rb.rsr.write(|w| w.rmvf().set_bit());

        clocks.rb.d2ccip1r.modify(|_, w| w.spi123sel().pll2_p().spi45sel().pll2_q());

        let gpioa = dp.GPIOA.split(&mut clocks.ahb4);
        let gpiob = dp.GPIOB.split(&mut clocks.ahb4);
        let gpioc = dp.GPIOC.split(&mut clocks.ahb4);
        let gpiod = dp.GPIOD.split(&mut clocks.ahb4);
        let gpioe = dp.GPIOE.split(&mut clocks.ahb4);
        let gpiof = dp.GPIOF.split(&mut clocks.ahb4);
        let gpiog = dp.GPIOG.split(&mut clocks.ahb4);

        let afe1 = {
            let a0_pin = gpiof.pf2.into_push_pull_output();
            let a1_pin = gpiof.pf5.into_push_pull_output();
            afe::ProgrammableGainAmplifier::new(a0_pin, a1_pin)
        };

        let afe2 = {
            let a0_pin = gpiod.pd14.into_push_pull_output();
            let a1_pin = gpiod.pd15.into_push_pull_output();
            afe::ProgrammableGainAmplifier::new(a0_pin, a1_pin)
        };

        // Configure the SPI interfaces to the ADCs and DACs.
        let adc1_spi = {
            let spi_miso = gpiob.pb14.into_alternate_af5().set_speed(hal::gpio::Speed::VeryHigh);
            let spi_sck = gpiob.pb10.into_alternate_af5().set_speed(hal::gpio::Speed::VeryHigh);
            let _spi_nss = gpiob.pb9.into_alternate_af5();

            let config = hal::spi::Config::new(hal::spi::Mode{
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .communication_mode(hal::spi::CommunicationMode::Receiver)
                .manage_cs()
                .cs_delay(220e-9)
                .frame_size(16);

            let mut spi = dp.SPI2.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    50.mhz(),
                    &clocks);

            spi.listen(hal::spi::Event::Rxp);

            spi
        };

        let adc2_spi = {
            let spi_miso = gpiob.pb4.into_alternate_af6().set_speed(hal::gpio::Speed::VeryHigh);
            let spi_sck = gpioc.pc10.into_alternate_af6().set_speed(hal::gpio::Speed::VeryHigh);
            let _spi_nss = gpioa.pa15.into_alternate_af6();


            let config = hal::spi::Config::new(hal::spi::Mode{
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .communication_mode(hal::spi::CommunicationMode::Receiver)
                .manage_cs()
                .frame_size(16)
                .cs_delay(220e-9);

            let spi = dp.SPI3.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    50.mhz(),
                    &clocks);

            spi
        };

        let dac1_spi = {
            let spi_miso = gpioe.pe5.into_alternate_af5();
            let spi_sck = gpioe.pe2.into_alternate_af5();
            let _spi_nss = gpioe.pe4.into_alternate_af5();

            let config = hal::spi::Config::new(hal::spi::Mode{
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .communication_mode(hal::spi::CommunicationMode::Transmitter)
                .manage_cs()
                .frame_size(16)
                .swap_mosi_miso();

            dp.SPI4.spi((spi_sck, spi_miso, hal::spi::NoMosi), config, 25.mhz(), &clocks)
        };

        let dac2_spi = {
            let spi_miso = gpiof.pf8.into_alternate_af5();
            let spi_sck = gpiof.pf7.into_alternate_af5();
            let _spi_nss = gpiof.pf6.into_alternate_af5();

            let config = hal::spi::Config::new(hal::spi::Mode{
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .communication_mode(hal::spi::CommunicationMode::Transmitter)
                .manage_cs()
                .frame_size(16)
                .swap_mosi_miso();

            dp.SPI5.spi((spi_sck, spi_miso, hal::spi::NoMosi), config, 25.mhz(), &clocks)
        };

        let pounder_devices = {
            let ad9959 =  {
                let qspi_interface = {
                    // Instantiate the QUADSPI pins and peripheral interface.
                    // TODO: Place these into a pins structure that is provided to the QSPI
                    // constructor.
                    let _qspi_clk = gpiob.pb2.into_alternate_af9();
                    let _qspi_ncs = gpioc.pc11.into_alternate_af9();
                    let _qspi_io0 = gpioe.pe7.into_alternate_af10();
                    let _qspi_io1 = gpioe.pe8.into_alternate_af10();
                    let _qspi_io2 = gpioe.pe9.into_alternate_af10();
                    let _qspi_io3 = gpioe.pe10.into_alternate_af10();

                    let qspi = hal::qspi::Qspi::new(dp.QUADSPI, &mut clocks, 10.mhz()).unwrap();
                    pounder::QspiInterface {qspi}
                };

                let mut reset_pin = gpioa.pa0.into_push_pull_output();
                let io_update = gpiog.pg7.into_push_pull_output();


                let delay = {
                    let frequency_hz = clocks.clocks.c_ck().0;
                    asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz (frequency_hz))
                };

                ad9959::Ad9959::new(qspi_interface,
                                    &mut reset_pin,
                                    io_update,
                                    delay,
                                    ad9959::Mode::FourBitSerial,
                                    100_000_000).unwrap()
            };

            let io_expander = {
                let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
                let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
                let i2c1 = dp.I2C1.i2c((scl, sda), 100.khz(), &clocks);
                mcp23017::MCP23017::default(i2c1).unwrap()
            };

            let spi = {
                let spi_mosi = gpiod.pd7.into_alternate_af5();
                let spi_miso = gpioa.pa6.into_alternate_af5();
                let spi_sck = gpiog.pg11.into_alternate_af5();

                let config = hal::spi::Config::new(hal::spi::Mode{
                        polarity: hal::spi::Polarity::IdleHigh,
                        phase: hal::spi::Phase::CaptureOnSecondTransition,
                    })
                    .frame_size(8);

                dp.SPI1.spi((spi_sck, spi_miso, spi_mosi), config, 25.mhz(), &clocks)
            };

            pounder::PounderDevices::new(io_expander, ad9959, spi).unwrap()
        };

        let mut fp_led_0 = gpiod.pd5.into_push_pull_output();
        let mut fp_led_1 = gpiod.pd6.into_push_pull_output();
        let mut fp_led_2 = gpiod.pd12.into_push_pull_output();
        let mut fp_led_3 = gpiog.pg4.into_push_pull_output();

        fp_led_0.set_low().unwrap();
        fp_led_1.set_low().unwrap();
        fp_led_2.set_low().unwrap();
        fp_led_3.set_low().unwrap();

        let mut eeprom_i2c = {
            let sda = gpiof.pf0.into_alternate_af4().set_open_drain();
            let scl = gpiof.pf1.into_alternate_af4().set_open_drain();
            dp.I2C2.i2c((scl, sda), 100.khz(), &clocks)
        };

        // Configure ethernet pins.
        {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpioe.pe3.into_push_pull_output();
            eth_phy_nrst.set_high().unwrap();
            eth_phy_nrst.set_low().unwrap();
            eth_phy_nrst.set_high().unwrap();
            let _rmii_ref_clk = gpioa.pa1.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_mdio = gpioa.pa2.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_mdc = gpioc.pc1.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_crs_dv = gpioa.pa7.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_rxd0 = gpioc.pc4.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_rxd1 = gpioc.pc5.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_tx_en = gpiob.pb11.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_txd0 = gpiob.pb12.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_txd1 = gpiog.pg14.into_alternate_af11().set_speed(hal::gpio::Speed::VeryHigh);
        }

        let mac_addr = match eeprom::read_eui48(&mut eeprom_i2c) {
            Err(_) => {
                info!("Could not read EEPROM, using default MAC address");
                net::wire::EthernetAddress([0x10, 0xE2, 0xD5, 0x00, 0x03, 0x00])
            }
            Ok(raw_mac) => net::wire::EthernetAddress(raw_mac),
        };

        let (network_interface, eth_mac) = {
            // Configure the ethernet controller
            let (eth_dma, eth_mac) = unsafe {
                ethernet::ethernet_init(
                        dp.ETHERNET_MAC,
                        dp.ETHERNET_MTL,
                        dp.ETHERNET_DMA,
                        &mut DES_RING,
                        mac_addr.clone())
            };

            let store = unsafe { &mut NET_STORE };

            store.ip_addrs[0] = net::wire::IpCidr::new(net::wire::IpAddress::v4(10, 0, 16, 99), 24);

            let neighbor_cache = net::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

            let interface = net::iface::EthernetInterfaceBuilder::new(eth_dma)
                    .ethernet_addr(mac_addr)
                    .neighbor_cache(neighbor_cache)
                    .ip_addrs(&mut store.ip_addrs[..])
                    .finalize();

            (interface, eth_mac)
        };

        cp.SCB.enable_icache();

        init_log();
        // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
        // info!("Built on {}", build_info::BUILT_TIME_UTC);
        // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

        // Utilize the cycle counter for RTFM scheduling.
        cp.DWT.enable_cycle_counter();

        let mut debug_pin = gpioc.pc6.into_push_pull_output();
        debug_pin.set_low().unwrap();

        let mut dac_pin = gpiob.pb15.into_push_pull_output();
        dac_pin.set_low().unwrap();

        // Configure timer 2 to trigger conversions for the ADC
        let mut timer2 = dp.TIM2.timer(500.khz(), &mut clocks);
        timer2.listen(hal::timer::Event::TimeOut);

        init::LateResources {
            adc1: adc1_spi,
            dac1: dac1_spi,
            adc2: adc2_spi,
            dac2: dac2_spi,
            _afe1: afe1,
            _afe2: afe2,

            dbg_pin: debug_pin,
            dac_pin: dac_pin,
            timer: timer2,
            pounder: pounder_devices,

            eeprom_i2c: eeprom_i2c,
            net_interface: network_interface,
            _eth_mac: eth_mac,
            mac_addr: mac_addr,
        }
    }

    #[task(binds = TIM2, resources = [dbg_pin, timer, adc1, adc2])]
    fn tim2(mut c: tim2::Context) {
        c.resources.timer.clear_uif_bit();
        c.resources.dbg_pin.set_high().unwrap();

        // Start a SPI transaction on ADC0 and ADC1
        c.resources.adc1.lock(|adc| adc.spi.cr1.modify(|_, w| w.cstart().set_bit()));
        c.resources.adc2.lock(|adc| adc.spi.cr1.modify(|_, w| w.cstart().set_bit()));

        c.resources.dbg_pin.set_low().unwrap();
    }

    #[task(binds = SPI2, resources = [adc1, dac1, adc2, dac2, iir_state, iir_ch, dac_pin], priority = 2)]
    fn adc_spi(c: adc_spi::Context) {
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();

        c.resources.dac_pin.set_high().unwrap();

        let output_ch1 = {
            let a: u16 = c.resources.adc1.read().unwrap();
            let x0 = f32::from(a as i16);
            let y0 = c.resources.iir_ch[0].update(&mut c.resources.iir_state[0], x0);
            y0 as i16 as u16 ^ 0x8000
        };
        c.resources.adc1.spi.ifcr.write(|w| w.eotc().set_bit());

        let output_ch2 = {
            let a: u16 = nb::block!(c.resources.adc2.read()).unwrap();
            let x0 = f32::from(a as i16);
            let y0 = c.resources.iir_ch[1].update(&mut c.resources.iir_state[1], x0);
            y0 as i16 as u16 ^ 0x8000
        };
        c.resources.adc2.spi.ifcr.write(|w| w.eotc().set_bit());

        c.resources.dac1.send(output_ch1).unwrap();
        c.resources.dac2.send(output_ch2).unwrap();

        c.resources.dac_pin.set_low().unwrap();
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();
    }

    #[idle(resources=[net_interface, mac_addr, iir_state, iir_ch])]
    fn idle(mut c: idle::Context) -> ! {

        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets = net::socket::SocketSet::new(&mut socket_set_entries[..]);

        let mut rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_handle0 = {
            let tcp_rx_buffer = net::socket::TcpSocketBuffer::new(&mut rx_storage[..]);
            let tcp_tx_buffer = net::socket::TcpSocketBuffer::new(&mut tx_storage[..]);
            let tcp_socket = net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
            sockets.add(tcp_socket)
        };

        let mut rx_storage2 = [0; TCP_RX_BUFFER_SIZE];
        let mut tx_storage2 = [0; TCP_TX_BUFFER_SIZE];
        let tcp_handle1 = {
            let tcp_rx_buffer = net::socket::TcpSocketBuffer::new(&mut rx_storage2[..]);
            let tcp_tx_buffer = net::socket::TcpSocketBuffer::new(&mut tx_storage2[..]);
            let tcp_socket = net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
            sockets.add(tcp_socket)
        };

        let mut server = server::Server::new();

        let mut time = 0u32;
        let mut next_ms = Instant::now();

        // TODO: Replace with reference to CPU clock from CCDR.
        next_ms += 400_000.cycles();

        loop {
            let tick = Instant::now() > next_ms;

            if tick {
                next_ms += 400_000.cycles();
                time += 1;
            }

            {
                let mut socket = sockets.get::<net::socket::TcpSocket>(tcp_handle0);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket
                        .listen(1234)
                        .unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else if tick && socket.can_send() {
                    let s = c.resources.iir_state.lock(|iir_state| server::Status {
                        t: time,
                        x0: iir_state[0][0],
                        y0: iir_state[0][2],
                        x1: iir_state[1][0],
                        y1: iir_state[1][2],
                    });
                    server::json_reply(&mut socket, &s);
                }
            }

            {
                let socket =
                    &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle1);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket
                        .listen(1235)
                        .unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else {
                    server.poll(socket, |req: &server::Request| {
                        if req.channel < 2 {
                            c.resources.iir_ch.lock(|iir_ch| {
                                iir_ch[req.channel as usize] = req.iir
                            });
                        }
                    });
                }
            }

            let sleep = match c.resources.net_interface.poll(&mut sockets,
                                             net::time::Instant::from_millis(time as i64)) {
                Ok(changed) => changed,
                Err(net::Error::Unrecognized) => true,
                Err(e) => {
                    info!("iface poll error: {:?}", e);
                    true
                }
            };

            if sleep {
                cortex_m::asm::wfi();
            }
        }
    }

    /*
    #[task(binds = ETH, resources = [net_interface], priority = 1)]
    fn eth(c: eth::Context) {
        let dma = &c.resources.net_interface.device();
        ETHERNET_PENDING.store(true, Ordering::Relaxed);
        dma.interrupt_handler()
    }
    */

    extern "C" {
        // hw interrupt handlers for RTFM to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};

#[exception]
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
