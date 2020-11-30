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
    let gpiod = unsafe { &*hal::stm32::GPIOD::ptr() };
    gpiod.odr.modify(|_, w| w.odr6().high().odr12().high()); // FP_LED_1, FP_LED_3
    #[cfg(feature = "nightly")]
    core::intrinsics::abort();
    #[cfg(not(feature = "nightly"))]
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

// use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use cortex_m_rt::exception;
use rtic::cyccnt::{Instant, U32Ext};
use stm32h7xx_hal as hal;
use stm32h7xx_hal::prelude::*;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use hal::{
    dma::{
        config::Priority,
        dma::{DMAReq, DmaConfig},
        traits::TargetAddress,
        MemoryToPeripheral, PeripheralToMemory, Transfer,
    },
    ethernet::{self, PHY},
};

use smoltcp as net;
use smoltcp::iface::Routes;
use smoltcp::wire::Ipv4Address;

use heapless::{consts::*, String};

// The desired sampling frequency of the ADCs.
const SAMPLE_FREQUENCY_KHZ: u32 = 500;

// The desired ADC sample processing buffer size.
const SAMPLE_BUFFER_SIZE: usize = 1;

#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

mod adc;
mod afe;
mod dac;
mod design_parameters;
mod eeprom;
mod pounder;
mod sampling_timer;
mod server;

use adc::{Adc0Input, Adc1Input, AdcInputs};
use dac::{Dac0Output, Dac1Output, DacOutputs};
use dsp::iir;

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
    routes_storage: [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 1],
}

static mut NET_STORE: NetStorage = NetStorage {
    // Placeholder for the real IP address, which is initialized at runtime.
    ip_addrs: [net::wire::IpCidr::Ipv6(
        net::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
    )],

    neighbor_cache: [None; 8],

    routes_storage: [None; 1],
};

const SCALE: f32 = ((1 << 15) - 1) as f32;

// static ETHERNET_PENDING: AtomicBool = AtomicBool::new(true);

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

type AFE0 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiof::PF2<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiof::PF5<hal::gpio::Output<hal::gpio::PushPull>>,
>;

type AFE1 = afe::ProgrammableGainAmplifier<
    hal::gpio::gpiod::PD14<hal::gpio::Output<hal::gpio::PushPull>>,
    hal::gpio::gpiod::PD15<hal::gpio::Output<hal::gpio::PushPull>>,
>;

macro_rules! route_request {
    ($request:ident,
            readable_attributes: [$($read_attribute:tt: $getter:tt),*],
            modifiable_attributes: [$($write_attribute:tt: $TYPE:ty, $setter:tt),*]) => {
        match $request.req {
            server::AccessRequest::Read => {
                match $request.attribute {
                $(
                    $read_attribute => {
                        #[allow(clippy::redundant_closure_call)]
                        let value = match $getter() {
                            Ok(data) => data,
                            Err(_) => return server::Response::error($request.attribute,
                                                                     "Failed to read attribute"),
                        };

                        let encoded_data: String<U256> = match serde_json_core::to_string(&value) {
                            Ok(data) => data,
                            Err(_) => return server::Response::error($request.attribute,
                                    "Failed to encode attribute value"),
                        };

                        server::Response::success($request.attribute, &encoded_data)
                    },
                 )*
                    _ => server::Response::error($request.attribute, "Unknown attribute")
                }
            },
            server::AccessRequest::Write => {
                match $request.attribute {
                $(
                    $write_attribute => {
                        let new_value = match serde_json_core::from_str::<$TYPE>(&$request.value) {
                            Ok(data) => data,
                            Err(_) => return server::Response::error($request.attribute,
                                    "Failed to decode value"),
                        };

                        #[allow(clippy::redundant_closure_call)]
                        match $setter(new_value) {
                            Ok(_) => server::Response::success($request.attribute, &$request.value),
                            Err(_) => server::Response::error($request.attribute,
                                    "Failed to set attribute"),
                        }
                    }
                 )*
                    _ => server::Response::error($request.attribute, "Unknown attribute")
                }
            }
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afe0: AFE0,
        afe1: AFE1,

        adcs: AdcInputs,
        dacs: DacOutputs,

        eeprom_i2c: hal::i2c::I2c<hal::stm32::I2C2>,

        // Note: It appears that rustfmt generates a format that GDB cannot recognize, which
        // results in GDB breakpoints being set improperly.
        #[rustfmt::skip]
        net_interface: net::iface::EthernetInterface<
            'static,
            'static,
            'static,
            ethernet::EthernetDMA<'static>>,
        eth_mac: ethernet::phy::LAN8742A<ethernet::EthernetMAC>,
        mac_addr: net::wire::EthernetAddress,

        pounder: Option<pounder::PounderDevices<asm_delay::AsmDelay>>,

        #[init([[0.; 5]; 2])]
        iir_state: [iir::IIRState; 2],
        #[init([iir::IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE }; 2])]
        iir_ch: [iir::IIR; 2],
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        let dp = c.device;
        let mut cp = c.core;

        let pwr = dp.PWR.constrain();
        let vos = pwr.freeze();

        // Enable SRAM3 for the ethernet descriptor ring.
        dp.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

        // Clear reset flags.
        dp.RCC.rsr.write(|w| w.rmvf().set_bit());

        // Select the PLLs for SPI.
        dp.RCC
            .d2ccip1r
            .modify(|_, w| w.spi123sel().pll2_p().spi45sel().pll2_q());

        let rcc = dp.RCC.constrain();
        let ccdr = rcc
            .use_hse(8.mhz())
            .sysclk(400.mhz())
            .hclk(200.mhz())
            .per_ck(100.mhz())
            .pll2_p_ck(100.mhz())
            .pll2_q_ck(100.mhz())
            .freeze(vos, &dp.SYSCFG);

        init_log();

        let mut delay = hal::delay::Delay::new(cp.SYST, ccdr.clocks);

        let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
        let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
        let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpioe = dp.GPIOE.split(ccdr.peripheral.GPIOE);
        let gpiof = dp.GPIOF.split(ccdr.peripheral.GPIOF);
        let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

        let afe0 = {
            let a0_pin = gpiof.pf2.into_push_pull_output();
            let a1_pin = gpiof.pf5.into_push_pull_output();
            afe::ProgrammableGainAmplifier::new(a0_pin, a1_pin)
        };

        let afe1 = {
            let a0_pin = gpiod.pd14.into_push_pull_output();
            let a1_pin = gpiod.pd15.into_push_pull_output();
            afe::ProgrammableGainAmplifier::new(a0_pin, a1_pin)
        };

        let dma_streams =
            hal::dma::dma::StreamsTuple::new(dp.DMA1, ccdr.peripheral.DMA1);

        // Configure timer 2 to trigger conversions for the ADC
        let timer2 = dp.TIM2.timer(
            SAMPLE_FREQUENCY_KHZ.khz(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );

        let mut sampling_timer = sampling_timer::SamplingTimer::new(timer2);
        let sampling_timer_channels = sampling_timer.channels();

        // Configure the SPI interfaces to the ADCs and DACs.
        let adcs = {
            let adc0 = {
                let spi_miso = gpiob
                    .pb14
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_sck = gpiob
                    .pb10
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let _spi_nss = gpiob
                    .pb9
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                let config = hal::spi::Config::new(hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .manage_cs()
                .suspend_when_inactive()
                .cs_delay(design_parameters::ADC_SETUP_TIME);

                let spi: hal::spi::Spi<_, _, u16> = dp.SPI2.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MHZ_MAX.mhz(),
                    ccdr.peripheral.SPI2,
                    &ccdr.clocks,
                );

                Adc0Input::new(
                    spi,
                    dma_streams.0,
                    dma_streams.1,
                    sampling_timer_channels.ch1,
                )
            };

            let adc1 = {
                let spi_miso = gpiob
                    .pb4
                    .into_alternate_af6()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_sck = gpioc
                    .pc10
                    .into_alternate_af6()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let _spi_nss = gpioa
                    .pa15
                    .into_alternate_af6()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                let config = hal::spi::Config::new(hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .manage_cs()
                .suspend_when_inactive()
                .cs_delay(design_parameters::ADC_SETUP_TIME);

                let spi: hal::spi::Spi<_, _, u16> = dp.SPI3.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MHZ_MAX.mhz(),
                    ccdr.peripheral.SPI3,
                    &ccdr.clocks,
                );

                Adc1Input::new(
                    spi,
                    dma_streams.2,
                    dma_streams.3,
                    sampling_timer_channels.ch2,
                )
            };

            AdcInputs::new(adc0, adc1)
        };

        let dacs = {
            let _dac_clr_n =
                gpioe.pe12.into_push_pull_output().set_high().unwrap();
            let _dac0_ldac_n =
                gpioe.pe11.into_push_pull_output().set_low().unwrap();
            let _dac1_ldac_n =
                gpioe.pe15.into_push_pull_output().set_low().unwrap();

            let dac0_spi = {
                let spi_miso = gpioe
                    .pe5
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_sck = gpioe
                    .pe2
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let _spi_nss = gpioe
                    .pe4
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                let config = hal::spi::Config::new(hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .manage_cs()
                .suspend_when_inactive()
                .communication_mode(hal::spi::CommunicationMode::Transmitter)
                .swap_mosi_miso();

                dp.SPI4.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MHZ_MAX.mhz(),
                    ccdr.peripheral.SPI4,
                    &ccdr.clocks,
                )
            };

            let dac1_spi = {
                let spi_miso = gpiof
                    .pf8
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_sck = gpiof
                    .pf7
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let _spi_nss = gpiof
                    .pf6
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                let config = hal::spi::Config::new(hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                })
                .manage_cs()
                .communication_mode(hal::spi::CommunicationMode::Transmitter)
                .suspend_when_inactive()
                .swap_mosi_miso();

                dp.SPI5.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MHZ_MAX.mhz(),
                    ccdr.peripheral.SPI5,
                    &ccdr.clocks,
                )
            };

            let dac0 = Dac0Output::new(
                dac0_spi,
                dma_streams.4,
                sampling_timer_channels.ch3,
            );
            let dac1 = Dac1Output::new(
                dac1_spi,
                dma_streams.5,
                sampling_timer_channels.ch4,
            );
            DacOutputs::new(dac0, dac1)
        };

        let mut fp_led_0 = gpiod.pd5.into_push_pull_output();
        let mut fp_led_1 = gpiod.pd6.into_push_pull_output();
        let mut fp_led_2 = gpiog.pg4.into_push_pull_output();
        let mut fp_led_3 = gpiod.pd12.into_push_pull_output();

        fp_led_0.set_low().unwrap();
        fp_led_1.set_low().unwrap();
        fp_led_2.set_low().unwrap();
        fp_led_3.set_low().unwrap();

        // Measure the Pounder PGOOD output to detect if pounder is present on Stabilizer.
        let pounder_pgood = gpiob.pb13.into_pull_down_input();
        delay.delay_ms(2u8);
        let pounder_devices = if pounder_pgood.is_high().unwrap() {
            let ad9959 = {
                let qspi_interface = {
                    // Instantiate the QUADSPI pins and peripheral interface.
                    let qspi_pins = {
                        let _qspi_ncs = gpioc
                            .pc11
                            .into_alternate_af9()
                            .set_speed(hal::gpio::Speed::VeryHigh);

                        let clk = gpiob
                            .pb2
                            .into_alternate_af9()
                            .set_speed(hal::gpio::Speed::VeryHigh);
                        let io0 = gpioe
                            .pe7
                            .into_alternate_af10()
                            .set_speed(hal::gpio::Speed::VeryHigh);
                        let io1 = gpioe
                            .pe8
                            .into_alternate_af10()
                            .set_speed(hal::gpio::Speed::VeryHigh);
                        let io2 = gpioe
                            .pe9
                            .into_alternate_af10()
                            .set_speed(hal::gpio::Speed::VeryHigh);
                        let io3 = gpioe
                            .pe10
                            .into_alternate_af10()
                            .set_speed(hal::gpio::Speed::VeryHigh);

                        (clk, io0, io1, io2, io3)
                    };

                    let qspi = hal::qspi::Qspi::bank2(
                        dp.QUADSPI,
                        qspi_pins,
                        10.mhz(),
                        &ccdr.clocks,
                        ccdr.peripheral.QSPI,
                    );
                    pounder::QspiInterface::new(qspi).unwrap()
                };

                let mut reset_pin = gpioa.pa0.into_push_pull_output();
                let io_update = gpiog.pg7.into_push_pull_output();

                let asm_delay = {
                    let frequency_hz = ccdr.clocks.c_ck().0;
                    asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
                        frequency_hz,
                    ))
                };

                ad9959::Ad9959::new(
                    qspi_interface,
                    &mut reset_pin,
                    io_update,
                    asm_delay,
                    ad9959::Mode::FourBitSerial,
                    100_000_000f32,
                    5,
                )
                .unwrap()
            };

            let io_expander = {
                let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
                let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
                let i2c1 = dp.I2C1.i2c(
                    (scl, sda),
                    100.khz(),
                    ccdr.peripheral.I2C1,
                    &ccdr.clocks,
                );
                mcp23017::MCP23017::default(i2c1).unwrap()
            };

            let spi = {
                let spi_mosi = gpiod
                    .pd7
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_miso = gpioa
                    .pa6
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);
                let spi_sck = gpiog
                    .pg11
                    .into_alternate_af5()
                    .set_speed(hal::gpio::Speed::VeryHigh);

                let config = hal::spi::Config::new(hal::spi::Mode {
                    polarity: hal::spi::Polarity::IdleHigh,
                    phase: hal::spi::Phase::CaptureOnSecondTransition,
                });

                // The maximum frequency of this SPI must be limited due to capacitance on the MISO
                // line causing a long RC decay.
                dp.SPI1.spi(
                    (spi_sck, spi_miso, spi_mosi),
                    config,
                    5.mhz(),
                    ccdr.peripheral.SPI1,
                    &ccdr.clocks,
                )
            };

            let (adc1, adc2) = {
                let (mut adc1, mut adc2) = hal::adc::adc12(
                    dp.ADC1,
                    dp.ADC2,
                    &mut delay,
                    ccdr.peripheral.ADC12,
                    &ccdr.clocks,
                );

                let adc1 = {
                    adc1.calibrate();
                    adc1.enable()
                };

                let adc2 = {
                    adc2.calibrate();
                    adc2.enable()
                };

                (adc1, adc2)
            };

            let adc1_in_p = gpiof.pf11.into_analog();
            let adc2_in_p = gpiof.pf14.into_analog();

            Some(
                pounder::PounderDevices::new(
                    io_expander,
                    ad9959,
                    spi,
                    adc1,
                    adc2,
                    adc1_in_p,
                    adc2_in_p,
                )
                .unwrap(),
            )
        } else {
            None
        };

        let mut eeprom_i2c = {
            let sda = gpiof.pf0.into_alternate_af4().set_open_drain();
            let scl = gpiof.pf1.into_alternate_af4().set_open_drain();
            dp.I2C2.i2c(
                (scl, sda),
                100.khz(),
                ccdr.peripheral.I2C2,
                &ccdr.clocks,
            )
        };

        // Configure ethernet pins.
        {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpioe.pe3.into_push_pull_output();
            eth_phy_nrst.set_low().unwrap();
            delay.delay_us(200u8);
            eth_phy_nrst.set_high().unwrap();
            let _rmii_ref_clk = gpioa
                .pa1
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_mdio = gpioa
                .pa2
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_mdc = gpioc
                .pc1
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_crs_dv = gpioa
                .pa7
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_rxd0 = gpioc
                .pc4
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_rxd1 = gpioc
                .pc5
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_tx_en = gpiob
                .pb11
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_txd0 = gpiob
                .pb12
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
            let _rmii_txd1 = gpiog
                .pg14
                .into_alternate_af11()
                .set_speed(hal::gpio::Speed::VeryHigh);
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
                ethernet::new_unchecked(
                    dp.ETHERNET_MAC,
                    dp.ETHERNET_MTL,
                    dp.ETHERNET_DMA,
                    &mut DES_RING,
                    mac_addr,
                    ccdr.peripheral.ETH1MAC,
                    &ccdr.clocks,
                )
            };

            // Reset and initialize the ethernet phy.
            let mut lan8742a =
                ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
            lan8742a.phy_reset();
            lan8742a.phy_init();

            unsafe { ethernet::enable_interrupt() };

            let store = unsafe { &mut NET_STORE };

            store.ip_addrs[0] = net::wire::IpCidr::new(
                net::wire::IpAddress::v4(10, 0, 16, 99),
                24,
            );

            let default_v4_gw = Ipv4Address::new(10, 0, 16, 1);
            let mut routes = Routes::new(&mut store.routes_storage[..]);
            routes.add_default_ipv4_route(default_v4_gw).unwrap();

            let neighbor_cache =
                net::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

            let interface = net::iface::EthernetInterfaceBuilder::new(eth_dma)
                .ethernet_addr(mac_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut store.ip_addrs[..])
                .routes(routes)
                .finalize();

            (interface, lan8742a)
        };

        cp.SCB.enable_icache();

        // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
        // info!("Built on {}", build_info::BUILT_TIME_UTC);
        // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

        // Utilize the cycle counter for RTIC scheduling.
        cp.DWT.enable_cycle_counter();

        // Start sampling ADCs.
        sampling_timer.start();

        init::LateResources {
            afe0,
            afe1,

            adcs,
            dacs,

            pounder: pounder_devices,

            eeprom_i2c,
            net_interface: network_interface,
            eth_mac,
            mac_addr,
        }
    }

    #[task(binds=DMA1_STR3, resources=[adcs, dacs, iir_state, iir_ch], priority=2)]
    fn adc_update(c: adc_update::Context) {
        let (adc0_samples, adc1_samples) =
            c.resources.adcs.transfer_complete_handler();

        let (dac0, dac1) = c.resources.dacs.prepare_data();

        for (i, (adc0, adc1)) in
            adc0_samples.iter().zip(adc1_samples.iter()).enumerate()
        {
            dac0[i] = {
                let x0 = f32::from(*adc0 as i16);
                let y0 = c.resources.iir_ch[0]
                    .update(&mut c.resources.iir_state[0], x0);
                // Note(unsafe): The filter limits ensure that the value is in range.
                // The truncation introduces 1/2 LSB distortion.
                let y0 = unsafe { y0.to_int_unchecked::<i16>() };
                // Convert to DAC code
                y0 as u16 ^ 0x8000
            };

            dac1[i] = {
                let x1 = f32::from(*adc1 as i16);
                let y1 = c.resources.iir_ch[1]
                    .update(&mut c.resources.iir_state[1], x1);
                // Note(unsafe): The filter limits ensure that the value is in range.
                // The truncation introduces 1/2 LSB distortion.
                let y1 = unsafe { y1.to_int_unchecked::<i16>() };
                // Convert to DAC code
                y1 as u16 ^ 0x8000
            };
        }

        c.resources.dacs.commit_data();
    }

    #[idle(resources=[net_interface, pounder, mac_addr, eth_mac, iir_state, iir_ch, afe0, afe1])]
    fn idle(mut c: idle::Context) -> ! {
        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets =
            net::socket::SocketSet::new(&mut socket_set_entries[..]);

        let mut rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_handle = {
            let tcp_rx_buffer =
                net::socket::TcpSocketBuffer::new(&mut rx_storage[..]);
            let tcp_tx_buffer =
                net::socket::TcpSocketBuffer::new(&mut tx_storage[..]);
            let tcp_socket =
                net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
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
                let socket =
                    &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket
                        .listen(1235)
                        .unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else {
                    server.poll(socket, |req| {
                        info!("Got request: {:?}", req);
                        route_request!(req,
                            readable_attributes: [
                                "stabilizer/iir/state": (|| {
                                    let state = c.resources.iir_state.lock(|iir_state|
                                        server::Status {
                                            t: time,
                                            x0: iir_state[0][0],
                                            y0: iir_state[0][2],
                                            x1: iir_state[1][0],
                                            y1: iir_state[1][2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                "stabilizer/afe0/gain": (|| c.resources.afe0.get_gain()),
                                "stabilizer/afe1/gain": (|| c.resources.afe1.get_gain()),
                                "pounder/in0": (|| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.get_input_channel_state(pounder::Channel::In0),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/in1": (|| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.get_input_channel_state(pounder::Channel::In1),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/out0": (|| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.get_output_channel_state(pounder::Channel::Out0),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/out1": (|| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.get_output_channel_state(pounder::Channel::Out1),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/dds/clock": (|| {
                                    match c.resources.pounder {
                                        Some(pounder) => pounder.get_dds_clock_config(),
                                        _ => Err(pounder::Error::Access),
                                    }
                                })
                            ],

                            modifiable_attributes: [
                                "stabilizer/iir0/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir1/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "pounder/in0": pounder::ChannelState, (|state| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.set_channel_state(pounder::Channel::In0, state),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/in1": pounder::ChannelState, (|state| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.set_channel_state(pounder::Channel::In1, state),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/out0": pounder::ChannelState, (|state| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.set_channel_state(pounder::Channel::Out0, state),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/out1": pounder::ChannelState, (|state| {
                                    match c.resources.pounder {
                                        Some(pounder) =>
                                            pounder.set_channel_state(pounder::Channel::Out1, state),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "pounder/dds/clock": pounder::DdsClockConfig, (|config| {
                                    match c.resources.pounder {
                                        Some(pounder) => pounder.configure_dds_clock(config),
                                        _ => Err(pounder::Error::Access),
                                    }
                                }),
                                "stabilizer/afe0/gain": afe::Gain, (|gain| {
                                    c.resources.afe0.set_gain(gain);
                                    Ok::<(), ()>(())
                                }),
                                "stabilizer/afe1/gain": afe::Gain, (|gain| {
                                    c.resources.afe1.set_gain(gain);
                                    Ok::<(), ()>(())
                                })
                            ]
                        )
                    });
                }
            }

            let sleep = match c.resources.net_interface.poll(
                &mut sockets,
                net::time::Instant::from_millis(time as i64),
            ) {
                Ok(changed) => !changed,
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

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 output error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 output error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
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
