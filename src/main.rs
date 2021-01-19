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

#[allow(unused_imports)]
use core::convert::TryInto;

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

// The number of ticks in the ADC sampling timer. The timer runs at 100MHz, so the step size is
// equal to 10ns per tick.
// Currently, the sample rate is equal to: Fsample = 100/256 MHz = 390.625 KHz
const ADC_SAMPLE_TICKS_LOG2: u16 = 8;
const ADC_SAMPLE_TICKS: u16 = 1 << ADC_SAMPLE_TICKS_LOG2;

// The desired ADC sample processing buffer size.
const SAMPLE_BUFFER_SIZE_LOG2: usize = 3;
const SAMPLE_BUFFER_SIZE: usize = 1 << SAMPLE_BUFFER_SIZE_LOG2;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// Frequency scaling factor for lock-in harmonic demodulation.
const HARMONIC: u32 = 1;
// Phase offset applied to the lock-in demodulation signal.
const PHASE_OFFSET: u32 = 0;

#[link_section = ".sram3.eth"]
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

mod adc;
mod afe;
mod dac;
mod design_parameters;
mod digital_input_stamper;
mod eeprom;
mod hrtimer;
mod pounder;
mod server;
mod timers;

use adc::{Adc0Input, Adc1Input};
use dac::{Dac0Output, Dac1Output};
use dsp::{
    iir, iir_int,
    reciprocal_pll::TimestampHandler,
    trig::{atan2, cossin},
    Complex,
};
use pounder::DdsOutput;

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
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        input_stamper: digital_input_stamper::InputStamper,
        timestamp_handler: TimestampHandler,
        iir_lockin: iir_int::IIR,
        iir_state_lockin: [iir_int::IIRState; 2],

        eeprom_i2c: hal::i2c::I2c<hal::stm32::I2C2>,

        dds_output: Option<DdsOutput>,

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

        pounder: Option<pounder::PounderDevices>,

        pounder_stamper: Option<pounder::timestamp::Timestamper>,

        // Format: iir_state[ch][cascade-no][coeff]
        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::IIRState; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE }; IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
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
        let mut gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

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
        let mut sampling_timer = {
            // The timer frequency is manually adjusted below, so the 1KHz setting here is a
            // dont-care.
            let mut timer2 =
                dp.TIM2.timer(1.khz(), ccdr.peripheral.TIM2, &ccdr.clocks);

            // Configure the timer to count at the designed tick rate. We will manually set the
            // period below.
            timer2.pause();
            timer2.reset_counter();
            timer2.set_tick_freq(design_parameters::TIMER_FREQUENCY);

            let mut sampling_timer = timers::SamplingTimer::new(timer2);
            sampling_timer.set_period_ticks((ADC_SAMPLE_TICKS - 1) as u32);

            // The sampling timer is used as the master timer for the shadow-sampling timer. Thus,
            // it generates a trigger whenever it is enabled.

            sampling_timer
        };

        let mut shadow_sampling_timer = {
            // The timer frequency is manually adjusted below, so the 1KHz setting here is a
            // dont-care.
            let mut timer3 =
                dp.TIM3.timer(1.khz(), ccdr.peripheral.TIM3, &ccdr.clocks);

            // Configure the timer to count at the designed tick rate. We will manually set the
            // period below.
            timer3.pause();
            timer3.reset_counter();
            timer3.set_tick_freq(design_parameters::TIMER_FREQUENCY);

            let mut shadow_sampling_timer =
                timers::ShadowSamplingTimer::new(timer3);
            shadow_sampling_timer.set_period_ticks(ADC_SAMPLE_TICKS - 1);

            // The shadow sampling timer is a slave-mode timer to the sampling timer. It should
            // always be in-sync - thus, we configure it to operate in slave mode using "Trigger
            // mode".
            // For TIM3, TIM2 can be made the internal trigger connection using ITR1. Thus, the
            // SamplingTimer start now gates the start of the ShadowSamplingTimer.
            shadow_sampling_timer.set_slave_mode(
                timers::TriggerSource::Trigger1,
                timers::SlaveMode::Trigger,
            );

            shadow_sampling_timer
        };

        let sampling_timer_channels = sampling_timer.channels();
        let shadow_sampling_timer_channels = shadow_sampling_timer.channels();

        let mut timestamp_timer = {
            // The timer frequency is manually adjusted below, so the 1KHz setting here is a
            // dont-care.
            let mut timer5 =
                dp.TIM5.timer(1.khz(), ccdr.peripheral.TIM5, &ccdr.clocks);

            // Configure the timer to count at the designed tick rate. We will manually set the
            // period below.
            timer5.pause();
            timer5.set_tick_freq(design_parameters::TIMER_FREQUENCY);

            // The timestamp timer must run at exactly a multiple of the sample timer based on the
            // batch size. To accomodate this, we manually set the prescaler identical to the sample
            // timer, but use a period that is longer.
            let mut timer = timers::TimestampTimer::new(timer5);

            let period =
                digital_input_stamper::calculate_timestamp_timer_period();
            timer.set_period_ticks(period);

            timer
        };

        let timestamp_timer_channels = timestamp_timer.channels();

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
                .communication_mode(hal::spi::CommunicationMode::Receiver)
                .cs_delay(design_parameters::ADC_SETUP_TIME);

                let spi: hal::spi::Spi<_, _, u16> = dp.SPI2.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MAX,
                    ccdr.peripheral.SPI2,
                    &ccdr.clocks,
                );

                Adc0Input::new(
                    spi,
                    dma_streams.0,
                    dma_streams.1,
                    dma_streams.2,
                    sampling_timer_channels.ch1,
                    shadow_sampling_timer_channels.ch1,
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
                .communication_mode(hal::spi::CommunicationMode::Receiver)
                .cs_delay(design_parameters::ADC_SETUP_TIME);

                let spi: hal::spi::Spi<_, _, u16> = dp.SPI3.spi(
                    (spi_sck, spi_miso, hal::spi::NoMosi),
                    config,
                    design_parameters::ADC_DAC_SCK_MAX,
                    ccdr.peripheral.SPI3,
                    &ccdr.clocks,
                );

                Adc1Input::new(
                    spi,
                    dma_streams.3,
                    dma_streams.4,
                    dma_streams.5,
                    sampling_timer_channels.ch2,
                    shadow_sampling_timer_channels.ch2,
                )
            };

            (adc0, adc1)
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
                    design_parameters::ADC_DAC_SCK_MAX,
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
                    design_parameters::ADC_DAC_SCK_MAX,
                    ccdr.peripheral.SPI5,
                    &ccdr.clocks,
                )
            };

            let dac0 = Dac0Output::new(
                dac0_spi,
                dma_streams.6,
                sampling_timer_channels.ch3,
            );
            let dac1 = Dac1Output::new(
                dac1_spi,
                dma_streams.7,
                sampling_timer_channels.ch4,
            );
            (dac0, dac1)
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
        let (pounder_devices, dds_output) = if pounder_pgood.is_high().unwrap()
        {
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
                        design_parameters::POUNDER_QSPI_FREQUENCY,
                        &ccdr.clocks,
                        ccdr.peripheral.QSPI,
                    );

                    pounder::QspiInterface::new(qspi).unwrap()
                };

                #[cfg(feature = "pounder_v1_1")]
                let reset_pin = gpiog.pg6.into_push_pull_output();
                #[cfg(not(feature = "pounder_v1_1"))]
                let reset_pin = gpioa.pa0.into_push_pull_output();

                let mut io_update = gpiog.pg7.into_push_pull_output();

                let ref_clk: hal::time::Hertz =
                    design_parameters::DDS_REF_CLK.into();

                let ad9959 = ad9959::Ad9959::new(
                    qspi_interface,
                    reset_pin,
                    &mut io_update,
                    &mut delay,
                    ad9959::Mode::FourBitSerial,
                    ref_clk.0 as f32,
                    design_parameters::DDS_MULTIPLIER,
                )
                .unwrap();

                // Return IO_Update
                gpiog.pg7 = io_update.into_analog();

                ad9959
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

            let pounder_devices = pounder::PounderDevices::new(
                io_expander,
                spi,
                adc1,
                adc2,
                adc1_in_p,
                adc2_in_p,
            )
            .unwrap();

            let dds_output = {
                let io_update_trigger = {
                    let _io_update = gpiog
                        .pg7
                        .into_alternate_af2()
                        .set_speed(hal::gpio::Speed::VeryHigh);

                    // Configure the IO_Update signal for the DDS.
                    let mut hrtimer = hrtimer::HighResTimerE::new(
                        dp.HRTIM_TIME,
                        dp.HRTIM_MASTER,
                        dp.HRTIM_COMMON,
                        ccdr.clocks,
                        ccdr.peripheral.HRTIM,
                    );

                    // IO_Update occurs after a fixed delay from the QSPI write. Note that the timer
                    // is triggered after the QSPI write, which can take approximately 120nS, so
                    // there is additional margin.
                    hrtimer.configure_single_shot(
                        hrtimer::Channel::Two,
                        design_parameters::POUNDER_IO_UPDATE_DURATION,
                        design_parameters::POUNDER_IO_UPDATE_DELAY,
                    );

                    // Ensure that we have enough time for an IO-update every sample.
                    let sample_frequency = {
                        let timer_frequency: hal::time::Hertz =
                            design_parameters::TIMER_FREQUENCY.into();
                        timer_frequency.0 as f32 / ADC_SAMPLE_TICKS as f32
                    };

                    let sample_period = 1.0 / sample_frequency;
                    assert!(
                        sample_period
                            > design_parameters::POUNDER_IO_UPDATE_DELAY
                    );

                    hrtimer
                };

                let (qspi, config) = ad9959.freeze();
                DdsOutput::new(qspi, io_update_trigger, config)
            };

            (Some(pounder_devices), Some(dds_output))
        } else {
            (None, None)
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

        let mut input_stamper = {
            let trigger = gpioa.pa3.into_alternate_af2();
            digital_input_stamper::InputStamper::new(
                trigger,
                timestamp_timer_channels.ch4,
            )
        };

        #[cfg(feature = "pounder_v1_1")]
        let pounder_stamper = {
            let dma2_streams =
                hal::dma::dma::StreamsTuple::new(dp.DMA2, ccdr.peripheral.DMA2);

            let etr_pin = gpioa.pa0.into_alternate_af3();

            // The frequency in the constructor is dont-care, as we will modify the period + clock
            // source manually below.
            let tim8 =
                dp.TIM8.timer(1.khz(), ccdr.peripheral.TIM8, &ccdr.clocks);
            let mut timestamp_timer = timers::PounderTimestampTimer::new(tim8);

            // Pounder is configured to generate a 500MHz reference clock, so a 125MHz sync-clock is
            // output. As a result, dividing the 125MHz sync-clk provides a 31.25MHz tick rate for
            // the timestamp timer. 31.25MHz corresponds with a 32ns tick rate.
            timestamp_timer.set_external_clock(timers::Prescaler::Div4);
            timestamp_timer.start();

            // We want the pounder timestamp timer to overflow once per batch.
            let tick_ratio = {
                let sync_clk_mhz: f32 = design_parameters::DDS_SYSTEM_CLK.0
                    as f32
                    / design_parameters::DDS_SYNC_CLK_DIV as f32;
                sync_clk_mhz / design_parameters::TIMER_FREQUENCY.0 as f32
            };

            let period = (tick_ratio
                * ADC_SAMPLE_TICKS as f32
                * SAMPLE_BUFFER_SIZE as f32) as u32
                / 4;
            timestamp_timer.set_period_ticks((period - 1).try_into().unwrap());
            let tim8_channels = timestamp_timer.channels();

            let stamper = pounder::timestamp::Timestamper::new(
                timestamp_timer,
                dma2_streams.0,
                tim8_channels.ch1,
                &mut sampling_timer,
                etr_pin,
            );

            Some(stamper)
        };

        #[cfg(not(feature = "pounder_v1_1"))]
        let pounder_stamper = None;

        let timestamp_handler = TimestampHandler::new(
            4,
            3,
            ADC_SAMPLE_TICKS_LOG2 as usize,
            SAMPLE_BUFFER_SIZE_LOG2,
        );
        let iir_lockin = iir_int::IIR::default();
        let iir_state_lockin = [iir_int::IIRState::default(); 2];

        // Start sampling ADCs.
        sampling_timer.start();
        timestamp_timer.start();
        input_stamper.start();

        init::LateResources {
            afes: (afe0, afe1),

            adcs,
            dacs,
            input_stamper,
            dds_output,
            pounder: pounder_devices,
            pounder_stamper,

            timestamp_handler,
            iir_lockin,
            iir_state_lockin,

            eeprom_i2c,
            net_interface: network_interface,
            eth_mac,
            mac_addr,
        }
    }

    /// Main DSP processing routine for Stabilizer.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    #[task(binds=DMA1_STR4, resources=[pounder_stamper, adcs, dacs, iir_state, iir_ch, dds_output, input_stamper, timestamp_handler, iir_lockin, iir_state_lockin], priority=2)]
    fn process(c: process::Context) {
        if let Some(stamper) = c.resources.pounder_stamper {
            let pounder_timestamps = stamper.acquire_buffer();
            info!("{:?}", pounder_timestamps);
        }

        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        let [dac0, dac1] = dac_samples;
        let iir_lockin = c.resources.iir_lockin;
        let iir_state_lockin = c.resources.iir_state_lockin;
        let iir_ch = c.resources.iir_ch;
        let iir_state = c.resources.iir_state;

        let (pll_phase, pll_frequency) = c
            .resources
            .timestamp_handler
            .update(c.resources.input_stamper.latest_timestamp());
        let frequency = pll_frequency.wrapping_mul(HARMONIC);
        let mut phase =
            PHASE_OFFSET.wrapping_add(pll_phase.wrapping_mul(HARMONIC));

        dac0.iter_mut().zip(dac1.iter_mut()).enumerate().for_each(
            |(i, (d0, d1))| {
                let m = cossin((phase as i32).wrapping_neg());
                phase = phase.wrapping_add(frequency);

                let signal = Complex(
                    iir_lockin.update(
                        &mut iir_state_lockin[0],
                        ((adc_samples[0][i] as i64 * m.0 as i64) >> 16) as i32,
                    ),
                    iir_lockin.update(
                        &mut iir_state_lockin[1],
                        ((adc_samples[0][i] as i64 * m.1 as i64) >> 16) as i32,
                    ),
                );

                let mut magnitude =
                    (signal.0 * signal.0 + signal.1 * signal.1) as f32;
                let mut phase = atan2(signal.1, signal.0) as f32;

                for j in 0..iir_state[0].len() {
                    magnitude =
                        iir_ch[0][j].update(&mut iir_state[0][j], magnitude);
                    phase = iir_ch[1][j].update(&mut iir_state[1][j], phase);
                }

                unsafe {
                    let magnitude = magnitude.to_int_unchecked::<i16>();
                    let phase = phase.to_int_unchecked::<i16>();

                    *d0 = magnitude as u16 ^ 0x8000;
                    *d1 = phase as u16 ^ 0x8000;
                }
            },
        );

        if let Some(dds_output) = c.resources.dds_output {
            let builder = dds_output.builder().update_channels(
                &[pounder::Channel::Out0.into()],
                Some(u32::MAX / 4),
                None,
                None,
            );

            builder.write_profile();
        }
    }

    #[idle(resources=[net_interface, pounder, mac_addr, eth_mac, iir_state, iir_ch, afes])]
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
                                            x0: iir_state[0][0][0],
                                            y0: iir_state[0][0][2],
                                            x1: iir_state[1][0][0],
                                            y1: iir_state[1][0][2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                // "_b" means cascades 2nd IIR
                                "stabilizer/iir_b/state": (|| {
                                    let state = c.resources.iir_state.lock(|iir_state|
                                        server::Status {
                                            t: time,
                                            x0: iir_state[0][IIR_CASCADE_LENGTH-1][0],
                                            y0: iir_state[0][IIR_CASCADE_LENGTH-1][2],
                                            x1: iir_state[1][IIR_CASCADE_LENGTH-1][0],
                                            y1: iir_state[1][IIR_CASCADE_LENGTH-1][2],
                                    });

                                    Ok::<server::Status, ()>(state)
                                }),
                                "stabilizer/afe0/gain": (|| c.resources.afes.0.get_gain()),
                                "stabilizer/afe1/gain": (|| c.resources.afes.1.get_gain())
                            ],

                            modifiable_attributes: [
                                "stabilizer/iir0/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][0] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir1/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][0] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir_b0/state": server::IirRequest, (|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][IIR_CASCADE_LENGTH-1] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/iir_b1/state": server::IirRequest,(|req: server::IirRequest| {
                                    c.resources.iir_ch.lock(|iir_ch| {
                                        if req.channel > 1 {
                                            return Err(());
                                        }

                                        iir_ch[req.channel as usize][IIR_CASCADE_LENGTH-1] = req.iir;

                                        Ok::<server::IirRequest, ()>(req)
                                    })
                                }),
                                "stabilizer/afe0/gain": afe::Gain, (|gain| {
                                    c.resources.afes.0.set_gain(gain);
                                    Ok::<(), ()>(())
                                }),
                                "stabilizer/afe1/gain": afe::Gain, (|gain| {
                                    c.resources.afes.1.set_gain(gain);
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
