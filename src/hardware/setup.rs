//! Stabilizer hardware configuration
//!
//! This file contains all of the hardware-specific configuration of Stabilizer.
use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{self, AtomicBool, Ordering};
use core::{fmt::Write, ptr, slice};
use embedded_hal_compat::{markers::ForwardOutputPin, Forward, ForwardCompat};
use heapless::String;
use stm32h7xx_hal::{
    self as hal,
    ethernet::{self, PHY},
    gpio::Speed,
    prelude::*,
};

use smoltcp_nal::smoltcp;

use crate::hardware::delay::AsmDelay;
use crate::settings::{AppSettings, NetSettings};

use super::{
    adc, afe, cpu_temp_sensor::CpuTempSensor, dac, delay, design_parameters,
    eeprom, input_stamper::InputStamper, metadata::ApplicationMetadata,
    platform, pounder, pounder::dds_output::DdsOutput, shared_adc::SharedAdc,
    timers, DigitalInput0, DigitalInput1, Eem, EthernetPhy, Gpio,
    HardwareVersion, NetworkStack, Pgia, SerialTerminal, SystemTimer, Systick,
    UsbDevice,
};

const NUM_TCP_SOCKETS: usize = 4;
const NUM_UDP_SOCKETS: usize = 1;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP and DNS sockets
    // respectively.
    pub sockets: [smoltcp::iface::SocketStorage<'static>; NUM_SOCKETS + 2],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub dns_storage: [Option<smoltcp::socket::dns::DnsQuery>; 1],
}

#[derive(Copy, Clone)]
pub struct UdpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 2048],
    tx_metadata: [smoltcp::storage::PacketMetadata<
        smoltcp::socket::udp::UdpMetadata,
    >; 10],
    rx_metadata: [smoltcp::storage::PacketMetadata<
        smoltcp::socket::udp::UdpMetadata,
    >; 10],
}

impl UdpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 2048],
            tx_metadata: [smoltcp::storage::PacketMetadata::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::EMPTY; 10],
        }
    }
}

#[derive(Copy, Clone)]
pub struct TcpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 1024],
}

impl TcpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 1024],
        }
    }
}

impl Default for NetStorage {
    fn default() -> Self {
        NetStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            sockets: [smoltcp::iface::SocketStorage::EMPTY; NUM_SOCKETS + 2],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::new(); NUM_UDP_SOCKETS],
            dns_storage: [None; 1],
        }
    }
}

/// The available networking devices on Stabilizer.
pub struct NetworkDevices {
    pub stack: NetworkStack,
    pub phy: EthernetPhy,
    pub mac_address: smoltcp::wire::EthernetAddress,
}

/// The available hardware interfaces on Stabilizer.
pub struct StabilizerDevices<
    C: serial_settings::Settings + 'static,
    const Y: usize,
> {
    pub temperature_sensor: CpuTempSensor,
    pub afes: [Pgia; 2],
    pub adcs: (adc::Adc0Input, adc::Adc1Input),
    pub dacs: (dac::Dac0Output, dac::Dac1Output),
    pub timestamper: InputStamper,
    pub adc_dac_timer: timers::SamplingTimer,
    pub timestamp_timer: timers::TimestampTimer,
    pub net: NetworkDevices,
    pub digital_inputs: (DigitalInput0, DigitalInput1),
    pub eem: Eem,
    pub usb_serial: SerialTerminal<C, Y>,
    pub usb: UsbDevice,
    pub metadata: &'static ApplicationMetadata,
    pub settings: C,
}

/// The available Pounder-specific hardware interfaces.
pub struct PounderDevices {
    pub pounder: pounder::PounderDevices,
    pub dds_output: DdsOutput,

    #[cfg(not(feature = "pounder_v1_0"))]
    pub timestamper: pounder::timestamp::Timestamper,
}

#[link_section = ".sram3.eth"]
/// Static storage for the ethernet DMA descriptor ring.
static mut DES_RING: MaybeUninit<
    ethernet::DesRing<{ super::TX_DESRING_CNT }, { super::RX_DESRING_CNT }>,
> = MaybeUninit::uninit();

/// Setup ITCM and load its code from flash.
///
/// For portability and maintainability this is implemented in Rust.
/// Since this is implemented in Rust the compiler may assume that bss and data are set
/// up already. There is no easy way to ensure this implementation will never need bss
/// or data. Hence we can't safely run this as the cortex-m-rt `pre_init` hook before
/// bss/data is setup.
///
/// Calling (through IRQ or directly) any code in ITCM before having called
/// this method is undefined.
fn load_itcm() {
    extern "C" {
        // ZST (`()`: not layout-stable. empty/zst struct in `repr(C)``: not "proper" C)
        static mut __sitcm: [u32; 0];
        static mut __eitcm: [u32; 0];
        static mut __siitcm: [u32; 0];
    }
    // NOTE(unsafe): Assuming the address symbols from the linker as well as
    // the source instruction data are all valid, this is safe as it only
    // copies linker-prepared data to where the code expects it to be.
    // Calling it multiple times is safe as well.

    unsafe {
        // ITCM is enabled on reset on our CPU but might not be on others.
        // Keep for completeness.
        const ITCMCR: *mut u32 = 0xE000_EF90usize as _;
        ptr::write_volatile(ITCMCR, ptr::read_volatile(ITCMCR) | 1);

        // Ensure ITCM is enabled before loading.
        atomic::fence(Ordering::SeqCst);

        let sitcm = core::ptr::addr_of_mut!(__sitcm) as *mut u32;
        let eitcm = core::ptr::addr_of!(__eitcm) as *const u32;
        let siitcm = core::ptr::addr_of!(__siitcm) as *const u32;

        let len = eitcm.offset_from(sitcm) as usize;
        let dst = slice::from_raw_parts_mut(sitcm, len);
        let src = slice::from_raw_parts(siitcm, len);
        // Load code into ITCM.
        dst.copy_from_slice(src);
    }

    // Ensure ITCM is loaded before potentially executing any instructions from it.
    atomic::fence(Ordering::SeqCst);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Configure the stabilizer hardware for operation.
///
/// # Note
/// Refer to [design_parameters::TIMER_FREQUENCY] to determine the frequency of the sampling timer.
///
/// # Args
/// * `core` - The cortex-m peripherals.
/// * `device` - The microcontroller peripherals to be configured.
/// * `clock` - A `SystemTimer` implementing `Clock`.
/// * `batch_size` - The size of each ADC/DAC batch.
/// * `sample_ticks` - The number of timer ticks between each sample.
///
/// # Returns
/// (stabilizer, pounder) where `stabilizer` is a `StabilizerDevices` structure containing all
/// stabilizer hardware interfaces in a disabled state. `pounder` is an `Option` containing
/// `Some(devices)` if pounder is detected, where `devices` is a `PounderDevices` structure
/// containing all of the pounder hardware interfaces in a disabled state.
pub fn setup<C, const Y: usize>(
    mut core: stm32h7xx_hal::stm32::CorePeripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
    clock: SystemTimer,
    batch_size: usize,
    sample_ticks: u32,
) -> (StabilizerDevices<C, Y>, Option<PounderDevices>)
where
    C: serial_settings::Settings + AppSettings,
{
    // Set up RTT logging
    {
        // Enable debug during WFE/WFI-induced sleep
        device.DBGMCU.cr.modify(|_, w| w.dbgsleep_d1().set_bit());

        // Set up RTT channel to use for `rprintln!()` as "best effort".
        // This removes a critical section around the logging and thus allows
        // high-prio tasks to always interrupt at low latency.
        // It comes at a cost:
        // If a high-priority tasks preempts while we are logging something,
        // and if we then also want to log from within that high-preiority task,
        // the high-prio log message will be lost.

        let channels = rtt_target::rtt_init_default!();
        // Note(unsafe): The closure we pass does not establish a critical section
        // as demanded but it does ensure synchronization and implements a lock.
        unsafe {
            rtt_target::set_print_channel_cs(
                channels.up.0,
                &((|arg, f| {
                    static LOCKED: AtomicBool = AtomicBool::new(false);
                    if LOCKED.compare_exchange_weak(
                        false,
                        true,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    ) == Ok(false)
                    {
                        f(arg);
                        LOCKED.store(false, Ordering::Release);
                    }
                }) as rtt_target::CriticalSectionFunc),
            );
        }

        static LOGGER: rtt_logger::RTTLogger =
            rtt_logger::RTTLogger::new(log::LevelFilter::Info);
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        log::info!("Starting");
    }

    // Check for a reboot to DFU before doing any system configuration.
    if platform::dfu_bootflag() {
        platform::execute_system_bootloader();
    }

    let pwr = device.PWR.constrain();
    let vos = pwr.freeze();

    // Enable SRAM3 for the ethernet descriptor ring.
    device.RCC.ahb2enr.modify(|_, w| w.sram3en().set_bit());

    // Clear reset flags.
    device.RCC.rsr.write(|w| w.rmvf().set_bit());

    // Select the PLLs for SPI.
    device
        .RCC
        .d2ccip1r
        .modify(|_, w| w.spi123sel().pll2_p().spi45sel().pll2_q());

    device.RCC.d1ccipr.modify(|_, w| w.qspisel().rcc_hclk3());

    device
        .RCC
        .d3ccipr
        .modify(|_, w| w.adcsel().per().spi6sel().pll2_q());

    let rcc = device.RCC.constrain();
    let mut ccdr = rcc
        .use_hse(8.MHz())
        .sysclk(design_parameters::SYSCLK.convert())
        .hclk(200.MHz())
        .per_ck(64.MHz()) // fixed frequency HSI, only used for internal ADC. This is not the "peripheral" clock for timers and others.
        .pll2_p_ck(100.MHz())
        .pll2_q_ck(100.MHz())
        .freeze(vos, &device.SYSCFG);

    // Set up USB clocks.
    ccdr.clocks.hsi48_ck().unwrap();
    ccdr.peripheral
        .kernel_usb_clk_mux(stm32h7xx_hal::rcc::rec::UsbClkSel::Hsi48);

    // Before being able to call any code in ITCM, load that code from flash.
    load_itcm();

    Systick::start(core.SYST, ccdr.clocks.sysclk().to_Hz());

    // After ITCM loading.
    core.SCB.enable_icache();

    let mut delay = delay::AsmDelay::new(ccdr.clocks.c_ck().to_Hz());

    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let mut gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    let dma_streams =
        hal::dma::dma::StreamsTuple::new(device.DMA1, ccdr.peripheral.DMA1);

    // Verify that batch period does not exceed RTIC Monotonic timer period.
    assert!(
        (batch_size as u32 * sample_ticks) as f32
            * design_parameters::TIMER_PERIOD
            * (super::MONOTONIC_FREQUENCY as f32)
            < 1.
    );

    // Configure timer 2 to trigger conversions for the ADC
    let mut sampling_timer = {
        // The timer frequency is manually adjusted below, so the 1KHz setting here is a
        // dont-care.
        let mut timer2 =
            device
                .TIM2
                .timer(1.kHz(), ccdr.peripheral.TIM2, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer2.pause();
        timer2.set_tick_freq(design_parameters::TIMER_FREQUENCY.convert());

        let mut sampling_timer = timers::SamplingTimer::new(timer2);
        sampling_timer.set_period_ticks(sample_ticks - 1);

        // The sampling timer is used as the master timer for the shadow-sampling timer. Thus,
        // it generates a trigger whenever it is enabled.

        sampling_timer
    };

    let mut shadow_sampling_timer = {
        // The timer frequency is manually adjusted below, so the 1KHz setting here is a
        // dont-care.
        let mut timer3 =
            device
                .TIM3
                .timer(1.kHz(), ccdr.peripheral.TIM3, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer3.pause();
        timer3.reset_counter();
        timer3.set_tick_freq(design_parameters::TIMER_FREQUENCY.convert());

        let mut shadow_sampling_timer =
            timers::ShadowSamplingTimer::new(timer3);
        shadow_sampling_timer.set_period_ticks(sample_ticks as u16 - 1);

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
            device
                .TIM5
                .timer(1.kHz(), ccdr.peripheral.TIM5, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer5.pause();
        timer5.set_tick_freq(design_parameters::TIMER_FREQUENCY.convert());

        // The timestamp timer runs at the counter cycle period as the sampling timers.
        // To accomodate this, we manually set the prescaler identical to the sample
        // timer, but use maximum overflow period.
        let mut timer = timers::TimestampTimer::new(timer5);

        // TODO: Check hardware synchronization of timestamping and the sampling timers
        // for phase shift determinism.

        timer.set_period_ticks(u32::MAX);

        timer
    };

    let timestamp_timer_channels = timestamp_timer.channels();

    // Configure the SPI interfaces to the ADCs and DACs.
    let adcs = {
        let adc0 = {
            let miso = gpiob.pb14.into_alternate().speed(Speed::VeryHigh);
            let sck = gpiob.pb10.into_alternate().speed(Speed::VeryHigh);
            let nss = gpiob.pb9.into_alternate().speed(Speed::VeryHigh);

            let config = hal::spi::Config::new(hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            })
            .hardware_cs(hal::spi::HardwareCS {
                mode: hal::spi::HardwareCSMode::WordTransaction,
                assertion_delay: design_parameters::ADC_SETUP_TIME,
                polarity: hal::spi::Polarity::IdleHigh,
            })
            .communication_mode(hal::spi::CommunicationMode::Receiver);

            let spi: hal::spi::Spi<_, _, u16> = device.SPI2.spi(
                (sck, miso, hal::spi::NoMosi, nss),
                config,
                design_parameters::ADC_DAC_SCK_MAX.convert(),
                ccdr.peripheral.SPI2,
                &ccdr.clocks,
            );

            adc::Adc0Input::new(
                spi,
                dma_streams.0,
                dma_streams.1,
                dma_streams.2,
                sampling_timer_channels.ch1,
                shadow_sampling_timer_channels.ch1,
                batch_size,
            )
        };

        let adc1 = {
            let miso = gpiob.pb4.into_alternate().speed(Speed::VeryHigh);
            let sck = gpioc.pc10.into_alternate().speed(Speed::VeryHigh);
            let nss = gpioa.pa15.into_alternate().speed(Speed::VeryHigh);

            let config = hal::spi::Config::new(hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            })
            .hardware_cs(hal::spi::HardwareCS {
                mode: hal::spi::HardwareCSMode::WordTransaction,
                assertion_delay: design_parameters::ADC_SETUP_TIME,
                polarity: hal::spi::Polarity::IdleHigh,
            })
            .communication_mode(hal::spi::CommunicationMode::Receiver);

            let spi: hal::spi::Spi<_, _, u16> = device.SPI3.spi(
                (sck, miso, hal::spi::NoMosi, nss),
                config,
                design_parameters::ADC_DAC_SCK_MAX.convert(),
                ccdr.peripheral.SPI3,
                &ccdr.clocks,
            );

            adc::Adc1Input::new(
                spi,
                dma_streams.3,
                dma_streams.4,
                dma_streams.5,
                sampling_timer_channels.ch2,
                shadow_sampling_timer_channels.ch2,
                batch_size,
            )
        };

        (adc0, adc1)
    };

    let dacs = {
        let mut dac_clr_n = gpioe.pe12.into_push_pull_output();
        dac_clr_n.set_high();

        let dac0_spi = {
            let miso = gpioe.pe5.into_alternate().speed(Speed::VeryHigh);
            let sck = gpioe.pe2.into_alternate().speed(Speed::VeryHigh);
            let nss = gpioe.pe4.into_alternate().speed(Speed::VeryHigh);

            let config = hal::spi::Config::new(hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            })
            .hardware_cs(hal::spi::HardwareCS {
                mode: hal::spi::HardwareCSMode::WordTransaction,
                assertion_delay: 0.0,
                polarity: hal::spi::Polarity::IdleHigh,
            })
            .communication_mode(hal::spi::CommunicationMode::Transmitter)
            .swap_mosi_miso();

            device.SPI4.spi(
                (sck, miso, hal::spi::NoMosi, nss),
                config,
                design_parameters::ADC_DAC_SCK_MAX.convert(),
                ccdr.peripheral.SPI4,
                &ccdr.clocks,
            )
        };

        let dac1_spi = {
            let miso = gpiof.pf8.into_alternate().speed(Speed::VeryHigh);
            let sck = gpiof.pf7.into_alternate().speed(Speed::VeryHigh);
            let nss = gpiof.pf6.into_alternate().speed(Speed::VeryHigh);

            let config = hal::spi::Config::new(hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            })
            .hardware_cs(hal::spi::HardwareCS {
                mode: hal::spi::HardwareCSMode::WordTransaction,
                assertion_delay: 0.0,
                polarity: hal::spi::Polarity::IdleHigh,
            })
            .communication_mode(hal::spi::CommunicationMode::Transmitter)
            .swap_mosi_miso();

            device.SPI5.spi(
                (sck, miso, hal::spi::NoMosi, nss),
                config,
                design_parameters::ADC_DAC_SCK_MAX.convert(),
                ccdr.peripheral.SPI5,
                &ccdr.clocks,
            )
        };

        let dac0 = dac::Dac0Output::new(
            dac0_spi,
            dma_streams.6,
            sampling_timer_channels.ch3,
            batch_size,
        );
        let dac1 = dac::Dac1Output::new(
            dac1_spi,
            dma_streams.7,
            sampling_timer_channels.ch4,
            batch_size,
        );

        dac_clr_n.set_low();
        // dac0_ldac_n
        gpioe.pe11.into_push_pull_output().set_low();
        // dac1_ldac_n
        gpioe.pe15.into_push_pull_output().set_low();
        dac_clr_n.set_high();

        (dac0, dac1)
    };

    let afes = {
        // AFE_PWR_ON on hardware revision v1.3.2
        gpioe.pe1.into_push_pull_output().set_high();

        let afe0 = afe::ProgrammableGainAmplifier::new([
            gpiof.pf2.into_push_pull_output().erase().forward(),
            gpiof.pf5.into_push_pull_output().erase().forward(),
        ]);

        let afe1 = afe::ProgrammableGainAmplifier::new([
            gpiod.pd14.into_push_pull_output().erase().forward(),
            gpiod.pd15.into_push_pull_output().erase().forward(),
        ]);

        [afe0, afe1]
    };

    let input_stamper = {
        let trigger = gpioa.pa3.into_alternate();
        InputStamper::new(trigger, timestamp_timer_channels.ch4)
    };

    let digital_inputs = {
        let di0 = gpiog.pg9.into_floating_input();
        let di1 = gpioc.pc15.into_floating_input();
        (di0, di1)
    };

    let mut eeprom_i2c = {
        let sda = gpiof.pf0.into_alternate().set_open_drain();
        let scl = gpiof.pf1.into_alternate().set_open_drain();
        device.I2C2.i2c(
            (scl, sda),
            100.kHz(),
            ccdr.peripheral.I2C2,
            &ccdr.clocks,
        )
    };

    let metadata = {
        // Read the hardware version pins.
        let hardware_version = HardwareVersion::from(
            &[
                gpiog.pg0.into_pull_down_input().is_high(),
                gpiog.pg1.into_pull_down_input().is_high(),
                gpiog.pg2.into_pull_down_input().is_high(),
                gpiog.pg3.into_pull_down_input().is_high(),
            ][..],
        );
        ApplicationMetadata::new(hardware_version)
    };

    let mac_addr = smoltcp::wire::EthernetAddress(eeprom::read_eui48(
        &mut eeprom_i2c,
        &mut delay,
    ));
    log::info!("EUI48: {}", mac_addr);

    let (flash, mut settings) = {
        let mut flash = {
            let (_, flash_bank2) = device.FLASH.split();
            super::flash::Flash(flash_bank2.unwrap())
        };

        let mut settings = C::new(NetSettings::new(mac_addr));
        crate::settings::SerialSettingsPlatform::load(
            &mut settings,
            &mut flash,
        );
        (flash, settings)
    };

    let network_devices = {
        let ethernet_pins = {
            // Reset the PHY before configuring pins.
            let mut eth_phy_nrst = gpioe.pe3.into_push_pull_output();
            eth_phy_nrst.set_low();
            delay.delay_us(200u8);
            eth_phy_nrst.set_high();

            let ref_clk = gpioa.pa1.into_alternate().speed(Speed::VeryHigh);
            let mdio = gpioa.pa2.into_alternate().speed(Speed::VeryHigh);
            let mdc = gpioc.pc1.into_alternate().speed(Speed::VeryHigh);
            let crs_dv = gpioa.pa7.into_alternate().speed(Speed::VeryHigh);
            let rxd0 = gpioc.pc4.into_alternate().speed(Speed::VeryHigh);
            let rxd1 = gpioc.pc5.into_alternate().speed(Speed::VeryHigh);
            let tx_en = gpiob.pb11.into_alternate().speed(Speed::VeryHigh);
            let txd0 = gpiob.pb12.into_alternate().speed(Speed::VeryHigh);
            let txd1 = gpiog.pg14.into_alternate().speed(Speed::VeryHigh);

            (ref_clk, mdio, mdc, crs_dv, rxd0, rxd1, tx_en, txd0, txd1)
        };

        let ring = unsafe { DES_RING.write(ethernet::DesRing::new()) };

        // Configure the ethernet controller
        let (mut eth_dma, eth_mac) = ethernet::new(
            device.ETHERNET_MAC,
            device.ETHERNET_MTL,
            device.ETHERNET_DMA,
            ethernet_pins,
            // Note(unsafe): We only call this function once to take ownership of the
            // descriptor ring.
            ring,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        );

        // Reset and initialize the ethernet phy.
        let mut lan8742a =
            ethernet::phy::LAN8742A::new(eth_mac.set_phy_addr(0));
        lan8742a.phy_reset();
        lan8742a.phy_init();

        unsafe { ethernet::enable_interrupt() };

        // Configure IP address according to DHCP socket availability
        let ip_addrs: smoltcp::wire::IpAddress = match settings.net().ip.parse()
        {
            Ok(addr) => addr,
            Err(e) => {
                log::warn!("Invalid IP address in settings: {e:?}. Defaulting to 0.0.0.0 (DHCP)");
                "0.0.0.0".parse().unwrap()
            }
        };

        let random_seed = {
            let mut rng =
                device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 8];
            rng.fill(&mut data).unwrap();
            data
        };

        // Note(unwrap): The hardware configuration function is only allowed to be called once.
        // Unwrapping is intended to panic if called again to prevent re-use of global memory.
        let store =
            cortex_m::singleton!(: NetStorage = NetStorage::default()).unwrap();

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(ip_addrs, 24);

        let mut ethernet_config = smoltcp::iface::Config::new(
            smoltcp::wire::HardwareAddress::Ethernet(mac_addr),
        );
        ethernet_config.random_seed = u64::from_be_bytes(random_seed);

        let mut interface = smoltcp::iface::Interface::new(
            ethernet_config,
            &mut eth_dma,
            smoltcp::time::Instant::ZERO,
        );

        interface
            .routes_mut()
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .unwrap();

        interface.update_ip_addrs(|ref mut addrs| {
            if !ip_addrs.is_unspecified() {
                addrs
                    .push(smoltcp::wire::IpCidr::new(ip_addrs, 24))
                    .unwrap();
            }
        });

        let mut sockets =
            smoltcp::iface::SocketSet::new(&mut store.sockets[..]);
        for storage in store.tcp_socket_storage[..].iter_mut() {
            let tcp_socket = {
                let rx_buffer = smoltcp::socket::tcp::SocketBuffer::new(
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::tcp::SocketBuffer::new(
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::tcp::Socket::new(rx_buffer, tx_buffer)
            };

            sockets.add(tcp_socket);
        }

        if ip_addrs.is_unspecified() {
            sockets.add(smoltcp::socket::dhcpv4::Socket::new());
        }

        sockets.add(smoltcp::socket::dns::Socket::new(
            &[],
            &mut store.dns_storage[..],
        ));

        for storage in store.udp_socket_storage[..].iter_mut() {
            let udp_socket = {
                let rx_buffer = smoltcp::socket::udp::PacketBuffer::new(
                    &mut storage.rx_metadata[..],
                    &mut storage.rx_storage[..],
                );
                let tx_buffer = smoltcp::socket::udp::PacketBuffer::new(
                    &mut storage.tx_metadata[..],
                    &mut storage.tx_storage[..],
                );

                smoltcp::socket::udp::Socket::new(rx_buffer, tx_buffer)
            };

            sockets.add(udp_socket);
        }

        let mut stack =
            smoltcp_nal::NetworkStack::new(interface, eth_dma, sockets, clock);

        stack.seed_random_port(&random_seed);

        NetworkDevices {
            stack,
            phy: lan8742a,
            mac_address: mac_addr,
        }
    };

    let mut fp_led_0 = gpiod.pd5.into_push_pull_output();
    let mut fp_led_1 = gpiod.pd6.into_push_pull_output();
    let mut fp_led_2 = gpiog.pg4.into_push_pull_output();
    let mut fp_led_3 = gpiod.pd12.into_push_pull_output();

    fp_led_0.set_low();
    fp_led_1.set_low();
    fp_led_2.set_low();
    fp_led_3.set_low();

    let (adc1, adc2, adc3) = {
        let (mut adc1, mut adc2) = hal::adc::adc12(
            device.ADC1,
            device.ADC2,
            stm32h7xx_hal::time::Hertz::MHz(25),
            &mut delay,
            ccdr.peripheral.ADC12,
            &ccdr.clocks,
        );
        let mut adc3 = hal::adc::Adc::adc3(
            device.ADC3,
            stm32h7xx_hal::time::Hertz::MHz(25),
            &mut delay,
            ccdr.peripheral.ADC3,
            &ccdr.clocks,
        );

        adc1.set_sample_time(hal::adc::AdcSampleTime::T_810);
        adc1.set_resolution(hal::adc::Resolution::SixteenBit);
        adc1.calibrate();
        adc2.set_sample_time(hal::adc::AdcSampleTime::T_810);
        adc2.set_resolution(hal::adc::Resolution::SixteenBit);
        adc2.calibrate();
        adc3.set_sample_time(hal::adc::AdcSampleTime::T_810);
        adc3.set_resolution(hal::adc::Resolution::SixteenBit);
        adc3.calibrate();

        hal::adc::Temperature::new().enable(&adc3);

        let adc1 = adc1.enable();
        let adc2 = adc2.enable();
        let adc3 = adc3.enable();

        (
            // The ADCs must live as global, mutable singletons so that we can hand out references
            // to the internal ADC. If they were instead to live within e.g. StabilizerDevices,
            // they would not yet live in 'static memory, which means that we could not hand out
            // references during initialization, since those references would be invalidated when
            // we move StabilizerDevices into the late RTIC resources.
            cortex_m::singleton!(: SharedAdc<hal::stm32::ADC1> = SharedAdc::new(adc1.slope() as f32, adc1)).unwrap(),
            cortex_m::singleton!(: SharedAdc<hal::stm32::ADC2> = SharedAdc::new(adc2.slope() as f32, adc2)).unwrap(),
            cortex_m::singleton!(: SharedAdc<hal::stm32::ADC3> = SharedAdc::new(adc3.slope() as f32, adc3)).unwrap(),
        )
    };

    // Measure the Pounder PGOOD output to detect if pounder is present on Stabilizer.
    let pounder_pgood = gpiob.pb13.into_pull_down_input();
    delay.delay_ms(2u8);
    let pounder = if pounder_pgood.is_high() {
        log::info!("Found Pounder");

        let i2c1 = {
            let sda = gpiob.pb7.into_alternate().set_open_drain();
            let scl = gpiob.pb8.into_alternate().set_open_drain();
            let i2c1 = device.I2C1.i2c(
                (scl, sda),
                400.kHz(),
                ccdr.peripheral.I2C1,
                &ccdr.clocks,
            );

            shared_bus::new_atomic_check!(hal::i2c::I2c<hal::stm32::I2C1> = i2c1).unwrap()
        };

        let spi = {
            let mosi = gpiod.pd7.into_alternate();
            let miso = gpioa.pa6.into_alternate();
            let sck = gpiog.pg11.into_alternate();

            let config = hal::spi::Config::new(hal::spi::Mode {
                polarity: hal::spi::Polarity::IdleHigh,
                phase: hal::spi::Phase::CaptureOnSecondTransition,
            });

            // The maximum frequency of this SPI must be limited due to capacitance on the MISO
            // line causing a long RC decay.
            device.SPI1.spi(
                (sck, miso, mosi),
                config,
                5.MHz(),
                ccdr.peripheral.SPI1,
                &ccdr.clocks,
            )
        };

        let pwr0 = adc1.create_channel(gpiof.pf11.into_analog());
        let pwr1 = adc2.create_channel(gpiof.pf14.into_analog());
        let aux_adc0 = adc3.create_channel(gpiof.pf3.into_analog());
        let aux_adc1 = adc3.create_channel(gpiof.pf4.into_analog());

        let pounder_devices = pounder::PounderDevices::new(
            i2c1.acquire_i2c(),
            spi,
            (pwr0, pwr1),
            (aux_adc0, aux_adc1),
        )
        .unwrap();

        let ad9959 = {
            let qspi_interface = {
                // Instantiate the QUADSPI pins and peripheral interface.
                let qspi_pins = {
                    let _ncs =
                        gpioc.pc11.into_alternate::<9>().speed(Speed::VeryHigh);

                    let clk = gpiob.pb2.into_alternate().speed(Speed::VeryHigh);
                    let io0 = gpioe.pe7.into_alternate().speed(Speed::VeryHigh);
                    let io1 = gpioe.pe8.into_alternate().speed(Speed::VeryHigh);
                    let io2 = gpioe.pe9.into_alternate().speed(Speed::VeryHigh);
                    let io3 =
                        gpioe.pe10.into_alternate().speed(Speed::VeryHigh);

                    (clk, io0, io1, io2, io3)
                };

                let qspi = device.QUADSPI.bank2(
                    qspi_pins,
                    design_parameters::POUNDER_QSPI_FREQUENCY.convert(),
                    &ccdr.clocks,
                    ccdr.peripheral.QSPI,
                );

                pounder::QspiInterface::new(qspi).unwrap()
            };

            #[cfg(not(feature = "pounder_v1_0"))]
            let reset_pin = gpiog.pg6.into_push_pull_output();
            #[cfg(feature = "pounder_v1_0")]
            let reset_pin = gpioa.pa0.into_push_pull_output();

            let mut io_update = gpiog.pg7.into_push_pull_output();

            // Delay to allow the pounder DDS reference clock to fully start up. The exact startup
            // time is not specified, but bench testing indicates it usually comes up within
            // 200-300uS. We do a larger delay to ensure that it comes up and is stable before
            // using it.
            delay.delay_ms(10u32);

            let mut ad9959 = ad9959::Ad9959::new(
                qspi_interface,
                reset_pin,
                &mut io_update,
                &mut delay,
                ad9959::Mode::FourBitSerial,
                design_parameters::DDS_REF_CLK.to_Hz() as f32,
                design_parameters::DDS_MULTIPLIER,
            )
            .unwrap();

            ad9959.self_test().unwrap();

            // Return IO_Update
            gpiog.pg7 = io_update.into_analog();

            ad9959
        };

        let dds_output = {
            let io_update_trigger = {
                let _io_update =
                    gpiog.pg7.into_alternate::<2>().speed(Speed::VeryHigh);

                // Configure the IO_Update signal for the DDS.
                let mut hrtimer = pounder::hrtimer::HighResTimerE::new(
                    device.HRTIM_TIME,
                    device.HRTIM_MASTER,
                    device.HRTIM_COMMON,
                    ccdr.clocks,
                    ccdr.peripheral.HRTIM,
                );

                // IO_Update occurs after a fixed delay from the QSPI write. Note that the timer
                // is triggered after the QSPI write, which can take approximately 120nS, so
                // there is additional margin.
                hrtimer.configure_single_shot(
                    pounder::hrtimer::Channel::Two,
                    design_parameters::POUNDER_IO_UPDATE_DELAY,
                    design_parameters::POUNDER_IO_UPDATE_DURATION,
                );

                // Ensure that we have enough time for an IO-update every batch.
                let sample_frequency = {
                    design_parameters::TIMER_FREQUENCY.to_Hz() as f32
                        / sample_ticks as f32
                };

                let sample_period = 1.0 / sample_frequency;
                assert!(
                    sample_period * batch_size as f32
                        > design_parameters::POUNDER_IO_UPDATE_DELAY
                );

                hrtimer
            };

            let (qspi, config) = ad9959.freeze();
            DdsOutput::new(qspi, io_update_trigger, config)
        };

        #[cfg(not(feature = "pounder_v1_0"))]
        let pounder_stamper = {
            log::info!("Assuming Pounder v1.1 or later");
            let etr_pin = gpioa.pa0.into_alternate();

            // The frequency in the constructor is dont-care, as we will modify the period + clock
            // source manually below.
            let tim8 =
                device
                    .TIM8
                    .timer(1.kHz(), ccdr.peripheral.TIM8, &ccdr.clocks);
            let mut timestamp_timer = timers::PounderTimestampTimer::new(tim8);

            // Pounder is configured to generate a 500MHz reference clock, so a 125MHz sync-clock is
            // output. As a result, dividing the 125MHz sync-clk provides a 31.25MHz tick rate for
            // the timestamp timer. 31.25MHz corresponds with a 32ns tick rate.
            // This is less than fCK_INT/3 of the timer as required for oversampling the trigger.
            timestamp_timer.set_external_clock(timers::Prescaler::Div4);
            timestamp_timer.start();

            // Set the timer to wrap at the u16 boundary to meet the PLL periodicity.
            // Scale and wrap before or after the PLL.
            timestamp_timer.set_period_ticks(u16::MAX);
            let tim8_channels = timestamp_timer.channels();

            pounder::timestamp::Timestamper::new(
                timestamp_timer,
                tim8_channels.ch1,
                &mut sampling_timer,
                etr_pin,
                batch_size,
            )
        };

        Some(PounderDevices {
            pounder: pounder_devices,
            dds_output,

            #[cfg(not(feature = "pounder_v1_0"))]
            timestamper: pounder_stamper,
        })
    } else {
        None
    };

    #[derive(Copy, Clone, Debug, PartialEq)]
    pub enum PoePower {
        /// No Power over Ethernet detected
        Absent,
        /// 802.3af (12.95 W) Power over Ethernet present
        Low,
        /// 802.3at (25.5 W) Power over Ethernet present
        High,
    }

    let poe_src_status = gpiob.pb6.into_floating_input();
    let poe_at_event = gpioc.pc14.into_floating_input();
    let poe = match (poe_src_status.is_high(), poe_at_event.is_high()) {
        (false, _) => PoePower::Absent,
        (true, false) => PoePower::Low,
        (true, true) => PoePower::High,
    };
    log::info!("PoE: {poe:?}",);

    let mut force_eem_source = gpioe.pe0.into_push_pull_output();

    // checks whether a pin can be weakly pulled high an low
    fn check_input<const P: char, const N: u8>(
        pin: hal::gpio::Pin<P, N, hal::gpio::Analog>,
        mut is_floating: bool,
        delay: &mut AsmDelay,
    ) -> (hal::gpio::Pin<P, N, hal::gpio::Analog>, bool) {
        let pin = pin.into_pull_up_input();
        delay.delay_ms(1u8);
        is_floating &= pin.is_high();
        let pin = pin.into_pull_down_input();
        delay.delay_ms(1u8);
        is_floating &= pin.is_low();
        (pin.into_analog(), is_floating)
    }

    let mut lvds0 = gpiog.pg13;
    let mut lvds1 = gpiob.pb5;
    let mut lvds3 = gpiog.pg8;
    let mut is_floating = true;
    // Default EEM population: LVDS0/1/3 driven by LVDS receivers
    (lvds0, is_floating) = check_input(lvds0, is_floating, &mut delay);
    (lvds1, is_floating) = check_input(lvds1, is_floating, &mut delay);
    (lvds3, is_floating) = check_input(lvds3, is_floating, &mut delay);
    let eem = if !is_floating {
        log::info!("EEM population variant: Default detected");
        force_eem_source.set_low();
        Eem::Gpio(Gpio {
            lvds4: gpiod.pd1.into_floating_input(),
            lvds5: gpiod.pd2.into_floating_input(),
            lvds6: gpiod.pd3.into_push_pull_output(),
            lvds7: gpiod.pd4.into_push_pull_output(),
        })
    } else {
        log::info!("EEM population variant: Output detected");
        if poe != PoePower::Low {
            log::warn!("No 802.3at PoE. Powering up EEM anyway.");
        }
        force_eem_source.set_high();
        delay.delay_ms(200u8);

        let spi = device
            .SPI6
            .spi(
                (
                    lvds0.into_alternate(),      // SCK
                    gpiog.pg12.into_alternate(), // MISO/SDO
                    lvds1.into_alternate(),      // MOSI/SDI
                ),
                hal::spi::MODE_0,
                20.MHz(),
                ccdr.peripheral.SPI6,
                &ccdr.clocks,
            )
            .forward();
        let spi = cortex_m::singleton!(:
            RefCell<Forward<hal::spi::Spi<hal::stm32::SPI6, hal::spi::Enabled>>> =
                RefCell::new(spi)).unwrap();

        let cs = [
            lvds3.into_push_pull_output().erase().forward(),
            gpiod.pd1.into_push_pull_output().erase().forward(),
            gpiod.pd2.into_push_pull_output().erase().forward(),
        ];
        let cs = cortex_m::singleton!(:
            RefCell<[Forward<hal::gpio::ErasedPin<hal::gpio::Output>, ForwardOutputPin>; 3]> =
                RefCell::new(cs)
        )
        .unwrap();

        match urukul::Urukul::new(
            spi,
            cs,
            gpiod.pd3.into_push_pull_output().erase().forward(),
            gpiod.pd4.into_push_pull_output().erase().forward(),
        ) {
            Ok(urukul) => Eem::Urukul(urukul),
            Err(err) => {
                log::warn!("Urukul initialization failed: {err:?}");
                Eem::None
            }
        }
    };

    let (usb_device, usb_serial) = {
        let _usb_id = gpioa.pa10.into_alternate::<10>();
        let usb_n = gpioa.pa11.into_alternate();
        let usb_p = gpioa.pa12.into_alternate();
        let usb = stm32h7xx_hal::usb_hs::USB2::new(
            device.OTG2_HS_GLOBAL,
            device.OTG2_HS_DEVICE,
            device.OTG2_HS_PWRCLK,
            usb_n,
            usb_p,
            ccdr.peripheral.USB2OTG,
            &ccdr.clocks,
        );

        let endpoint_memory =
            cortex_m::singleton!(: Option<&'static mut [u32]> = None).unwrap();
        endpoint_memory.replace(
            &mut cortex_m::singleton!(: [u32; 1024] = [0; 1024]).unwrap()[..],
        );
        let usb_bus = cortex_m::singleton!(: usb_device::bus::UsbBusAllocator<super::UsbBus> =
        stm32h7xx_hal::usb_hs::UsbBus::new(
            usb,
            endpoint_memory.take().unwrap(),
        ))
        .unwrap();

        let read_store = cortex_m::singleton!(: [u8; 128] = [0; 128]).unwrap();
        let write_store =
            cortex_m::singleton!(: [u8; 1024] = [0; 1024]).unwrap();
        let serial = usbd_serial::SerialPort::new_with_store(
            usb_bus,
            &mut read_store[..],
            &mut write_store[..],
        );

        // Generate a device serial number from the MAC address.
        let serial_number = cortex_m::singleton!(: String<17> = {
            let mut s = String::new();
            write!(s, "{mac_addr}").unwrap();
            s
        })
        .unwrap();

        let usb_device = usb_device::device::UsbDeviceBuilder::new(
            usb_bus,
            usb_device::device::UsbVidPid(0x1209, 0x392F),
        )
        .strings(&[usb_device::device::StringDescriptors::default()
            .manufacturer("ARTIQ/Sinara")
            .product("Stabilizer")
            .serial_number(serial_number)])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        (usb_device, serial)
    };

    let usb_terminal = {
        let input_buffer =
            cortex_m::singleton!(: [u8; 128] = [0u8; 128]).unwrap();
        let serialize_buffer =
            cortex_m::singleton!(: [u8; 512] = [0u8; 512]).unwrap();

        serial_settings::Runner::new(
            crate::settings::SerialSettingsPlatform {
                interface: serial_settings::BestEffortInterface::new(
                    usb_serial,
                ),
                storage: flash,
                metadata,
                _settings_marker: core::marker::PhantomData,
            },
            input_buffer,
            serialize_buffer,
            &mut settings,
        )
        .unwrap()
    };

    let stabilizer = StabilizerDevices {
        afes,
        adcs,
        dacs,
        temperature_sensor: CpuTempSensor::new(
            adc3.create_channel(hal::adc::Temperature::new()),
        ),
        usb_serial: usb_terminal,
        timestamper: input_stamper,
        net: network_devices,
        adc_dac_timer: sampling_timer,
        timestamp_timer,
        digital_inputs,
        eem,
        usb: usb_device,
        metadata,
        settings,
    };

    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    // info!("Built on {}", build_info::BUILT_TIME_UTC);
    // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);
    log::info!("setup() complete");

    (stabilizer, pounder)
}
