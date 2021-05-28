///! Stabilizer hardware configuration
///!
///! This file contains all of the hardware-specific configuration of Stabilizer.
use stm32h7xx_hal::{
    self as hal,
    ethernet::{self, PHY},
    prelude::*,
};

use smoltcp_nal::smoltcp;

use embedded_hal::digital::v2::{InputPin, OutputPin};

use super::{
    adc, afe, cycle_counter::CycleCounter, dac, design_parameters,
    digital_input_stamper, eeprom, pounder, system_timer, timers, DdsOutput,
    DigitalInput0, DigitalInput1, EthernetPhy, NetworkStack, AFE0, AFE1,
};

const NUM_TCP_SOCKETS: usize = 5;
const NUM_UDP_SOCKETS: usize = 1;
const NUM_SOCKETS: usize = NUM_UDP_SOCKETS + NUM_TCP_SOCKETS;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],

    // Note: There is an additional socket set item required for the DHCP socket.
    pub sockets:
        [Option<smoltcp::socket::SocketSetItem<'static>>; NUM_SOCKETS + 1],
    pub tcp_socket_storage: [TcpSocketStorage; NUM_TCP_SOCKETS],
    pub udp_socket_storage: [UdpSocketStorage; NUM_UDP_SOCKETS],
    pub neighbor_cache:
        [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache:
        [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],

    pub dhcp_rx_metadata: [smoltcp::socket::RawPacketMetadata; 1],
    pub dhcp_tx_metadata: [smoltcp::socket::RawPacketMetadata; 1],
    pub dhcp_tx_storage: [u8; 600],
    pub dhcp_rx_storage: [u8; 600],
}

pub struct UdpSocketStorage {
    rx_storage: [u8; 1024],
    tx_storage: [u8; 1024],
    tx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
    rx_metadata: [smoltcp::storage::PacketMetadata<smoltcp::wire::IpEndpoint>; 10],
}

impl UdpSocketStorage {
    const fn new() -> Self {
        Self {
            rx_storage: [0; 1024],
            tx_storage: [0; 1024],
            tx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
            rx_metadata: [smoltcp::storage::PacketMetadata::<smoltcp::wire::IpEndpoint>::EMPTY; 10],
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

impl NetStorage {
    pub fn new() -> Self {
        NetStorage {
            // Placeholder for the real IP address, which is initialized at runtime.
            ip_addrs: [smoltcp::wire::IpCidr::Ipv6(
                smoltcp::wire::Ipv6Cidr::SOLICITED_NODE_PREFIX,
            )],
            neighbor_cache: [None; 8],
            routes_cache: [None; 8],
            sockets: [None, None, None, None, None, None, None],
            tcp_socket_storage: [TcpSocketStorage::new(); NUM_TCP_SOCKETS],
            udp_socket_storage: [UdpSocketStorage::new(); NUM_UDP_SOCKETS],
            dhcp_tx_storage: [0; 600],
            dhcp_rx_storage: [0; 600],
            dhcp_rx_metadata: [smoltcp::socket::RawPacketMetadata::EMPTY; 1],
            dhcp_tx_metadata: [smoltcp::socket::RawPacketMetadata::EMPTY; 1],
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
pub struct StabilizerDevices {
    pub afes: (AFE0, AFE1),
    pub adcs: (adc::Adc0Input, adc::Adc1Input),
    pub dacs: (dac::Dac0Output, dac::Dac1Output),
    pub timestamper: digital_input_stamper::InputStamper,
    pub adc_dac_timer: timers::SamplingTimer,
    pub timestamp_timer: timers::TimestampTimer,
    pub net: NetworkDevices,
    pub cycle_counter: CycleCounter,
    pub digital_inputs: (DigitalInput0, DigitalInput1),
}

/// The available Pounder-specific hardware interfaces.
pub struct PounderDevices {
    pub pounder: pounder::PounderDevices,
    pub dds_output: DdsOutput,

    #[cfg(feature = "pounder_v1_1")]
    pub timestamper: pounder::timestamp::Timestamper,
}

#[link_section = ".sram3.eth"]
/// Static storage for the ethernet DMA descriptor ring.
static mut DES_RING: ethernet::DesRing = ethernet::DesRing::new();

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
        static mut __sitcm: u32;
        static mut __eitcm: u32;
        static mut __siitcm: u32;
    }
    use core::{ptr, slice, sync::atomic};

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
        atomic::fence(atomic::Ordering::SeqCst);

        let len =
            (&__eitcm as *const u32).offset_from(&__sitcm as *const _) as usize;
        let dst = slice::from_raw_parts_mut(&mut __sitcm as *mut _, len);
        let src = slice::from_raw_parts(&__siitcm as *const _, len);
        // Load code into ITCM.
        dst.copy_from_slice(src);
    }

    // Ensure ITCM is loaded before potentially executing any instructions from it.
    atomic::fence(atomic::Ordering::SeqCst);
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Configure the stabilizer hardware for operation.
///
/// # Args
/// * `core` - The RTIC core for configuring the cortex-M core of the device.
/// * `device` - The microcontroller peripherals to be configured.
///
/// # Returns
/// (stabilizer, pounder) where `stabilizer` is a `StabilizerDevices` structure containing all
/// stabilizer hardware interfaces in a disabled state. `pounder` is an `Option` containing
/// `Some(devices)` if pounder is detected, where `devices` is a `PounderDevices` structure
/// containing all of the pounder hardware interfaces in a disabled state.
pub fn setup(
    mut core: rtic::Peripherals,
    device: stm32h7xx_hal::stm32::Peripherals,
) -> (StabilizerDevices, Option<PounderDevices>) {
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

    let rcc = device.RCC.constrain();
    let ccdr = rcc
        .use_hse(8.mhz())
        .sysclk(400.mhz())
        .hclk(200.mhz())
        .per_ck(100.mhz())
        .pll2_p_ck(100.mhz())
        .pll2_q_ck(100.mhz())
        .freeze(vos, &device.SYSCFG);

    // Set up RTT logging
    {
        // Enable debug during WFE/WFI-induced sleep
        device.DBGMCU.cr.modify(|_, w| w.dbgsleep_d1().set_bit());

        use rtt_logger::RTTLogger;

        static LOGGER: RTTLogger = RTTLogger::new(log::LevelFilter::Info);
        rtt_target::rtt_init_print!();
        log::set_logger(&LOGGER)
            .map(|()| log::set_max_level(log::LevelFilter::Trace))
            .unwrap();
        log::info!("Starting");
    }

    // Before being able to call any code in ITCM, load that code from flash.
    load_itcm();

    // Set up the system timer for RTIC scheduling.
    {
        let tim15 =
            device
                .TIM15
                .timer(10.khz(), ccdr.peripheral.TIM15, &ccdr.clocks);
        system_timer::SystemTimer::initialize(tim15);
    }

    let mut delay = asm_delay::AsmDelay::new(asm_delay::bitrate::Hertz(
        ccdr.clocks.c_ck().0,
    ));

    let gpioa = device.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = device.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = device.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = device.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpioe = device.GPIOE.split(ccdr.peripheral.GPIOE);
    let gpiof = device.GPIOF.split(ccdr.peripheral.GPIOF);
    let mut gpiog = device.GPIOG.split(ccdr.peripheral.GPIOG);

    let _uart_tx = gpiod.pd8.into_push_pull_output().set_speed(hal::gpio::Speed::VeryHigh);

    let dma_streams =
        hal::dma::dma::StreamsTuple::new(device.DMA1, ccdr.peripheral.DMA1);

    // Early, before the DMA1 peripherals (#272)
    #[cfg(feature = "pounder_v1_1")]
    let dma2_streams =
        hal::dma::dma::StreamsTuple::new(device.DMA2, ccdr.peripheral.DMA2);

    // Configure timer 2 to trigger conversions for the ADC
    let mut sampling_timer = {
        // The timer frequency is manually adjusted below, so the 1KHz setting here is a
        // dont-care.
        let mut timer2 =
            device
                .TIM2
                .timer(1.khz(), ccdr.peripheral.TIM2, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer2.pause();
        timer2.set_tick_freq(design_parameters::TIMER_FREQUENCY);

        let mut sampling_timer = timers::SamplingTimer::new(timer2);
        sampling_timer
            .set_period_ticks((design_parameters::ADC_SAMPLE_TICKS - 1) as u32);

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
                .timer(1.khz(), ccdr.peripheral.TIM3, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer3.pause();
        timer3.reset_counter();
        timer3.set_tick_freq(design_parameters::TIMER_FREQUENCY);

        let mut shadow_sampling_timer =
            timers::ShadowSamplingTimer::new(timer3);
        shadow_sampling_timer
            .set_period_ticks(design_parameters::ADC_SAMPLE_TICKS - 1);

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
                .timer(1.khz(), ccdr.peripheral.TIM5, &ccdr.clocks);

        // Configure the timer to count at the designed tick rate. We will manually set the
        // period below.
        timer5.pause();
        timer5.set_tick_freq(design_parameters::TIMER_FREQUENCY);

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

            let spi: hal::spi::Spi<_, _, u16> = device.SPI2.spi(
                (spi_sck, spi_miso, hal::spi::NoMosi),
                config,
                design_parameters::ADC_DAC_SCK_MAX,
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

            let spi: hal::spi::Spi<_, _, u16> = device.SPI3.spi(
                (spi_sck, spi_miso, hal::spi::NoMosi),
                config,
                design_parameters::ADC_DAC_SCK_MAX,
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
            )
        };

        (adc0, adc1)
    };

    let dacs = {
        let _dac_clr_n = gpioe.pe12.into_push_pull_output().set_high().unwrap();
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

            device.SPI4.spi(
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

            device.SPI5.spi(
                (spi_sck, spi_miso, hal::spi::NoMosi),
                config,
                design_parameters::ADC_DAC_SCK_MAX,
                ccdr.peripheral.SPI5,
                &ccdr.clocks,
            )
        };

        let dac0 = dac::Dac0Output::new(
            dac0_spi,
            dma_streams.6,
            sampling_timer_channels.ch3,
        );
        let dac1 = dac::Dac1Output::new(
            dac1_spi,
            dma_streams.7,
            sampling_timer_channels.ch4,
        );
        (dac0, dac1)
    };

    let afes = {
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

        (afe0, afe1)
    };

    let input_stamper = {
        let trigger = gpioa.pa3.into_alternate_af2();
        digital_input_stamper::InputStamper::new(
            trigger,
            timestamp_timer_channels.ch4,
        )
    };

    let digital_inputs = {
        let di0 = gpiog.pg9.into_floating_input();
        let di1 = gpioc.pc15.into_floating_input();
        (di0, di1)
    };

    let mut eeprom_i2c = {
        let sda = gpiof.pf0.into_alternate_af4().set_open_drain();
        let scl = gpiof.pf1.into_alternate_af4().set_open_drain();
        device.I2C2.i2c(
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

    let mac_addr = smoltcp::wire::EthernetAddress(eeprom::read_eui48(
        &mut eeprom_i2c,
        &mut delay,
    ));
    log::info!("EUI48: {}", mac_addr);

    let network_devices = {
        // Configure the ethernet controller
        let (eth_dma, eth_mac) = unsafe {
            ethernet::new_unchecked(
                device.ETHERNET_MAC,
                device.ETHERNET_MTL,
                device.ETHERNET_DMA,
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

        // Note(unwrap): The hardware configuration function is only allowed to be called once.
        // Unwrapping is intended to panic if called again to prevent re-use of global memory.
        let store =
            cortex_m::singleton!(: NetStorage = NetStorage::new()).unwrap();

        store.ip_addrs[0] = smoltcp::wire::IpCidr::new(
            smoltcp::wire::IpAddress::Ipv4(
                smoltcp::wire::Ipv4Address::UNSPECIFIED,
            ),
            0,
        );

        let mut routes =
            smoltcp::iface::Routes::new(&mut store.routes_cache[..]);
        routes
            .add_default_ipv4_route(smoltcp::wire::Ipv4Address::UNSPECIFIED)
            .unwrap();

        let neighbor_cache =
            smoltcp::iface::NeighborCache::new(&mut store.neighbor_cache[..]);

        let interface = smoltcp::iface::EthernetInterfaceBuilder::new(eth_dma)
            .ethernet_addr(mac_addr)
            .neighbor_cache(neighbor_cache)
            .ip_addrs(&mut store.ip_addrs[..])
            .routes(routes)
            .finalize();

        let mut sockets = {
            let mut sockets =
                smoltcp::socket::SocketSet::new(&mut store.sockets[..]);

            for storage in store.tcp_socket_storage[..].iter_mut() {
                let tcp_socket = {
                    let rx_buffer = smoltcp::socket::TcpSocketBuffer::new(
                        &mut storage.rx_storage[..],
                    );
                    let tx_buffer = smoltcp::socket::TcpSocketBuffer::new(
                        &mut storage.tx_storage[..],
                    );

                    smoltcp::socket::TcpSocket::new(rx_buffer, tx_buffer)
                };
                sockets.add(tcp_socket);
            }

            for storage in store.udp_socket_storage[..].iter_mut() {
                let udp_socket = {
                    let rx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                        &mut storage.rx_metadata[..],
                        &mut storage.rx_storage[..],
                    );
                    let tx_buffer = smoltcp::socket::UdpSocketBuffer::new(
                        &mut storage.tx_metadata[..],
                        &mut storage.tx_storage[..],
                    );

                    smoltcp::socket::UdpSocket::new(rx_buffer, tx_buffer)
                };
                sockets.add(udp_socket);
            }

            sockets
        };

        let dhcp_client = {
            let dhcp_rx_buffer = smoltcp::socket::RawSocketBuffer::new(
                &mut store.dhcp_rx_metadata[..],
                &mut store.dhcp_rx_storage[..],
            );

            let dhcp_tx_buffer = smoltcp::socket::RawSocketBuffer::new(
                &mut store.dhcp_tx_metadata[..],
                &mut store.dhcp_tx_storage[..],
            );

            smoltcp::dhcp::Dhcpv4Client::new(
                &mut sockets,
                dhcp_rx_buffer,
                dhcp_tx_buffer,
                // Smoltcp indicates that an instant with a negative time is indicative that time is
                // not yet available. We can't get the current instant yet, so indicate an invalid
                // time value.
                smoltcp::time::Instant::from_millis(-1),
            )
        };

        let random_seed = {
            let mut rng =
                device.RNG.constrain(ccdr.peripheral.RNG, &ccdr.clocks);
            let mut data = [0u8; 4];
            rng.fill(&mut data).unwrap();
            data
        };

        let mut stack = smoltcp_nal::NetworkStack::new(
            interface,
            sockets,
            Some(dhcp_client),
        );

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

    fp_led_0.set_low().unwrap();
    fp_led_1.set_low().unwrap();
    fp_led_2.set_low().unwrap();
    fp_led_3.set_low().unwrap();

    // Measure the Pounder PGOOD output to detect if pounder is present on Stabilizer.
    let pounder_pgood = gpiob.pb13.into_pull_down_input();
    delay.delay_ms(2u8);
    let pounder = if pounder_pgood.is_high().unwrap() {
        log::info!("Found Pounder");
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
                    device.QUADSPI,
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

            let mut ad9959 = ad9959::Ad9959::new(
                qspi_interface,
                reset_pin,
                &mut io_update,
                &mut delay,
                ad9959::Mode::FourBitSerial,
                ref_clk.0 as f32,
                design_parameters::DDS_MULTIPLIER,
            )
            .unwrap();

            ad9959.self_test().unwrap();

            // Return IO_Update
            gpiog.pg7 = io_update.into_analog();

            ad9959
        };

        let io_expander = {
            let sda = gpiob.pb7.into_alternate_af4().set_open_drain();
            let scl = gpiob.pb8.into_alternate_af4().set_open_drain();
            let i2c1 = device.I2C1.i2c(
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
            device.SPI1.spi(
                (spi_sck, spi_miso, spi_mosi),
                config,
                5.mhz(),
                ccdr.peripheral.SPI1,
                &ccdr.clocks,
            )
        };

        let (adc1, adc2) = {
            let (mut adc1, mut adc2) = hal::adc::adc12(
                device.ADC1,
                device.ADC2,
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
                    design_parameters::POUNDER_IO_UPDATE_DURATION,
                    design_parameters::POUNDER_IO_UPDATE_DELAY,
                );

                // Ensure that we have enough time for an IO-update every sample.
                let sample_frequency = {
                    let timer_frequency: hal::time::Hertz =
                        design_parameters::TIMER_FREQUENCY.into();
                    timer_frequency.0 as f32
                        / design_parameters::ADC_SAMPLE_TICKS as f32
                };

                let sample_period = 1.0 / sample_frequency;
                assert!(
                    sample_period > design_parameters::POUNDER_IO_UPDATE_DELAY
                );

                hrtimer
            };

            let (qspi, config) = ad9959.freeze();
            DdsOutput::new(qspi, io_update_trigger, config)
        };

        #[cfg(feature = "pounder_v1_1")]
        let pounder_stamper = {
            log::info!("Assuming Pounder v1.1 or later");
            let etr_pin = gpioa.pa0.into_alternate_af3();

            // The frequency in the constructor is dont-care, as we will modify the period + clock
            // source manually below.
            let tim8 =
                device
                    .TIM8
                    .timer(1.khz(), ccdr.peripheral.TIM8, &ccdr.clocks);
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
                dma2_streams.0,
                tim8_channels.ch1,
                &mut sampling_timer,
                etr_pin,
            )
        };

        Some(PounderDevices {
            pounder: pounder_devices,
            dds_output,

            #[cfg(feature = "pounder_v1_1")]
            timestamper: pounder_stamper,
        })
    } else {
        None
    };

    let cycle_counter = {
        // Set TRCENA bit, which is required for DWT counter to tick. (This is also
        // set automatically when running in the debugger, and only cleared on power
        // reset, not on soft reset.)
        core.DCB.enable_trace();
        CycleCounter::new(core.DWT, ccdr.clocks.c_ck())
    };

    let stabilizer = StabilizerDevices {
        afes,
        adcs,
        dacs,
        timestamper: input_stamper,
        net: network_devices,
        adc_dac_timer: sampling_timer,
        timestamp_timer,
        cycle_counter,
        digital_inputs,
    };

    // Enable the instruction cache.
    core.SCB.enable_icache();

    // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
    // info!("Built on {}", build_info::BUILT_TIME_UTC);
    // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);
    log::info!("setup() complete");

    (stabilizer, pounder)
}
