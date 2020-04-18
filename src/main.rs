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

use core::ptr;
// use core::sync::atomic::{AtomicU32, AtomicBool, Ordering};
use core::fmt::Write;
use cortex_m_rt::exception;
use heapless::{consts::*, String, Vec};
use rtfm::cyccnt::{Instant, U32Ext as _};
use stm32h7::stm32h743v as pac;

use smoltcp as net;

use serde::{de::DeserializeOwned, Deserialize, Serialize};
use serde_json_core::{de::from_slice, ser::to_string};

mod eth;

mod iir;
use iir::*;

mod board;
mod eeprom;
mod i2c;

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

const SCALE: f32 = ((1 << 15) - 1) as f32;

// static ETHERNET_PENDING: AtomicBool = AtomicBool::new(true);

const TCP_RX_BUFFER_SIZE: usize = 8192;
const TCP_TX_BUFFER_SIZE: usize = 8192;

macro_rules! create_socket {
    ($set:ident, $rx_storage:ident, $tx_storage:ident, $target:ident) => {
        let mut $rx_storage = [0; TCP_RX_BUFFER_SIZE];
        let mut $tx_storage = [0; TCP_TX_BUFFER_SIZE];
        let tcp_rx_buffer =
            net::socket::TcpSocketBuffer::new(&mut $rx_storage[..]);
        let tcp_tx_buffer =
            net::socket::TcpSocketBuffer::new(&mut $tx_storage[..]);
        let tcp_socket =
            net::socket::TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);
        let $target = $set.add(tcp_socket);
    };
}

#[rtfm::app(device = stm32h7::stm32h743v, peripherals = true, monotonic = rtfm::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        spi: (pac::SPI1, pac::SPI2, pac::SPI4, pac::SPI5),
        i2c: pac::I2C2,
        ethernet_periph:
            (pac::ETHERNET_MAC, pac::ETHERNET_DMA, pac::ETHERNET_MTL),
        #[init([[0.; 5]; 2])]
        iir_state: [IIRState; 2],
        #[init([IIR { ba: [1., 0., 0., 0., 0.], y_offset: 0., y_min: -SCALE - 1., y_max: SCALE }; 2])]
        iir_ch: [IIR; 2],
        #[link_section = ".sram3.eth"]
        #[init(eth::Device::new())]
        ethernet: eth::Device,
    }

    #[init(schedule = [tick])]
    fn init(c: init::Context) -> init::LateResources {
        board::init();

        init_log();
        // info!("Version {} {}", build_info::PKG_VERSION, build_info::GIT_VERSION.unwrap());
        // info!("Built on {}", build_info::BUILT_TIME_UTC);
        // info!("{} {}", build_info::RUSTC_VERSION, build_info::TARGET);

        // c.schedule.tick(Instant::now()).unwrap();

        let dp = c.device;
        init::LateResources {
            spi: (dp.SPI1, dp.SPI2, dp.SPI4, dp.SPI5),
            i2c: dp.I2C2,
            ethernet_periph: (
                dp.ETHERNET_MAC,
                dp.ETHERNET_DMA,
                dp.ETHERNET_MTL,
            ),
        }
    }

    #[idle(resources = [ethernet, ethernet_periph, iir_state, iir_ch, i2c])]
    fn idle(c: idle::Context) -> ! {
        let (MAC, DMA, MTL) = c.resources.ethernet_periph;

        let hardware_addr = match eeprom::read_eui48(c.resources.i2c) {
            Err(_) => {
                info!("Could not read EEPROM, using default MAC address");
                net::wire::EthernetAddress([0x10, 0xE2, 0xD5, 0x00, 0x03, 0x00])
            }
            Ok(raw_mac) => net::wire::EthernetAddress(raw_mac),
        };
        info!("MAC: {}", hardware_addr);

        unsafe { c.resources.ethernet.init(hardware_addr, MAC, DMA, MTL) };
        let mut neighbor_cache_storage = [None; 8];
        let neighbor_cache =
            net::iface::NeighborCache::new(&mut neighbor_cache_storage[..]);
        let local_addr = net::wire::IpAddress::v4(10, 0, 16, 99);
        let mut ip_addrs = [net::wire::IpCidr::new(local_addr, 24)];
        let mut iface =
            net::iface::EthernetInterfaceBuilder::new(c.resources.ethernet)
                .ethernet_addr(hardware_addr)
                .neighbor_cache(neighbor_cache)
                .ip_addrs(&mut ip_addrs[..])
                .finalize();
        let mut socket_set_entries: [_; 8] = Default::default();
        let mut sockets =
            net::socket::SocketSet::new(&mut socket_set_entries[..]);
        create_socket!(sockets, tcp_rx_storage0, tcp_tx_storage0, tcp_handle0);
        create_socket!(sockets, tcp_rx_storage0, tcp_tx_storage0, tcp_handle1);

        // unsafe { eth::enable_interrupt(DMA); }
        let mut time = 0u32;
        let mut next_ms = Instant::now();
        next_ms += 400_000.cycles();
        let mut server = Server::new();
        let mut iir_state: resources::iir_state = c.resources.iir_state;
        let mut iir_ch: resources::iir_ch = c.resources.iir_ch;
        loop {
            // if ETHERNET_PENDING.swap(false, Ordering::Relaxed) { }
            let tick = Instant::now() > next_ms;
            if tick {
                next_ms += 400_000.cycles();
                time += 1;
            }
            {
                let socket =
                    &mut *sockets.get::<net::socket::TcpSocket>(tcp_handle0);
                if socket.state() == net::socket::TcpState::CloseWait {
                    socket.close();
                } else if !(socket.is_open() || socket.is_listening()) {
                    socket
                        .listen(1234)
                        .unwrap_or_else(|e| warn!("TCP listen error: {:?}", e));
                } else if tick && socket.can_send() {
                    let s = iir_state.lock(|iir_state| Status {
                        t: time,
                        x0: iir_state[0][0],
                        y0: iir_state[0][2],
                        x1: iir_state[1][0],
                        y1: iir_state[1][2],
                    });
                    json_reply(socket, &s);
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
                    server.poll(socket, |req: &Request| {
                        if req.channel < 2 {
                            iir_ch.lock(|iir_ch| {
                                iir_ch[req.channel as usize] = req.iir
                            });
                        }
                    });
                }
            }

            if !match iface.poll(
                &mut sockets,
                net::time::Instant::from_millis(time as i64),
            ) {
                Ok(changed) => changed,
                Err(net::Error::Unrecognized) => true,
                Err(e) => {
                    info!("iface poll error: {:?}", e);
                    true
                }
            } {
                // cortex_m::asm::wfi();
            }
        }
    }

    #[task(priority = 1, schedule = [tick])]
    fn tick(c: tick::Context) {
        static mut TIME: u32 = 0;
        *TIME += 1;
        const PERIOD: u32 = 200_000_000;
        c.schedule.tick(c.scheduled + PERIOD.cycles()).unwrap();
    }

    // seems to slow it down
    // #[link_section = ".data.spi1"]
    #[task(binds = SPI1, resources = [spi, iir_state, iir_ch], priority = 2)]
    fn spi1(c: spi1::Context) {
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();
        let (spi1, spi2, spi4, spi5) = c.resources.spi;
        let iir_ch = c.resources.iir_ch;
        let iir_state = c.resources.iir_state;

        let sr = spi1.sr.read();
        if sr.eot().bit_is_set() {
            spi1.ifcr.write(|w| w.eotc().set_bit());
        }
        if sr.rxp().bit_is_set() {
            let rxdr = &spi1.rxdr as *const _ as *const u16;
            let a = unsafe { ptr::read_volatile(rxdr) };
            let x0 = f32::from(a as i16);
            let y0 = iir_ch[0].update(&mut iir_state[0], x0);
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
            let y0 = iir_ch[1].update(&mut iir_state[1], x0);
            let d = y0 as i16 as u16 ^ 0x8000;
            let txdr = &spi4.txdr as *const _ as *mut u16;
            unsafe { ptr::write_volatile(txdr, d) };
        }
        #[cfg(feature = "bkpt")]
        cortex_m::asm::bkpt();
    }

    /*
    #[task(binds = ETH, resources = [ethernet_periph], priority = 1)]
    fn eth(c: eth::Context) {
        let dma = &c.resources.ethernet_periph.1;
        ETHERNET_PENDING.store(true, Ordering::Relaxed);
        unsafe { eth::interrupt_handler(dma) }
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

#[derive(Deserialize, Serialize)]
struct Request {
    channel: u8,
    iir: IIR,
}

#[derive(Serialize)]
struct Response<'a> {
    code: i32,
    message: &'a str,
}

#[derive(Serialize)]
struct Status {
    t: u32,
    x0: f32,
    y0: f32,
    x1: f32,
    y1: f32,
}

fn json_reply<T: Serialize>(socket: &mut net::socket::TcpSocket, msg: &T) {
    let mut u: String<U128> = to_string(msg).unwrap();
    u.push('\n').unwrap();
    socket.write_str(&u).unwrap();
}

struct Server {
    data: Vec<u8, U256>,
    discard: bool,
}

impl Server {
    fn new() -> Self {
        Self {
            data: Vec::new(),
            discard: false,
        }
    }

    fn poll<T, F, R>(
        &mut self,
        socket: &mut net::socket::TcpSocket,
        f: F,
    ) -> Option<R>
    where
        T: DeserializeOwned,
        F: FnOnce(&T) -> R,
    {
        while socket.can_recv() {
            let found = socket
                .recv(|buf| {
                    let (len, found) =
                        match buf.iter().position(|&c| c as char == '\n') {
                            Some(end) => (end + 1, true),
                            None => (buf.len(), false),
                        };
                    if self.data.len() + len >= self.data.capacity() {
                        self.discard = true;
                        self.data.clear();
                    } else if !self.discard && len > 0 {
                        self.data.extend_from_slice(&buf[..len]).unwrap();
                    }
                    (len, found)
                })
                .unwrap();
            if found {
                if self.discard {
                    self.discard = false;
                    json_reply(
                        socket,
                        &Response {
                            code: 520,
                            message: "command buffer overflow",
                        },
                    );
                    self.data.clear();
                } else {
                    let r = from_slice::<T>(&self.data[..self.data.len() - 1]);
                    self.data.clear();
                    match r {
                        Ok(res) => {
                            let r = f(&res);
                            json_reply(
                                socket,
                                &Response {
                                    code: 200,
                                    message: "ok",
                                },
                            );
                            return Some(r);
                        }
                        Err(err) => {
                            warn!("parse error {:?}", err);
                            json_reply(
                                socket,
                                &Response {
                                    code: 550,
                                    message: "parse error",
                                },
                            );
                        }
                    }
                }
            }
        }
        None
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
