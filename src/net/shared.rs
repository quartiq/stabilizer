use shared_bus::{AtomicCheckMutex, BusMutex};
use minimq::embedded_nal;
use smoltcp_nal::smoltcp;

use crate::hardware::NetworkStack;

pub struct NetworkStackProxy<'a, S> {
    mutex: &'a AtomicCheckMutex<S>
}

impl<'a> NetworkStackProxy<'a, NetworkStack> {
    pub fn poll(&mut self, now: u32) -> Result<bool, smoltcp::Error> {
        self.mutex.lock(|stack| stack.poll(now))
    }
    pub fn handle_link_reset(&mut self) {
        self.mutex.lock(|stack| stack.handle_link_reset())
    }
}

macro_rules! forward {
    ($func:ident($($v:ident: $IT:ty),*) -> $T:ty) => {
        fn $func(&self, $($v: $IT),*) -> $T {
            self.mutex.lock(|stack| stack.$func($($v),*))
        }
    }
}

impl<'a, S> embedded_nal::TcpStack for NetworkStackProxy<'a, S>
where
    S: embedded_nal::TcpStack
{
    type TcpSocket = S::TcpSocket;
    type Error = S::Error;

    forward! {open(mode: embedded_nal::Mode) -> Result<S::TcpSocket, S::Error>}
    forward! {connect(socket: S::TcpSocket, remote: embedded_nal::SocketAddr) -> Result<S::TcpSocket, S::Error>}
    forward! {is_connected(socket: &S::TcpSocket) -> Result<bool, S::Error>}
    forward! {write(socket: &mut S::TcpSocket, buffer: &[u8]) -> embedded_nal::nb::Result<usize, S::Error>}
    forward! {read(socket: &mut S::TcpSocket, buffer: &mut [u8]) -> embedded_nal::nb::Result<usize, S::Error>}
    forward! {close(socket: S::TcpSocket) -> Result<(), S::Error>}
}

pub struct NetworkManager {
    mutex: AtomicCheckMutex<NetworkStack>
}

impl NetworkManager {
    pub fn new(stack: NetworkStack) -> Self {
        Self { mutex: AtomicCheckMutex::create(stack) }
    }

    pub fn acquire_stack<'a>(&'a self) -> NetworkStackProxy<'a, NetworkStack> {
        NetworkStackProxy {
            mutex: &self.mutex
        }
    }
}
