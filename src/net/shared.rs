///! Network Stack Sharing Utilities
///!
///! # Design
///! This module provides a mechanism for sharing a single network stack safely between drivers
///that may or may not execute in multiple contexts. The design copies that of `shared-bus`.
///!
///! Specifically, the network stack is stored in a global static singleton and proxies to the
///! underlying stack are handed out. The proxies provide an identical API for the
///! `embedded_nal::TcpStack` stack trait, so they can be provided direclty to drivers that require
///! a network stack.
///!
///! In order to ensure that pre-emption does not occur while accessing the same network stack from
///! multiple interrupt contexts, the proxy uses an atomic boolean check - if the flag indicates the
///! stack is in use, the proxy will generate a panic. The actual synchronization mechanism (mutex)
///! leverages RTIC resource allocation. All devices that use the underlying network stack must be
///! placed in a single RTIC resource, which will cause RTIC to prevent contention for the
///! underlying network stack.
use minimq::embedded_nal;
use shared_bus::{AtomicCheckMutex, BusMutex};

use crate::hardware::NetworkStack;

/// A manager for a shared network stack.
pub struct NetworkManager {
    mutex: AtomicCheckMutex<NetworkStack>,
}

/// A basic proxy that references a shared network stack.
pub struct NetworkStackProxy<'a, S> {
    mutex: &'a AtomicCheckMutex<S>,
}

impl<'a, S> NetworkStackProxy<'a, S> {
    /// Using the proxy, access the underlying network stack directly.
    ///
    /// # Args
    /// * `f` - A closure which will be provided the network stack as an argument.
    ///
    /// # Returns
    /// Any value returned by the provided closure
    pub fn lock<R, F: FnOnce(&mut S) -> R>(&mut self, f: F) -> R {
        self.mutex.lock(|stack| f(stack))
    }
}

// A simple forwarding macro taken from the `embedded-nal` to forward the embedded-nal API into the
// proxy structure.
macro_rules! forward {
    ($func:ident($($v:ident: $IT:ty),*) -> $T:ty) => {
        fn $func(&self, $($v: $IT),*) -> $T {
            self.mutex.lock(|stack| stack.$func($($v),*))
        }
    }
}

// Implement a TCP stack for the proxy if the underlying network stack implements it.
impl<'a, S> embedded_nal::TcpStack for NetworkStackProxy<'a, S>
where
    S: embedded_nal::TcpStack,
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

impl NetworkManager {
    /// Construct a new manager for a shared network stack
    ///
    /// # Args
    /// * `stack` - The network stack that is being shared.
    pub fn new(stack: NetworkStack) -> Self {
        Self {
            mutex: AtomicCheckMutex::create(stack),
        }
    }

    /// Acquire a proxy to the shared network stack.
    ///
    /// # Returns
    /// A proxy that can be used in place of the network stack. Note the requirements of
    /// concurrency listed in the description of this file for usage.
    pub fn acquire_stack<'a>(&'a self) -> NetworkStackProxy<'a, NetworkStack> {
        NetworkStackProxy { mutex: &self.mutex }
    }
}
