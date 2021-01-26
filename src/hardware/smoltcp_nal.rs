use core::cell::RefCell;
///! Network abstraction layer for smoltcp.
use heapless::{consts, Vec};
use minimq::embedded_nal::{self as nal, nb};

use super::Ethernet;

pub struct NetStorage {
    pub ip_addrs: [smoltcp::wire::IpCidr; 1],
    pub sockets: [Option<smoltcp::socket::SocketSetItem<'static, 'static>>; 1],
    pub neighbor_cache:
        [Option<(smoltcp::wire::IpAddress, smoltcp::iface::Neighbor)>; 8],
    pub routes_cache:
        [Option<(smoltcp::wire::IpCidr, smoltcp::iface::Route)>; 8],
    pub tx_storage: [u8; 4096],
    pub rx_storage: [u8; 4096],
}

#[derive(Debug)]
pub enum NetworkError {
    NoSocket,
    ConnectionFailure,
    ReadFailure,
    WriteFailure,
    Unsupported,
}

pub struct NetworkStack<'a, 'b, 'c> {
    network_interface: RefCell<Ethernet>,
    sockets: RefCell<smoltcp::socket::SocketSet<'a, 'b, 'c>>,
    next_port: RefCell<u16>,
    unused_handles: RefCell<Vec<smoltcp::socket::SocketHandle, consts::U16>>,
}

impl<'a, 'b, 'c> NetworkStack<'a, 'b, 'c> {
    pub fn new(
        interface: Ethernet,
        sockets: smoltcp::socket::SocketSet<'a, 'b, 'c>,
    ) -> Self {
        let mut unused_handles: Vec<
            smoltcp::socket::SocketHandle,
            consts::U16,
        > = Vec::new();
        for socket in sockets.iter() {
            unused_handles.push(socket.handle()).unwrap();
        }

        NetworkStack {
            network_interface: RefCell::new(interface),
            sockets: RefCell::new(sockets),
            next_port: RefCell::new(49152),
            unused_handles: RefCell::new(unused_handles),
        }
    }

    pub fn update(&self, time: u32) -> bool {
        match self.network_interface.borrow_mut().poll(
            &mut self.sockets.borrow_mut(),
            smoltcp::time::Instant::from_millis(time as i64),
        ) {
            Ok(changed) => changed == false,
            Err(e) => {
                info!("{:?}", e);
                true
            }
        }
    }

    fn get_ephemeral_port(&self) -> u16 {
        // Get the next ephemeral port
        let current_port = self.next_port.borrow().clone();

        let (next, wrap) = self.next_port.borrow().overflowing_add(1);
        *self.next_port.borrow_mut() = if wrap { 49152 } else { next };

        return current_port;
    }
}

impl<'a, 'b, 'c> nal::TcpStack for NetworkStack<'a, 'b, 'c> {
    type Error = NetworkError;
    type TcpSocket = smoltcp::socket::SocketHandle;

    fn open(
        &self,
        _mode: nal::Mode,
    ) -> Result<smoltcp::socket::SocketHandle, NetworkError> {
        match self.unused_handles.borrow_mut().pop() {
            Some(handle) => {
                // Abort any active connections on the handle.
                let mut sockets = self.sockets.borrow_mut();
                let internal_socket: &mut smoltcp::socket::TcpSocket =
                    &mut *sockets.get(handle);
                internal_socket.abort();

                Ok(handle)
            }
            None => Err(NetworkError::NoSocket),
        }
    }

    fn connect(
        &self,
        socket: smoltcp::socket::SocketHandle,
        remote: nal::SocketAddr,
    ) -> Result<smoltcp::socket::SocketHandle, NetworkError> {
        let mut sockets = self.sockets.borrow_mut();
        let internal_socket: &mut smoltcp::socket::TcpSocket =
            &mut *sockets.get(socket);

        // If we're already in the process of connecting, ignore the request silently.
        if internal_socket.is_open() {
            return Ok(socket);
        }

        match remote.ip() {
            nal::IpAddr::V4(addr) => {
                let octets = addr.octets();
                let address = smoltcp::wire::Ipv4Address::new(
                    octets[0], octets[1], octets[2], octets[3],
                );
                internal_socket
                    .connect(
                        (address, remote.port()),
                        self.get_ephemeral_port(),
                    )
                    .map_err(|_| NetworkError::ConnectionFailure)?;
                Ok(socket)
            }

            // We only support IPv4.
            _ => Err(NetworkError::Unsupported),
        }
    }

    fn is_connected(
        &self,
        socket: &smoltcp::socket::SocketHandle,
    ) -> Result<bool, NetworkError> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut smoltcp::socket::TcpSocket =
            &mut *sockets.get(*socket);
        Ok(socket.may_send() && socket.may_recv())
    }

    fn write(
        &self,
        socket: &mut smoltcp::socket::SocketHandle,
        buffer: &[u8],
    ) -> nb::Result<usize, NetworkError> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut smoltcp::socket::TcpSocket =
            &mut *sockets.get(*socket);
        socket
            .send_slice(buffer)
            .map_err(|_| nb::Error::Other(NetworkError::WriteFailure))
    }

    fn read(
        &self,
        socket: &mut smoltcp::socket::SocketHandle,
        buffer: &mut [u8],
    ) -> nb::Result<usize, NetworkError> {
        let mut sockets = self.sockets.borrow_mut();
        let socket: &mut smoltcp::socket::TcpSocket =
            &mut *sockets.get(*socket);
        socket
            .recv_slice(buffer)
            .map_err(|_| nb::Error::Other(NetworkError::ReadFailure))
    }

    fn close(
        &self,
        socket: smoltcp::socket::SocketHandle,
    ) -> Result<(), NetworkError> {
        let mut sockets = self.sockets.borrow_mut();
        let internal_socket: &mut smoltcp::socket::TcpSocket =
            &mut *sockets.get(socket);
        internal_socket.close();

        self.unused_handles.borrow_mut().push(socket).unwrap();
        Ok(())
    }
}
