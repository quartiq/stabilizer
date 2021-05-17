use core::borrow::BorrowMut;
use heapless::{
    spsc::{Consumer, Producer, Queue},
    Vec,
};
use serde::Serialize;
use smoltcp_nal::embedded_nal::{Mode, SocketAddr, TcpStack};

use super::NetworkReference;
use crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE;

// The number of data blocks that we will buffer in the queue.
type BlockBufferSize = heapless::consts::U10;

pub fn setup_streaming(
    stack: NetworkReference,
) -> (BlockGenerator, DataStream) {
    let queue = cortex_m::singleton!(: Queue<AdcDacData, BlockBufferSize> = Queue::new()).unwrap();

    let (producer, consumer) = queue.split();

    let generator = BlockGenerator::new(producer);

    let stream = DataStream::new(stack, consumer);

    (generator, stream)
}

pub struct AdcDacData {
    block_id: u32,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

pub struct BlockGenerator {
    queue: Producer<'static, AdcDacData, BlockBufferSize>,
    current_id: u32,
}

impl BlockGenerator {
    pub fn new(queue: Producer<'static, AdcDacData, BlockBufferSize>) -> Self {
        Self {
            queue,
            current_id: 0,
        }
    }

    pub fn send(
        &mut self,
        adcs: &[&[u16; SAMPLE_BUFFER_SIZE]; 2],
        dacs: &[&mut [u16; SAMPLE_BUFFER_SIZE]; 2],
    ) {
        let block = AdcDacData {
            block_id: self.current_id,
            adcs: [*adcs[0], *adcs[1]],
            dacs: [*dacs[0], *dacs[1]],
        };

        self.current_id = self.current_id.wrapping_add(1);

        // We perform best-effort enqueueing of the data block.
        self.queue.enqueue(block).ok();
    }
}

pub struct DataStream {
    stack: NetworkReference,
    socket: Option<<NetworkReference as TcpStack>::TcpSocket>,
    queue: Consumer<'static, AdcDacData, BlockBufferSize>,
    current_index: u32,
    remote: Option<SocketAddr>,
}

#[derive(Serialize)]
struct DataBlock {
    block_id: u32,
    block_size: usize,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

impl DataStream {
    pub fn new(
        stack: NetworkReference,
        consumer: Consumer<'static, AdcDacData, BlockBufferSize>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            current_index: 0,
            remote: None,
            queue: consumer,
        }
    }

    fn open(&mut self, remote: SocketAddr) -> Result<(), ()> {
        if self.socket.is_some() {
            // Note(unwrap): We guarantee that the socket is available above.
            let socket = self.socket.take().unwrap();
            self.stack.close(socket).unwrap();
        }

        let socket =
            self.stack
                .open(Mode::NonBlocking)
                .map_err(|err| match err {
                    <NetworkReference as TcpStack>::Error::NoIpAddress => (),
                    other => {
                        log::info!("Network Error: {:?}", other);
                        ()
                    }
                })?;

        // TODO: How should we handle a connection failure?
        let socket = self.stack.connect(socket, remote).unwrap();

        // Note(unwrap): The socket will be empty before we replace it.
        self.socket.replace(socket);

        Ok(())
    }

    pub fn set_remote(&mut self, remote: SocketAddr) {
        // If the remote is identical to what we already have, do nothing.
        if let Some(current_remote) = self.remote {
            if current_remote == remote {
                return;
            }
        }

        // Open the new remote connection.
        self.open(remote).ok();
        self.remote = Some(remote);
    }

    pub fn process(&mut self) {
        while let Some(data) = self.queue.dequeue() {
            // If there's no socket available, try to connect to our remote.
            if self.socket.is_none() && self.remote.is_some() {
                // If we still can't open the remote, continue.
                if self.open(self.remote.unwrap()).is_err() {
                    continue;
                }
            }

            let block = DataBlock {
                adcs: data.adcs,
                dacs: data.dacs,
                block_id: data.block_id,
                block_size: SAMPLE_BUFFER_SIZE,
            };

            // Increment the current block index.
            self.current_index = self.current_index.wrapping_add(1);

            // Serialize the datablock.
            // TODO: Do we want to packetize the data block as well?
            let data: Vec<u8, heapless::consts::U256> =
                postcard::to_vec(&block).unwrap();

            let mut socket = self.socket.borrow_mut().unwrap();

            // Transmit the data block.
            // TODO: How should we handle partial packet transmission?
            match self.stack.write(&mut socket, &data) {
                Ok(len) => assert!(len == data.len()),
                _ => info!("Dropping packet"),
            }
        }
    }
}
