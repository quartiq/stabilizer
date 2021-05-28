use core::borrow::BorrowMut;
use heapless::{
    spsc::{Consumer, Producer, Queue},
};
use smoltcp_nal::embedded_nal::{SocketAddr, UdpClientStack};

use super::NetworkReference;
use crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE;

// The number of data blocks that we will buffer in the queue.
const BLOCK_BUFFER_SIZE: usize = 30;

const SUBSAMPLE_RATE: usize = 2;

pub fn setup_streaming(
    stack: NetworkReference,
) -> (BlockGenerator, DataStream) {
    let queue = cortex_m::singleton!(: Queue<AdcDacData, BLOCK_BUFFER_SIZE> = Queue::new()).unwrap();

    let (producer, consumer) = queue.split();

    let generator = BlockGenerator::new(producer);

    let stream = DataStream::new(stack, consumer);

    (generator, stream)
}

pub fn serialize_blocks<'a>(buffer: &'a mut [u8], max_buffer_size: usize, queue: &mut Consumer<'static,
        AdcDacData, BLOCK_BUFFER_SIZE>) -> &'a [u8] {
    // While there is space in the buffer, serialize into it.

    let block_size = (SAMPLE_BUFFER_SIZE / SUBSAMPLE_RATE * 2) * 2 * 2 + 8;

    // Truncate the buffer to the maximum buffer size.
    let buffer: &mut [u8] = if buffer.len() > max_buffer_size {
        &mut buffer[..max_buffer_size]
    } else {
        buffer
    };

    // Serialize blocks into the buffer until either the buffer or the queue are exhausted.
    let mut enqueued_blocks: usize = 0;
    for buf in buffer.chunks_exact_mut(block_size) {
        // If there are no more blocks, return the serialized data.
        let data = match queue.dequeue() {
            Some(data) => data,
            None => break,
        };

        let block = DataBlock {
            adcs: data.adcs,
            dacs: data.dacs,
            block_id: data.block_id,
            block_size: SAMPLE_BUFFER_SIZE,
        };

        enqueued_blocks += 1;
        let length = block.to_slice(buf, SUBSAMPLE_RATE);
        assert!(length == block_size);
    }

    &buffer[..block_size * enqueued_blocks]
}

#[derive(Debug)]
pub struct AdcDacData {
    block_id: u32,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

pub struct BlockGenerator {
    queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    current_id: u32,
}

impl BlockGenerator {
    pub fn new(queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>) -> Self {
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

        // Note(unwrap): The buffering of the queue and processing of blocks must be fast enough
        // such that blocks will never be silently dropped.
        self.queue.enqueue(block).unwrap();
    }
}

pub struct DataStream {
    stack: NetworkReference,
    socket: Option<<NetworkReference as UdpClientStack>::UdpSocket>,
    queue: Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    remote: Option<SocketAddr>,
    buffer: [u8; 1024],
}

struct DataBlock {
    block_id: u32,
    block_size: usize,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

impl DataBlock {
    pub fn to_slice(self, buf: &mut [u8], subsample: usize) -> usize {
        let block_size = self.block_size / subsample;
        buf[0..4].copy_from_slice(&self.block_id.to_be_bytes());
        buf[4..8].copy_from_slice(&block_size.to_be_bytes());

        let mut offset: usize = 8;
        for device in &[self.adcs, self.dacs] {
            for channel in device {
                for sample in channel.iter().step_by(subsample) {
                    buf[offset..offset+2].copy_from_slice(&sample.to_be_bytes());
                    offset += 2;
                }
            }
        }

        offset
    }

}

impl DataStream {
    pub fn new(
        stack: NetworkReference,
        consumer: Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            remote: None,
            queue: consumer,
            buffer: [0; 1024],
        }
    }

    fn close(&mut self) {
        // Note(unwrap): We guarantee that the socket is available above.
        let socket = self.socket.take().unwrap();
        self.stack.close(socket).unwrap();

        log::info!("Stream Disconnecting");
    }

    fn open(&mut self, remote: SocketAddr) -> Result<(), ()> {
        if self.socket.is_some() {
            self.close();
        }

        let mut socket =
            self.stack
                .socket()
                .map_err(|err| match err {
                    <NetworkReference as UdpClientStack>::Error::NoIpAddress => (),
                    _ => ()
                })?;

        // TODO: How should we handle a connection failure?
        self.stack.connect(&mut socket, remote).unwrap();

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
        // If there's no socket available, try to connect to our remote.
        if self.socket.is_none() && self.remote.is_some() {
            // If we still can't open the remote, continue.
            if self.open(self.remote.unwrap()).is_err() {

                // Clear the queue out.
                while self.queue.ready() {
                    self.queue.dequeue();
                }
                return;
            }
        }

        if self.queue.ready() {
            let mut handle = self.socket.borrow_mut().unwrap();
            let capacity = self.stack.lock(|stack| stack.get_remaining_send_buffer(handle.handle)).unwrap();

            let data = serialize_blocks(&mut self.buffer, capacity, &mut self.queue);

            // Transmit the data block.
            // TODO: Should we measure how many packets get dropped as telemetry?
            self.stack.send(&mut handle, &data).ok();
        }
    }
}
