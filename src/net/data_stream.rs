use core::borrow::BorrowMut;
use heapless::spsc::{Consumer, Producer, Queue};
use miniconf::MiniconfAtomic;
use serde::Deserialize;
use smoltcp_nal::embedded_nal::{IpAddr, Ipv4Addr, SocketAddr, UdpClientStack};

use super::NetworkReference;
use crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE;

// The number of data blocks that we will buffer in the queue.
const BLOCK_BUFFER_SIZE: usize = 30;

const SUBSAMPLE_RATE: usize = 1;

#[derive(Copy, Clone, Debug, MiniconfAtomic, Deserialize)]
pub struct StreamTarget {
    pub ip: [u8; 4],
    pub port: u16,
}

impl Default for StreamTarget {
    fn default() -> Self {
        Self {
            ip: [0; 4],
            port: 0,
        }
    }
}

impl Into<SocketAddr> for StreamTarget {
    fn into(self) -> SocketAddr {
        SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(
                self.ip[0], self.ip[1], self.ip[2], self.ip[3],
            )),
            self.port,
        )
    }
}

pub fn setup_streaming(
    stack: NetworkReference,
) -> (BlockGenerator, DataStream) {
    let queue = cortex_m::singleton!(: Queue<AdcDacData, BLOCK_BUFFER_SIZE> = Queue::new()).unwrap();

    let (producer, consumer) = queue.split();

    let generator = BlockGenerator::new(producer);

    let stream = DataStream::new(stack, consumer);

    (generator, stream)
}

#[derive(Debug, Copy, Clone)]
pub struct AdcDacData {
    block_id: u16,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

pub struct BlockGenerator {
    queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    current_id: u16,
}

impl BlockGenerator {
    pub fn new(
        queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    ) -> Self {
        Self {
            queue,
            current_id: 0,
        }
    }

    pub fn send(
        &mut self,
        adcs: &[&mut [u16; SAMPLE_BUFFER_SIZE]; 2],
        dacs: &[&mut [u16; SAMPLE_BUFFER_SIZE]; 2],
    ) {
        let block = AdcDacData {
            block_id: self.current_id,
            adcs: [*adcs[0], *adcs[1]],
            dacs: [*dacs[0], *dacs[1]],
        };

        self.current_id = self.current_id.wrapping_add(1);
        self.queue.enqueue(block).ok();
    }
}

pub struct DataStream {
    stack: NetworkReference,
    socket: Option<<NetworkReference as UdpClientStack>::UdpSocket>,
    queue: Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    remote: Option<SocketAddr>,
    buffer: [u8; 1024],
}

// Datapacket format:
//
// Header:
// [0..2]: Start block ID (u16)
// [2..3]: Num Blocks present (u8) <N>
// [3..4]: Batch Size (u8) <BS>
//
// Following the header, batches are added sequentially. Each batch takes the form of:
// [<BS>*0..<BS>*2]: ADC0
// [<BS>*2..<BS>*4]: ADC1
// [<BS>*4..<BS>*6]: DAC0
// [<BS>*6..<BS>*8]: DAC1
struct DataPacket<'a> {
    buf: &'a mut [u8],
    subsample_rate: usize,
    start_id: Option<u16>,
    num_blocks: u8,
    write_index: usize,
}

impl<'a> DataPacket<'a> {
    pub fn new(buf: &'a mut [u8], subsample_rate: usize) -> Self {
        Self {
            buf,
            start_id: None,
            num_blocks: 0,
            subsample_rate,
            write_index: 4,
        }
    }

    pub fn add_batch(&mut self, batch: &AdcDacData) -> Result<(), ()> {
        // Check that the block is sequential.
        if let Some(id) = &self.start_id {
            if batch.block_id != id.wrapping_add(self.num_blocks.into()) {
                return Err(());
            }
        } else {
            // Otherwise, this is the first block. Record the strt ID.
            self.start_id = Some(batch.block_id);
        }

        // Check that there is space for the block.
        let block_size_bytes = SAMPLE_BUFFER_SIZE / self.subsample_rate * 4 * 2;
        if self.buf.len() - self.get_packet_size() < block_size_bytes {
            return Err(());
        }

        // Copy the samples into the buffer.
        for device in &[batch.adcs, batch.dacs] {
            for channel in device {
                for sample in channel.iter().step_by(self.subsample_rate) {
                    self.buf[self.write_index..self.write_index + 2]
                        .copy_from_slice(&sample.to_be_bytes());
                    self.write_index += 2;
                }
            }
        }

        Ok(())
    }

    fn get_packet_size(&self) -> usize {
        let header_length = 4;
        let block_sample_size = SAMPLE_BUFFER_SIZE / self.subsample_rate;
        let block_size_bytes = block_sample_size * 2 * 4;

        block_size_bytes * self.num_blocks as usize + header_length
    }

    pub fn finish(self) -> usize {
        let block_sample_size = SAMPLE_BUFFER_SIZE / self.subsample_rate;

        // Write the header into the block.
        self.buf[0..2].copy_from_slice(&self.start_id.unwrap().to_be_bytes());
        self.buf[2] = self.num_blocks;
        self.buf[3] = block_sample_size as u8;

        // Return the length of the packet to transmit.
        self.get_packet_size()
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

        // If the remote address is unspecified, just close the existing socket.
        if remote.ip().is_unspecified() {
            if self.socket.is_some() {
                self.close();
            }

            return Err(());
        }

        let mut socket = self.stack.socket().map_err(|err| match err {
            <NetworkReference as UdpClientStack>::Error::NoIpAddress => (),
            _ => (),
        })?;

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
            // Dequeue data from the queue into a larger block structure.
            let mut packet = DataPacket::new(&mut self.buffer, SUBSAMPLE_RATE);
            while self.queue.ready() {
                // Note(unwrap): We check above that the queue is ready before calling this.
                if packet.add_batch(self.queue.peek().unwrap()).is_err() {
                    // If we cannot add another batch, break out of the loop and send the packet.
                    break;
                }

                // Remove the batch that we just added.
                self.queue.dequeue();
            }

            // Transmit the data block.
            let mut handle = self.socket.borrow_mut().unwrap();
            let size = packet.finish();
            self.stack.send(&mut handle, &self.buffer[..size]).ok();
        }
    }
}
