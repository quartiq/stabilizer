///! Stabilizer data stream capabilities
///!
///! # Design
///! Stabilizer data streamining utilizes UDP packets to send live data streams at high throughput.
///! Packets are always sent in a best-effort fashion, and data may be dropped. Each packet contains
///! an identifier that can be used to detect any dropped data.
///!
///! The current implementation utilizes an single-producer, single-consumer queue to send data
///! between a high priority task and the UDP transmitter.
///!
///! A "batch" of data is defined to be a single item in the SPSC queue sent to the UDP transmitter
///! thread. The transmitter thread then serializes as many sequential "batches" into a single UDP
///! packet as possible. The UDP packet is also given a header indicating the starting batch
///! sequence number and the number of batches present. If the UDP transmitter encounters a
///! non-sequential batch, it does not enqueue it into the packet and instead transmits any staged
///! data. The non-sequential batch is then transmitted in a new UDP packet. This method allows a
///! receiver to detect dropped batches (e.g. due to processing overhead).
use core::borrow::BorrowMut;
use heapless::spsc::{Consumer, Producer, Queue};
use miniconf::MiniconfAtomic;
use serde::Deserialize;
use smoltcp_nal::embedded_nal::{IpAddr, Ipv4Addr, SocketAddr, UdpClientStack};

use super::NetworkReference;
use crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE;

// The number of data blocks that we will buffer in the queue.
const BLOCK_BUFFER_SIZE: usize = 30;

// A factor that data may be subsampled at.
const SUBSAMPLE_RATE: usize = 1;

/// Represents the destination for the UDP stream to send data to.
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

impl From<StreamTarget> for SocketAddr {
    fn from(target: StreamTarget) -> SocketAddr {
        SocketAddr::new(
            IpAddr::V4(Ipv4Addr::new(
                target.ip[0],
                target.ip[1],
                target.ip[2],
                target.ip[3],
            )),
            target.port,
        )
    }
}

/// A basic "batch" of data.
// Note: In the future, the stream may be generic over this type.
#[derive(Debug, Copy, Clone)]
pub struct AdcDacData {
    block_id: u16,
    adcs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
    dacs: [[u16; SAMPLE_BUFFER_SIZE]; 2],
}

/// Configure streaming on a device.
///
/// # Args
/// * `stack` - A reference to the shared network stack.
///
/// # Returns
/// (generator, stream) where `generator` can be used to enqueue "batches" for transmission. The
/// `stream` is the logically consumer (UDP transmitter) of the enqueued data.
pub fn setup_streaming(
    stack: NetworkReference,
) -> (BlockGenerator, DataStream) {
    let queue = cortex_m::singleton!(: Queue<AdcDacData, BLOCK_BUFFER_SIZE> = Queue::new()).unwrap();

    let (producer, consumer) = queue.split();

    let generator = BlockGenerator::new(producer);

    let stream = DataStream::new(stack, consumer);

    (generator, stream)
}

/// The data generator for a stream.
pub struct BlockGenerator {
    queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    current_id: u16,
}

impl BlockGenerator {
    /// Construct a new generator.
    /// # Args
    /// * `queue` - The producer portion of the SPSC queue to enqueue data into.
    ///
    /// # Returns
    /// The generator to use.
    fn new(queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>) -> Self {
        Self {
            queue,
            current_id: 0,
        }
    }

    /// Schedule data to be sent by the generator.
    ///
    /// # Note
    /// If no space is available, the data batch may be silently dropped.
    ///
    /// # Args
    /// * `adcs` - The ADC data to transmit.
    /// * `dacs` - The DAC data to transmit.
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

/// Represents a single UDP packet sent by the stream.
///
/// # Packet Format
/// All data is sent in network-endian format. The format is as follows
///
/// Header:
/// [0..2]: Start block ID (u16)
/// [2..3]: Num Blocks present (u8) <N>
/// [3..4]: Batch Size (u8) <BS>
///
/// Following the header, batches are added sequentially. Each batch takes the form of:
/// [<BS>*0..<BS>*2]: ADC0
/// [<BS>*2..<BS>*4]: ADC1
/// [<BS>*4..<BS>*6]: DAC0
/// [<BS>*6..<BS>*8]: DAC1
struct DataPacket<'a> {
    buf: &'a mut [u8],
    subsample_rate: usize,
    start_id: Option<u16>,
    num_blocks: u8,
    write_index: usize,
}

impl<'a> DataPacket<'a> {
    /// Construct a new packet.
    ///
    /// # Args
    /// * `buf` - The location to serialize the data packet into.
    /// * `subsample_rate` - The factor at which to subsample data from batches.
    pub fn new(buf: &'a mut [u8], subsample_rate: usize) -> Self {
        Self {
            buf,
            start_id: None,
            num_blocks: 0,
            subsample_rate,
            write_index: 4,
        }
    }

    /// Add a batch of data to the packet.
    ///
    /// # Note
    /// Serialization occurs as the packet is added.
    ///
    /// # Args
    /// * `batch` - The batch to add to the packet.
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

        self.num_blocks += 1;

        Ok(())
    }

    fn get_packet_size(&self) -> usize {
        let header_length = 4;
        let block_sample_size = SAMPLE_BUFFER_SIZE / self.subsample_rate;
        let block_size_bytes = block_sample_size * 2 * 4;

        block_size_bytes * self.num_blocks as usize + header_length
    }

    /// Complete the packet and prepare it for transmission.
    ///
    /// # Returns
    /// The size of the packet. The user should utilize the original buffer provided for packet
    /// construction to access the packet.
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

/// The "consumer" portion of the data stream.
///
/// # Note
/// This is responsible for consuming data and sending it over UDP.
pub struct DataStream {
    stack: NetworkReference,
    socket: Option<<NetworkReference as UdpClientStack>::UdpSocket>,
    queue: Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    remote: SocketAddr,
    buffer: [u8; 1024],
}

impl DataStream {
    /// Construct a new data streamer.
    ///
    /// # Args
    /// * `stack` - A reference to the shared network stack.
    /// * `consumer` - The read side of the queue containing data to transmit.
    fn new(
        stack: NetworkReference,
        consumer: Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            remote: StreamTarget::default().into(),
            queue: consumer,
            buffer: [0; 1024],
        }
    }

    fn close(&mut self) {
        // Note(unwrap): We guarantee that the socket is available above.
        let socket = self.socket.take().unwrap();
        self.stack.close(socket).unwrap();
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

        let mut socket = self.stack.socket().map_err(|_| ())?;

        // Note(unwrap): We only connect with a new socket, so it is guaranteed to not already be
        // bound.
        self.stack.connect(&mut socket, remote).unwrap();

        self.socket.replace(socket);

        Ok(())
    }

    /// Configure the remote endpoint of the stream.
    ///
    /// # Args
    /// * `remote` - The destination to send stream data to.
    pub fn set_remote(&mut self, remote: SocketAddr) {
        // If the remote is identical to what we already have, do nothing.
        if remote == self.remote {
            return;
        }

        // Open the new remote connection.
        self.open(remote).ok();
        self.remote = remote;
    }

    /// Process any data for transmission.
    pub fn process(&mut self) {
        // If there's no socket available, try to connect to our remote.
        if self.socket.is_none() {
            // If we can't open the socket (e.g. we do not have an IP address yet), clear data from
            // the queue.
            if self.open(self.remote).is_err() {
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
