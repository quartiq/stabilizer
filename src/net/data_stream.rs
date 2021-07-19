//! Stabilizer data stream capabilities
//!
//! # Design
//! Data streamining utilizes UDP packets to send live data streams at high throughput.
//! Packets are always sent in a best-effort fashion, and data may be dropped. Each packet contains
//! an identifier that can be used to detect dropped data.
//!
//! Refer to [DataPacket] for information about the serialization format of each UDP packet.
//!
//! # Example
//! A sample Python script is available in `scripts/stream_throughput.py` to demonstrate reception
//! of livestreamed data.
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
///
/// # Miniconf
/// `{"ip": <addr>, "port": <port>}`
///
/// * `<addr>` is an array of 4 bytes. E.g. `[192, 168, 0, 1]`
/// * `<port>` is any unsigned 16-bit value.
///
/// ## Example
/// `{"ip": [192, 168,0, 1], "port": 1111}`
#[derive(Copy, Clone, Debug, MiniconfAtomic, Deserialize, Default)]
pub struct StreamTarget {
    pub ip: [u8; 4],
    pub port: u16,
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

/// # Stream Packet
/// Represents a single UDP packet sent by the stream.
///
/// A "batch" of data is defined to be the data collected for a single invocation of the DSP
/// routine. A packet is composed of as many sequential batches as can fit.
///
/// The packet is given a header indicating the starting batch sequence number and the number of
/// batches present. If the UDP transmitter encounters a non-sequential batch, it does not enqueue
/// it into the packet and instead transmits any staged data. The non-sequential batch is then
/// transmitted in a new UDP packet. This method allows a receiver to detect dropped batches (e.g.
/// due to processing overhead).
///
/// ## Data Format
///
/// Data sent via UDP is sent in "blocks". Each block is a single batch of ADC/DAC codes from an
/// individual DSP processing routine. Each block is assigned a unique 16-bit identifier. The identifier
/// increments by one for each block and rolls over. All blocks in a single packet are guaranteed to
/// contain sequential identifiers.
///
/// All data is transmitted in network-endian (big-endian) format.
///
/// ### Quick Reference
///
/// In the reference below, any values enclosed in parentheses represents the number of bytes used for
/// that value. E.g. "Batch size (1)" indicates 1 byte is used to represent the batch size.
/// ```
/// # UDP packets take the following form
/// <Header>,<Batch 1>,[<Batch 2>, ...<Batch N>]
///
/// # The header takes the following form
/// <Header> = <Starting ID (2)>,<Number blocks [N] (1)>,<Batch size [BS] (1)>
///
/// # Each batch takes the following form
/// <Batch N> = <ADC0>,<ADC1>,<DAC0>,<DAC1>
///
/// # Where
/// <ADCx/DACx> = <Sample 1 (2)>, ...<Sample BS (2)>
/// ```
///
/// ### Packet Format
/// Multiple blocks are sent in a single UDP packet simultaneously. Each UDP packet transmitted
/// contains a header followed by the serialized data blocks.
/// ```
/// <Header>,<Batch 1>,[<Batch 2>, ...<Batch N>]
/// ```
///
/// ### Header
/// A header takes the following form:
/// * The starting block ID (2 bytes)
/// * The number of blocks present in the packet (1 byte)
/// * The size of each bach in samples (1 byte)
///
/// ```
/// <Starting ID (2)>,<N blocks (1)>,<Batch size (1)>
/// ```
///
/// ### Data Blocks
/// Following the header, each block is sequentially serialized. Each block takes the following form:
/// ```
/// <ADC0 samples>,<ADC1 samples>,<DAC0 samples>,<DAC1 samples>
/// ```
///
/// Where `<XXX samples>` is an array of N 16-bit ADC/DAC samples. The number of samples is provided in the
/// header.
///
/// ADC and DAC codes are transmitted in raw machine-code format. Please refer to the datasheet for the
/// ADC and DAC if you need to convert these to voltages.
pub struct DataPacket<'a> {
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
        if let Some(socket) = self.socket.take() {
            log::info!("Closing stream");
            // Note(unwrap): We guarantee that the socket is available above.
            self.stack.close(socket).unwrap();
        }
    }

    // Open new socket.
    fn open(&mut self) -> Result<(), ()> {
        // If there is already a socket of if remote address is unspecified,
        // do not open a new socket.
        if self.socket.is_some() || self.remote.ip().is_unspecified() {
            return Err(());
        }

        log::info!("Opening stream");

        let mut socket = self.stack.socket().or(Err(()))?;

        // Note(unwrap): We only connect with a new socket, so it is guaranteed to not already be
        // bound.
        self.stack.connect(&mut socket, self.remote).unwrap();

        self.socket.replace(socket);

        Ok(())
    }

    /// Configure the remote endpoint of the stream.
    ///
    /// # Args
    /// * `remote` - The destination to send stream data to.
    pub fn set_remote(&mut self, remote: SocketAddr) {
        // Close socket to be reopened if the remote has changed.
        if remote != self.remote {
            self.close();
        }
        self.remote = remote;
    }

    /// Process any data for transmission.
    pub fn process(&mut self) {
        match self.socket.as_mut() {
            None => {
                // If there's no socket available, try to connect to our remote.
                if self.open().is_ok() {
                    // If we just successfully opened the socket, flush old data from queue.
                    while self.queue.dequeue().is_some() {}
                }
            }
            Some(handle) => {
                if self.queue.ready() {
                    // Dequeue data from the queue into a larger block structure.
                    let mut packet =
                        DataPacket::new(&mut self.buffer, SUBSAMPLE_RATE);
                    while self
                        .queue
                        .peek()
                        .and_then(|batch| packet.add_batch(batch).ok())
                        .is_some()
                    {
                        // Dequeue the batch that we just added to the packet.
                        self.queue.dequeue();
                    }

                    // Transmit the data packet.
                    let size = packet.finish();
                    self.stack.send(handle, &self.buffer[..size]).ok();
                }
            }
        }
    }
}
