//! Stabilizer data stream capabilities
//!
//! # Design
//! Data streamining utilizes UDP packets to send live data streams at high throughput.
//! Packets are always sent in a best-effort fashion, and data may be dropped.
//!
//! Stabilizer organizes livestreamed data into batches within a "Frame" that will be sent as a UDP
//! packet. Each frame consits of a header followed by sequential batch serializations. The packet
//! header is constant for all streaming capabilities, but the serialization format after the header
//! is application-defined.
//!
//! ## Frame Header
//! The header consists of the following, all in little-endian.
//!
//! * **Magic word 0x057B** <u16>: a constant to identify Stabilizer streaming data.
//! * **Format Code** <u8>: a unique ID that indicates the serialization format of each batch of data
//!   in the frame. Refer to [StreamFormat] for further information.
//! * **Batch Size** <u8>: the number of samples in each batch of data.
//! * **Sequence Number** <u32>: an the sequence number of the first batch in the frame.
//!   This can be used to determine if and how many stream batches are lost.
//!
//! # Example
//! A sample Python script is available in `scripts/stream_throughput.py` to demonstrate reception
//! of livestreamed data.
use heapless::spsc::{Consumer, Producer, Queue};
use miniconf::MiniconfAtomic;
use num_enum::IntoPrimitive;
use serde::{Deserialize, Serialize};
use smoltcp_nal::embedded_nal::{IpAddr, Ipv4Addr, SocketAddr, UdpClientStack};

use heapless::pool::{Box, Init, Pool, Uninit};

use super::NetworkReference;

const MAGIC_WORD: u16 = 0x057B;

// The size of the header, calculated in bytes.
// The header has a 16-bit magic word, an 8-bit format, 8-bit batch-size, and 32-bit sequence
// number, which corresponds to 8 bytes total.
const HEADER_SIZE: usize = 8;

// The number of frames that can be buffered.
const FRAME_COUNT: usize = 4;

// The size of each livestream frame in bytes.
const FRAME_SIZE: usize = 1024 + HEADER_SIZE;

// The size of the frame queue must be at least as large as the number of frame buffers. Every
// allocated frame buffer should fit in the queue.
const FRAME_QUEUE_SIZE: usize = FRAME_COUNT * 2;

// Static storage used for a heapless::Pool of frame buffers.
static mut FRAME_DATA: [u8; FRAME_SIZE * FRAME_COUNT] =
    [0; FRAME_SIZE * FRAME_COUNT];

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
#[derive(
    Copy, Clone, Debug, MiniconfAtomic, Serialize, Deserialize, Default,
)]
pub struct StreamTarget {
    pub ip: [u8; 4],
    pub port: u16,
}

/// Specifies the format of streamed data
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, IntoPrimitive)]
pub enum StreamFormat {
    /// Reserved, unused format specifier.
    Unknown = 0,

    /// Streamed data contains ADC0, ADC1, DAC0, and DAC1 sequentially in little-endian format.
    ///
    /// # Example
    /// With a batch size of 2, the serialization would take the following form:
    /// ```
    /// <ADC0[0]> <ADC0[1]> <ADC1[0]> <ADC1[1]> <DAC0[0]> <DAC0[1]> <DAC1[0]> <DAC1[1]>
    /// ```
    AdcDacData = 1,
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
) -> (FrameGenerator, DataStream) {
    // The queue needs to be at least as large as the frame count to ensure that every allocated
    // frame can potentially be enqueued for transmission.
    let queue =
        cortex_m::singleton!(: Queue<StreamFrame, FRAME_QUEUE_SIZE> = Queue::new())
            .unwrap();
    let (producer, consumer) = queue.split();

    let frame_pool =
        cortex_m::singleton!(: Pool<[u8; FRAME_SIZE]>= Pool::new()).unwrap();

    // Note(unsafe): We guarantee that FRAME_DATA is only accessed once in this function.
    let memory = unsafe { &mut FRAME_DATA };
    frame_pool.grow(memory);

    let generator = FrameGenerator::new(producer, frame_pool);

    let stream = DataStream::new(stack, consumer, frame_pool);

    (generator, stream)
}

#[derive(Debug)]
struct StreamFrame {
    buffer: Box<[u8; FRAME_SIZE], Init>,
    offset: usize,
}

impl StreamFrame {
    pub fn new(
        buffer: Box<[u8; FRAME_SIZE], Uninit>,
        format: u8,
        buffer_size: u8,
        sequence_number: u32,
    ) -> Self {
        let mut buffer = unsafe { buffer.assume_init() };
        buffer[0..2].copy_from_slice(&MAGIC_WORD.to_ne_bytes());
        buffer[2] = format;
        buffer[3] = buffer_size;
        buffer[4..8].copy_from_slice(&sequence_number.to_ne_bytes());
        Self {
            buffer,
            offset: HEADER_SIZE,
        }
    }

    pub fn add_batch<F, const T: usize>(&mut self, mut f: F)
    where
        F: FnMut(&mut [u8]),
    {
        f(&mut self.buffer[self.offset..self.offset + T]);

        self.offset += T;
    }

    pub fn is_full<const T: usize>(&self) -> bool {
        self.offset + T > self.buffer.len()
    }

    pub fn finish(&mut self) -> &[u8] {
        &self.buffer[..self.offset]
    }
}

/// The data generator for a stream.
pub struct FrameGenerator {
    queue: Producer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
    pool: &'static Pool<[u8; FRAME_SIZE]>,
    current_frame: Option<StreamFrame>,
    sequence_number: u32,
    format: u8,
    batch_size: u8,
}

impl FrameGenerator {
    fn new(
        queue: Producer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
        pool: &'static Pool<[u8; FRAME_SIZE]>,
    ) -> Self {
        Self {
            queue,
            pool,
            batch_size: 0,
            format: StreamFormat::Unknown.into(),
            current_frame: None,
            sequence_number: 0,
        }
    }

    /// Configure the format of the stream.
    ///
    /// # Note:
    /// This function shall only be called once upon initializing streaming
    ///
    /// # Args
    /// * `format` - The desired format of the stream.
    /// * `batch_size` - The number of samples in each data batch. See
    /// [crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE]
    #[doc(hidden)]
    pub(crate) fn configure(&mut self, format: impl Into<u8>, batch_size: u8) {
        self.format = format.into();
        self.batch_size = batch_size;
    }

    /// Add a batch to the current stream frame.
    ///
    /// # Args
    /// * `f` - A closure that will be provided the buffer to write batch data into. The buffer will
    ///   be the size of the `T` template argument.
    pub fn add<F, const T: usize>(&mut self, f: F)
    where
        F: FnMut(&mut [u8]),
    {
        let sequence_number = self.sequence_number;
        self.sequence_number = self.sequence_number.wrapping_add(1);

        if self.current_frame.is_none() {
            if let Some(buffer) = self.pool.alloc() {
                self.current_frame.replace(StreamFrame::new(
                    buffer,
                    self.format as u8,
                    self.batch_size,
                    sequence_number,
                ));
            } else {
                return;
            }
        }

        // Note(unwrap): We ensure the frame is present above.
        let current_frame = self.current_frame.as_mut().unwrap();

        current_frame.add_batch::<_, T>(f);

        if current_frame.is_full::<T>() {
            // Note(unwrap): The queue is designed to be at least as large as the frame buffer
            // count, so this enqueue should always succeed.
            self.queue
                .enqueue(self.current_frame.take().unwrap())
                .unwrap();
        }
    }
}

/// The "consumer" portion of the data stream.
///
/// # Note
/// This is responsible for consuming data and sending it over UDP.
pub struct DataStream {
    stack: NetworkReference,
    socket: Option<<NetworkReference as UdpClientStack>::UdpSocket>,
    queue: Consumer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
    frame_pool: &'static Pool<[u8; FRAME_SIZE]>,
    remote: SocketAddr,
}

impl DataStream {
    /// Construct a new data streamer.
    ///
    /// # Args
    /// * `stack` - A reference to the shared network stack.
    /// * `consumer` - The read side of the queue containing data to transmit.
    /// * `frame_pool` - The Pool to return stream frame objects into.
    fn new(
        stack: NetworkReference,
        consumer: Consumer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
        frame_pool: &'static Pool<[u8; FRAME_SIZE]>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            remote: StreamTarget::default().into(),
            queue: consumer,
            frame_pool,
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
                    while let Some(frame) = self.queue.dequeue() {
                        self.frame_pool.free(frame.buffer);
                    }
                }
            }
            Some(handle) => {
                if let Some(mut frame) = self.queue.dequeue() {
                    // Transmit the frame and return it to the pool.
                    self.stack.send(handle, frame.finish()).ok();
                    self.frame_pool.free(frame.buffer)
                }
            }
        }
    }
}
