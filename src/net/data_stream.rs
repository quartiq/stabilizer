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
//! * **Magic word 0x057B** (u16): a constant to identify Stabilizer streaming data.
//! * **Format Code** (u8): a unique ID that indicates the serialization format of each batch of data
//!   in the frame. Refer to [StreamFormat] for further information.
//! * **Batch Count** (u8): the number of batches of data.
//! * **Sequence Number** (u32): an the sequence number of the first batch in the frame.
//!   This can be used to determine if and how many stream batches are lost.
//!
//! # Example
//! A sample Python script is available in `scripts/stream_throughput.py` to demonstrate reception
//! of livestreamed data.
use core::{fmt::Write, mem::MaybeUninit};
use heapless::{
    pool::{Box, Init, Pool, Uninit},
    spsc::{Consumer, Producer, Queue},
    String,
};
use num_enum::IntoPrimitive;
use serde::Serialize;
use serde_with::DeserializeFromStr;
use smoltcp_nal::embedded_nal::{nb, SocketAddr, UdpClientStack};

use super::NetworkReference;

// Magic first bytes indicating a UDP frame of straming data
const MAGIC: u16 = 0x057B;

// The size of the header, calculated in words.
// The header has a 16-bit magic word, an 8-bit format, 8-bit batch-size, and 32-bit sequence
// number, which corresponds to 8 bytes.
const HEADER_SIZE: usize = 8;

// The number of frames that can be buffered.
const FRAME_COUNT: usize = 4;

// The size of each frame in bytes.
// Ensure the resulting ethernet frame is within the MTU:
// 1500 MTU - 40 IP6 header - 8 UDP header
const FRAME_SIZE: usize = 1500 - 40 - 8;

// The size of the frame queue must be at least as large as the number of frame buffers. Every
// allocated frame buffer should fit in the queue.
const FRAME_QUEUE_SIZE: usize = FRAME_COUNT * 2;

type Frame = [MaybeUninit<u8>; FRAME_SIZE];

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
#[derive(Copy, Clone, Debug, DeserializeFromStr, PartialEq, Eq)]
pub struct StreamTarget(pub SocketAddr);

impl Default for StreamTarget {
    fn default() -> Self {
        Self("0.0.0.0:0".parse().unwrap())
    }
}

impl Serialize for StreamTarget {
    fn serialize<S: serde::Serializer>(
        &self,
        serializer: S,
    ) -> Result<S::Ok, S::Error> {
        let mut display: String<30> = String::new();
        write!(&mut display, "{}", self.0).unwrap();
        serializer.serialize_str(&display)
    }
}

impl core::str::FromStr for StreamTarget {
    type Err = &'static str;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let addr = SocketAddr::from_str(s)
            .map_err(|_| "Invalid socket address format")?;
        Ok(Self(addr))
    }
}

impl core::fmt::Display for StreamTarget {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        self.0.fmt(f)
    }
}

/// Specifies the format of streamed data
#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq, IntoPrimitive)]
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

    /// Streamed data in FLS (fiber length stabilization) format. See the FLS application for
    /// detailed definition.
    Fls = 2,
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

    let frame_pool = cortex_m::singleton!(: Pool<Frame> = Pool::new()).unwrap();

    let memory = cortex_m::singleton!(FRAME_DATA: [u8; core::mem::size_of::<u8>() * FRAME_SIZE * FRAME_COUNT] =
    [0; core::mem::size_of::<u8>() * FRAME_SIZE * FRAME_COUNT]).unwrap();

    frame_pool.grow(memory);

    let generator = FrameGenerator::new(producer, frame_pool);

    let stream = DataStream::new(stack, consumer, frame_pool);

    (generator, stream)
}

#[derive(Debug)]
struct StreamFrame {
    buffer: Box<Frame, Init>,
    offset: usize,
    batches: u8,
}

impl StreamFrame {
    pub fn new(
        buffer: Box<Frame, Uninit>,
        format_id: u8,
        sequence_number: u32,
    ) -> Self {
        let mut buffer = buffer.init([MaybeUninit::uninit(); FRAME_SIZE]);

        for (byte, buf) in MAGIC
            .to_le_bytes()
            .iter()
            .chain(&[format_id, 0])
            .chain(sequence_number.to_le_bytes().iter())
            .zip(buffer.iter_mut())
        {
            buf.write(*byte);
        }

        Self {
            buffer,
            offset: HEADER_SIZE,
            batches: 0,
        }
    }

    pub fn add_batch<F>(&mut self, mut f: F) -> usize
    where
        F: FnMut(&mut [MaybeUninit<u8>]) -> usize,
    {
        let len = f(&mut self.buffer[self.offset..]);
        self.offset += len;
        self.batches += 1;
        len
    }

    pub fn is_full(&self, len: usize) -> bool {
        self.offset + len > self.buffer.len()
    }

    pub fn finish(&mut self) -> &[MaybeUninit<u8>] {
        self.buffer[3].write(self.batches);
        &self.buffer[..self.offset]
    }
}

/// The data generator for a stream.
pub struct FrameGenerator {
    queue: Producer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
    pool: &'static Pool<Frame>,
    current_frame: Option<StreamFrame>,
    sequence_number: u32,
    format: u8,
}

impl FrameGenerator {
    fn new(
        queue: Producer<'static, StreamFrame, FRAME_QUEUE_SIZE>,
        pool: &'static Pool<Frame>,
    ) -> Self {
        Self {
            queue,
            pool,
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
    #[doc(hidden)]
    pub(crate) fn configure(&mut self, format: impl Into<u8>) {
        self.format = format.into();
    }

    /// Add a batch to the current stream frame.
    ///
    /// # Args
    /// * `f` - A closure that will be provided the buffer to write batch data into.
    ///         Returns the number of bytes written.
    pub fn add<F>(&mut self, f: F)
    where
        F: FnMut(&mut [MaybeUninit<u8>]) -> usize,
    {
        let sequence_number = self.sequence_number;
        self.sequence_number = self.sequence_number.wrapping_add(1);

        if self.current_frame.is_none() {
            if let Some(buffer) = self.pool.alloc() {
                self.current_frame.replace(StreamFrame::new(
                    buffer,
                    self.format,
                    sequence_number,
                ));
            } else {
                return;
            }
        }

        // Note(unwrap): We ensure the frame is present above.
        let current_frame = self.current_frame.as_mut().unwrap();

        let len = current_frame.add_batch(f);

        if current_frame.is_full(len) {
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
    frame_pool: &'static Pool<Frame>,
    remote: StreamTarget,
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
        frame_pool: &'static Pool<Frame>,
    ) -> Self {
        Self {
            stack,
            socket: None,
            remote: StreamTarget::default(),
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
        if self.socket.is_some() || self.remote.0.ip().is_unspecified() {
            return Err(());
        }

        let mut socket = self.stack.socket().or(Err(()))?;

        // We may fail to connect if we don't have an IP address yet.
        if self.stack.connect(&mut socket, self.remote.0).is_err() {
            self.stack.close(socket).unwrap();
            return Err(());
        }

        self.socket.replace(socket);

        log::info!("Opening stream");

        Ok(())
    }

    /// Configure the remote endpoint of the stream.
    ///
    /// # Args
    /// * `remote` - The destination to send stream data to.
    pub fn set_remote(&mut self, remote: StreamTarget) {
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
                    let buf = frame.finish();
                    let data = unsafe {
                        core::slice::from_raw_parts(
                            buf.as_ptr() as *const u8,
                            core::mem::size_of_val(buf),
                        )
                    };

                    // If we fail to send, it can only be because the socket got closed on us (i.e.
                    // address update due to DHCP). If this happens, reopen the socket.
                    match self.stack.send(handle, data) {
                        Ok(_) => {},

                        // Our IP address may have changedm so handle reopening the UDP stream.
                        Err(nb::Error::Other(smoltcp_nal::NetworkError::UdpWriteFailure(smoltcp_nal::smoltcp::socket::udp::SendError::Unaddressable))) => {
                            log::warn!( "IP address updated during stream. Reopening socket");
                            let socket = self.socket.take().unwrap();
                            self.stack.close(socket).unwrap();
                        }

                        // The buffer should clear up once ICMP resolves the IP address, so ignore
                        // this error.
                        Err(nb::Error::Other(smoltcp_nal::NetworkError::UdpWriteFailure(smoltcp_nal::smoltcp::socket::udp::SendError::BufferFull))) => {}

                        Err(other) => {
                            log::warn!("Unexpected UDP error during data stream: {other:?}");
                        }
                    }
                    self.frame_pool.free(frame.buffer)
                }
            }
        }
    }
}
