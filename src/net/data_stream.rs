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

use heapless::pool::{Box, Init, Pool, Uninit};

use super::NetworkReference;

const FRAME_COUNT: usize = 6;
const FRAME_SIZE: usize = 1024;

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
    let queue =
        cortex_m::singleton!(: Queue<StreamFrame, FRAME_COUNT> = Queue::new())
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

struct StreamFrame {
    format: u16,
    sequence_number: u16,
    buffer: Box<[u8; FRAME_SIZE], Init>,
    offset: usize,
}

impl StreamFrame {
    pub fn new(
        buffer: Box<[u8; FRAME_SIZE], Uninit>,
        format: u16,
        sequence_number: u16,
    ) -> Self {
        Self {
            format,
            offset: 4,
            sequence_number,
            buffer: unsafe { buffer.assume_init() },
        }
    }

    pub fn add_batch<F, const T: usize>(&mut self, mut f: F)
    where
        F: FnMut(&mut [u8]),
    {
        assert!(!self.is_full::<T>(), "Batch cannot be added to full frame");

        let result = f(&mut self.buffer[self.offset..self.offset + T]);

        self.offset += T;

        result
    }

    pub fn is_full<const T: usize>(&self) -> bool {
        self.offset + T >= self.buffer.len()
    }

    pub fn finish(&mut self) -> &[u8] {
        let offset = self.offset;
        self.buffer[0..2].copy_from_slice(&self.sequence_number.to_ne_bytes());
        self.buffer[2..4].copy_from_slice(&self.format.to_ne_bytes());
        &self.buffer[..offset]
    }
}

/// The data generator for a stream.
pub struct FrameGenerator {
    queue: Producer<'static, StreamFrame, FRAME_COUNT>,
    pool: &'static Pool<[u8; FRAME_SIZE]>,
    current_frame: Option<StreamFrame>,
    sequence_number: u16,
}

impl FrameGenerator {
    fn new(
        queue: Producer<'static, StreamFrame, FRAME_COUNT>,
        pool: &'static Pool<[u8; FRAME_SIZE]>,
    ) -> Self {
        Self {
            queue,
            pool,
            current_frame: None,
            sequence_number: 0,
        }
    }

    pub fn add<F, const T: usize>(&mut self, format: u16, f: F)
    where
        F: FnMut(&mut [u8]),
    {
        let sequence_number = self.sequence_number;
        self.sequence_number = self.sequence_number.wrapping_add(1);

        if self.current_frame.is_none() {
            if let Some(buffer) = self.pool.alloc() {
                self.current_frame.replace(StreamFrame::new(
                    buffer,
                    format,
                    sequence_number,
                ));
            } else {
                return;
            }
        }

        self.current_frame.as_mut().unwrap().add_batch::<_, T>(f);

        if self.current_frame.as_ref().unwrap().is_full::<T>() {
            if self
                .queue
                .enqueue(self.current_frame.take().unwrap())
                .is_err()
            {
                // Given that the queue is the same size as the number of frames available, this
                // should never occur.
                panic!("Frame enqueue failure")
            }
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
    queue: Consumer<'static, StreamFrame, FRAME_COUNT>,
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
        consumer: Consumer<'static, StreamFrame, FRAME_COUNT>,
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
