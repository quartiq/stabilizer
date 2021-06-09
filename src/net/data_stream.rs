use core::borrow::BorrowMut;
use heapless::spsc::{Consumer, Producer, Queue};
use smoltcp_nal::embedded_nal::{SocketAddr, UdpClientStack};

use super::NetworkReference;
use crate::hardware::design_parameters::SAMPLE_BUFFER_SIZE;

// The number of samples contained in a single block. Note that each sample corresponds ot 8 byte
// s(2 bytes per ADC/DAC code, 4 codes total).
const BLOCK_SAMPLE_SIZE: usize = 50;

// The number of data blocks that we will buffer in the queue.
const BLOCK_BUFFER_SIZE: usize = 30;

const SUBSAMPLE_RATE: usize = 1;

pub fn setup_streaming(
    stack: NetworkReference,
) -> (BlockGenerator, DataStream) {
    let queue = cortex_m::singleton!(: Queue<AdcDacData, BLOCK_BUFFER_SIZE> = Queue::new()).unwrap();

    let (producer, consumer) = queue.split();

    let generator = BlockGenerator::new(producer);

    let stream = DataStream::new(stack, consumer);

    (generator, stream)
}

fn serialize_blocks<'a>(
    buffer: &'a mut [u8],
    max_buffer_size: usize,
    queue: &mut Consumer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
) -> &'a [u8] {
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

#[derive(Debug, Copy, Clone)]
pub struct AdcDacData {
    block_id: u32,
    adcs: [[u16; BLOCK_SAMPLE_SIZE]; 2],
    dacs: [[u16; BLOCK_SAMPLE_SIZE]; 2],
}

pub struct BlockGenerator {
    queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    current_block: AdcDacData,
    num_samples: usize,
}

impl BlockGenerator {
    pub fn new(
        queue: Producer<'static, AdcDacData, BLOCK_BUFFER_SIZE>,
    ) -> Self {
        Self {
            queue,
            current_block: AdcDacData {
                block_id: 0,
                adcs: [[0; BLOCK_SAMPLE_SIZE]; 2],
                dacs: [[0; BLOCK_SAMPLE_SIZE]; 2],
            },
            num_samples: 0,
        }
    }

    pub fn send(
        &mut self,
        adcs: &[&mut [u16; SAMPLE_BUFFER_SIZE]; 2],
        dacs: &[&mut [u16; SAMPLE_BUFFER_SIZE]; 2],
    ) {
        let mut processed_samples = 0;

        while processed_samples < SAMPLE_BUFFER_SIZE {
            let remaining_samples = SAMPLE_BUFFER_SIZE - processed_samples;
            let free_space = BLOCK_SAMPLE_SIZE - self.num_samples;
            let copy_sample_length = if remaining_samples < free_space {
                remaining_samples
            } else {
                free_space
            };

            let start_src = self.num_samples;
            let end_src = start_src + copy_sample_length;

            let start_dst = processed_samples;
            let end_dst = start_dst + copy_sample_length;

            self.current_block.adcs[0][start_src..end_src]
                .copy_from_slice(&adcs[0][start_dst..end_dst]);
            self.current_block.adcs[1][start_src..end_src]
                .copy_from_slice(&adcs[1][start_dst..end_dst]);
            self.current_block.dacs[0][start_src..end_src]
                .copy_from_slice(&dacs[0][start_dst..end_dst]);
            self.current_block.dacs[1][start_src..end_src]
                .copy_from_slice(&dacs[1][start_dst..end_dst]);

            self.num_samples += copy_sample_length;

            // If the data block is full, push it onto the queue.
            if self.num_samples == BLOCK_SAMPLE_SIZE {
                // Note: We silently ignore dropped blocks here. The queue can fill up if the
                // service routing isn't being called often enough.
                self.queue.enqueue(self.current_block).ok();

                self.current_block.block_id =
                    self.current_block.block_id.wrapping_add(1);
                self.num_samples = 0;
            }

            processed_samples += copy_sample_length;
        }
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
    adcs: [[u16; BLOCK_SAMPLE_SIZE]; 2],
    dacs: [[u16; BLOCK_SAMPLE_SIZE]; 2],
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
                    buf[offset..offset + 2]
                        .copy_from_slice(&sample.to_be_bytes());
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
            let mut handle = self.socket.borrow_mut().unwrap();
            let capacity = self
                .stack
                .lock(|stack| {
                    stack.with_udp_socket(handle, |socket| {
                        socket.payload_send_capacity()
                    })
                })
                .unwrap();

            let data =
                serialize_blocks(&mut self.buffer, capacity, &mut self.queue);

            // Transmit the data block.
            self.stack.send(&mut handle, &data).ok();
        }
    }
}
