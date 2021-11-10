#!/usr/bin/python3

import asyncio
import logging
import struct
from collections import namedtuple

import numpy as np

logger = logging.getLogger(__name__)


def wrap(x):
    """Wrap to 32 bit integer"""
    return x & 0xffffffff


class AdcDac:
    """Stabilizer default striming data format"""
    format_id = 1
    adc_volt_per_lsb = 4.096 * 5 / (1 << 16)
    dac_volt_per_lsb = adc_volt_per_lsb

    def __init__(self, header, body):
        self.header = header
        frame = np.frombuffer(body, "<i2").reshape(-1, 4, header.batch_size)
        self.batch_count = frame.shape[0]
        self.data = frame.swapaxes(0, 1).reshape(4, -1)
        # convert DAC offset binary to two's complement
        self.data[2:] ^= np.int16(0x8000)

    def to_si(self):
        return {
            "adc": self.data[:2] * self.adc_volt_per_lsb,
            "dac": self.data[2:] * self.dac_volt_per_lsb,
        }


class StabilizerStream(asyncio.DatagramProtocol):
    # The magic header half-word at the start of each packet.
    magic = 0x057B
    header_fmt = struct.Struct("<HBBI")
    header = namedtuple("Header", "magic format_id batch_size sequence")
    parsers = {
        AdcDac.format_id: AdcDac,
    }

    @classmethod
    async def open(cls, local_addr, maxsize=1):
        loop = asyncio.get_running_loop()
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: cls(maxsize), local_addr=local_addr)
        return protocol

    def __init__(self, maxsize):
        self.queue = asyncio.Queue(maxsize)

    def connection_made(self, transport):
        logger.info("Connection made (listening)")
        self.transport = transport

    def connection_lost(self):
        logger.info("Connection lost")
        del self.transport

    def datagram_received(self, frame, remote_addr):
        header = self.header._make(self.header_fmt.unpack_from(frame))
        if header.magic != self.magic:
            logger.warning("Bad frame magic: %#04x, ignoring", header.magic)
            return
        try:
            parser = self.parsers[header.format_id]
        except KeyError:
            logger.warning("No parser for format %s, ignoring", header.format_id)
            return
        body = frame[self.header_fmt.size:]
        data = parser(header, body)
        try:
            self.queue.put_nowait(data)
        except asyncio.QueueFull:
            logger.debug("Dropping frame: %#08x", header.sequence)


async def measure(stream, duration):
    class Statistics:
        expect = None
        received = 0
        lost = 0
        bytes = 0
    stat = Statistics()

    async def record():
        while True:
            data = await stream.queue.get()
            if stat.expect is not None:
                stat.lost += wrap(data.header.sequence - stat.expect)
            stat.received += data.batch_count
            stat.expect = wrap(data.header.sequence + data.batch_count)
            stat.bytes += data.data.nbytes
            data.to_si()

    try:
        await asyncio.wait_for(record(), timeout=duration)
    except asyncio.TimeoutError:
        pass

    logger.info("Received %g MB, %g MB/s", stat.bytes/1e6,
            stat.bytes/1e6/duration)

    sent = stat.received + stat.lost
    if sent:
        loss = stat.lost/sent
    else:
        loss = 1
    logger.info("Loss: %s/%s batches (%g %%)", stat.lost, sent, loss*1e2)
    return loss


async def main():
    import argparse
    parser = argparse.ArgumentParser(description="Stabilizer streaming demo")
    parser.add_argument("--port", type=int, default=9293,
                        help="Local port to listen on")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Local address to listen on")
    parser.add_argument("--maxsize", type=int, default=1,
                        help="Frame queue size")
    parser.add_argument("--duration", type=float, default=1.,
                        help="Test duration")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    stream = await StabilizerStream.open((args.host, args.port), args.maxsize)
    loss = await measure(stream, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
