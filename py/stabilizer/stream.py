#!/usr/bin/python3
"""Stabilizer streaming receiver and parsers"""

import argparse
import asyncio
import logging
import struct
from collections import namedtuple
from dataclasses import dataclass

import numpy as np

from . import DAC_VOLTS_PER_LSB

logger = logging.getLogger(__name__)


def wrap(wide):
    """Wrap to 32 bit integer"""
    return wide & 0xffffffff


class AdcDac:
    """Stabilizer default striming data format"""
    format_id = 1

    def __init__(self, header, body):
        self.header = header
        frame = np.frombuffer(body, "<i2").reshape(-1, 4, header.batch_size)
        self.batch_count = frame.shape[0]
        self.data = frame.swapaxes(0, 1).reshape(4, -1)
        # convert DAC offset binary to two's complement
        self.data[2:] ^= np.int16(0x8000)

    def to_mu(self):
        """Return the raw data in machine units"""
        return self.data

    def to_si(self):
        """Convert the raw data to SI units"""
        return {
            "adc": self.data[:2] * DAC_VOLTS_PER_LSB,
            "dac": self.data[2:] * DAC_VOLTS_PER_LSB,
        }


class StabilizerStream(asyncio.DatagramProtocol):
    """Stabilizer streaming receiver protocol"""
    # The magic header half-word at the start of each packet.
    magic = 0x057B
    header_fmt = struct.Struct("<HBBI")
    header = namedtuple("Header", "magic format_id batch_size sequence")
    parsers = {
        AdcDac.format_id: AdcDac,
    }

    @classmethod
    async def open(cls, local_addr, maxsize=1):
        """Open a UDP socket and start receiving frames"""
        loop = asyncio.get_running_loop()
        _transport, protocol = await loop.create_datagram_endpoint(
            lambda: cls(maxsize), local_addr=local_addr)
        return protocol

    def __init__(self, maxsize):
        self.queue = asyncio.Queue(maxsize)

    def connection_made(self, _transport):
        logger.info("Connection made (listening)")

    def connection_lost(self, _exc):
        logger.info("Connection lost")

    def datagram_received(self, data, _addr):
        header = self.header._make(self.header_fmt.unpack_from(data))
        if header.magic != self.magic:
            logger.warning("Bad frame magic: %#04x, ignoring", header.magic)
            return
        try:
            parser = self.parsers[header.format_id]
        except KeyError:
            logger.warning("No parser for format %s, ignoring", header.format_id)
            return
        body = data[self.header_fmt.size:]
        frame = parser(header, body)
        try:
            self.queue.put_nowait(frame)
        except asyncio.QueueFull:
            logger.debug("Dropping frame: %#08x", header.sequence)


async def measure(stream, duration):
    """Measure throughput and loss of stream reception"""
    @dataclass
    class _Statistics:
        expect = None
        received = 0
        lost = 0
        bytes = 0
    stat = _Statistics()

    async def _record():
        while True:
            data = await stream.queue.get()
            if stat.expect is not None:
                stat.lost += wrap(data.header.sequence - stat.expect)
            stat.received += data.batch_count
            stat.expect = wrap(data.header.sequence + data.batch_count)
            stat.bytes += data.data.nbytes
            data.to_si()

    try:
        await asyncio.wait_for(_record(), timeout=duration)
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
    """Test CLI"""
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
    await measure(stream, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
