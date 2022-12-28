#!/usr/bin/python3
"""Stabilizer streaming receiver and parsers"""

import argparse
import asyncio
import logging
import struct
import socket
from collections import namedtuple
from dataclasses import dataclass

import numpy as np

from . import DAC_VOLTS_PER_LSB

logger = logging.getLogger(__name__)

Trace = namedtuple("Trace", "values scale label")


def wrap(wide):
    """Wrap to 32 bit integer"""
    return wide & 0xffffffff


def get_local_ip(remote):
    """Get the local IP of a connection to the to a remote host.
    Returns a list of four octets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote, 1883))
        address = sock.getsockname()[0]
    finally:
        sock.close()
    return list(map(int, address.split(".")))


class AdcDac:
    """Stabilizer default striming data format"""
    format_id = 1

    def __init__(self, header, body):
        self.header = header
        self.body = body

    def batch_count(self):
        """Return the number of batches in the frame"""
        return self.size() // (4 * 2 * self.header.batch_size)

    def size(self):
        """Return the data size of the frame in bytes"""
        return len(self.body)

    def to_mu(self):
        """Return the raw data in machine units"""
        data = np.frombuffer(self.body, "<i2")
        data = data.reshape(-1, 4, self.header.batch_size)
        data = data.swapaxes(0, 1).reshape(4, -1)
        # convert DAC offset binary to two's complement
        data[2:] ^= np.int16(0x8000)
        return data

    def to_si(self):
        """Convert the raw data to SI units"""
        data = self.to_mu() * DAC_VOLTS_PER_LSB
        return {
            "adc": data[:2],
            "dac": data[2:],
        }

    def to_traces(self):
        """Convert the raw data to labelled Trace instances"""
        data = self.to_mu()
        return [
            Trace(data[0], scale=DAC_VOLTS_PER_LSB, label='ADC0'),
            Trace(data[1], scale=DAC_VOLTS_PER_LSB, label='ADC1'),
            Trace(data[2], scale=DAC_VOLTS_PER_LSB, label='DAC0'),
            Trace(data[3], scale=DAC_VOLTS_PER_LSB, label='DAC1')
        ]


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
        transport, protocol = await loop.create_datagram_endpoint(
            lambda: cls(maxsize), local_addr=local_addr)
        # Increase the OS UDP receive buffer size to 4 MiB so that latency
        # spikes don't impact much. Achieving 4 MiB may require increasing
        # the max allowed buffer size, e.g. via
        # `sudo sysctl net.core.rmem_max=26214400` but nowadays the default
        # max appears to be ~ 50 MiB already.
        sock = transport.get_extra_info("socket")
        if sock is not None:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
        return transport, protocol

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
        frame = parser(header, data[self.header_fmt.size:])
        if self.queue.full():
            old = self.queue.get_nowait()
            logger.debug("Dropping frame: %#08x", old.header.sequence)
        self.queue.put_nowait(frame)


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
            frame = await stream.queue.get()
            if stat.expect is not None:
                stat.lost += wrap(frame.header.sequence - stat.expect)
            batch_count = frame.batch_count()
            stat.received += batch_count
            stat.expect = wrap(frame.header.sequence + batch_count)
            stat.bytes += frame.size()
            # test conversion
            # frame.to_si()

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
    _transport, stream = await StabilizerStream.open(
        (args.host, args.port), args.maxsize)
    await measure(stream, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
