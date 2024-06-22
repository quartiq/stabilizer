#!/usr/bin/python3
"""Stabilizer streaming receiver and parsers"""

import argparse
import asyncio
import logging
import struct
import socket
import ipaddress
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
        return sock.getsockname()[0]
    finally:
        sock.close()


class AdcDac:
    """Stabilizer default striming data format"""
    format_id = 1

    def __init__(self, header, body):
        self.header = header
        self.body = body

    def size(self):
        """Return the data size of the frame in bytes"""
        return len(self.body)

    def to_mu(self):
        """Return the raw data in machine units"""
        data = np.frombuffer(self.body, "<i2")
        # batch, channel, sample
        data = data.reshape(self.header.batches, 4, -1)
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
    header = namedtuple("Header", "magic format_id batches sequence")
    parsers = {
        AdcDac.format_id: AdcDac,
    }

    @classmethod
    async def open(cls, addr, port, broker, maxsize=1):
        """Open a UDP socket and start receiving frames"""
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # Increase the OS UDP receive buffer size to 4 MiB so that latency
        # spikes don't impact much. Achieving 4 MiB may require increasing
        # the max allowed buffer size, e.g. via
        # `sudo sysctl net.core.rmem_max=26214400` but nowadays the default
        # max appears to be ~ 50 MiB already.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)

        # We need to specify which interface to receive broadcasts from, or Windows may choose the
        # wrong one. Thus, use the broker address to figure out our local address for the interface
        # of interest.
        if ipaddress.ip_address(addr).is_multicast:
            print('Subscribing to multicast')
            group = socket.inet_aton(addr)
            iface = socket.inet_aton('.'.join([str(x) for x in get_local_ip(broker)]))
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, group + iface)
            sock.bind(('', port))
        else:
            sock.bind((addr, port))

        transport, protocol = await loop.create_datagram_endpoint(lambda: cls(maxsize), sock=sock)
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
            stat.received += frame.header.batches
            stat.expect = wrap(frame.header.sequence + frame.header.batches)
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
    parser.add_argument("--broker", default="mqtt",
                        help="The MQTT broker address")
    parser.add_argument("--maxsize", type=int, default=1,
                        help="Frame queue size")
    parser.add_argument("--duration", type=float, default=1.,
                        help="Test duration")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    _transport, stream = await StabilizerStream.open(
        args.host, args.port, args.broker, args.maxsize)
    await measure(stream, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
