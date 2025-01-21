#!/usr/bin/python3
# pylint: disable=too-few-public-methods

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
    return wide & 0xFFFFFFFF


def get_local_ip(remote):
    """Get the local IP of a connection to the to a remote host.
    Returns a list of four octets."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote, 9))  # discard
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
            Trace(data[0], scale=DAC_VOLTS_PER_LSB, label="ADC0"),
            Trace(data[1], scale=DAC_VOLTS_PER_LSB, label="ADC1"),
            Trace(data[2], scale=DAC_VOLTS_PER_LSB, label="DAC0"),
            Trace(data[3], scale=DAC_VOLTS_PER_LSB, label="DAC1"),
        ]


class ThermostatEem:
    """Thermostat-EEM format"""

    format_id = 3

    def __init__(self, header, body):
        self.header = header
        self.body = body

    def size(self):
        """Return the data size of the frame in bytes"""
        return len(self.body)

    def to_si(self):
        """Return the parsed data in SI units"""
        return np.frombuffer(
            self.body, np.dtype([("input", "<f4", (4, 4)), ("output", "<f4", (4,))])
        )


class Frame:
    """Stream frame constisting of a header and multiple data batches"""

    # The magic header half-word at the start of each packet.
    magic = 0x057B
    header_fmt = struct.Struct("<HBBI")
    header = namedtuple("Header", "magic format_id batches sequence")
    parsers = {
        AdcDac.format_id: AdcDac,
        ThermostatEem.format_id: ThermostatEem,
    }

    @classmethod
    def parse(cls, data):
        """Parse known length frame"""
        header = cls.header._make(cls.header_fmt.unpack_from(data))
        if header.magic != cls.magic:
            raise ValueError(f"Bad frame magic: {header.magic:#04x}")
        try:
            parser = cls.parsers[header.format_id]
        except KeyError as exc:
            raise ValueError(f"No parser for format: {header.format_id}") from exc
        return parser(header, data[cls.header_fmt.size :])


class Stream(asyncio.DatagramProtocol):
    """Stabilizer streaming receiver protocol"""

    @classmethod
    async def open(cls, port=9293, addr="0.0.0.0", local="0.0.0.0", maxsize=1):
        """Open a UDP socket and start receiving frames"""
        loop = asyncio.get_running_loop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except NameError:
            pass  # Windows
        # Increase the OS UDP receive buffer size so that latency
        # spikes don't impact much. Achieving this may require increasing
        # the max allowed buffer size, e.g. via
        # `sudo sysctl net.core.rmem_max=26214400` but nowadays the default
        # max appears to be ~ 50 MiB already, at least on Linux.
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        # We need to specify which interface to receive multicasts from, or Windows may choose the
        # wrong one. Thus, use a bind address to figure out our local address for the interface
        # of interest. There's also an interface index, at least on linux, but apparently windows
        # sockets don't do that.
        if ipaddress.ip_address(addr).is_multicast:
            multiaddr = socket.inet_aton(addr)
            local = socket.inet_aton(local)
            sock.setsockopt(
                socket.IPPROTO_IP,
                socket.IP_ADD_MEMBERSHIP,
                multiaddr + local,
            )
        sock.bind((addr, port))
        return await loop.create_datagram_endpoint(
            lambda: cls(maxsize),
            sock=sock,
        )

    def __init__(self, maxsize):
        self.queue = asyncio.Queue(maxsize)

    def connection_made(self, _transport):
        logger.info("Connection made (listening)")

    def connection_lost(self, _exc):
        logger.info("Connection lost")

    def datagram_received(self, data, _addr):
        try:
            frame = Frame.parse(data)
        except ValueError as e:
            logger.warning("Parse error: %s", e)
            return
        if self.queue.full():
            old = self.queue.get_nowait()
            logger.debug("Dropping frame: %#08x", old.header.sequence)
        self.queue.put_nowait(frame)


async def measure(stream, duration):
    """Measure throughput and loss of stream reception"""

    expect = None
    received = 0
    lost = 0
    bytes = 0

    async def _record():
        while True:
            frame = await stream.queue.get()
            if expect is not None:
                lost += wrap(frame.header.sequence - expect)
            received += frame.header.batches
            expect = wrap(frame.header.sequence + frame.header.batches)
            bytes += frame.size()

    try:
        await asyncio.wait_for(_record(), timeout=duration)
    except asyncio.TimeoutError:
        pass

    logger.info("Received %g MB payload, %g MB/s", bytes / 1e6, bytes / 1e6 / duration)

    sent = received + lost
    loss = lost / sent if sent else 1
    logger.info("Loss: %s/%s batches (%g %%)", lost, sent, loss * 1e2)
    return loss


async def main():
    """Test CLI"""
    parser = argparse.ArgumentParser(description="Stabilizer streaming demo")
    parser.add_argument(
        "--port", type=int, default=9293, help="Local port to listen on [%(default)s]"
    )
    parser.add_argument(
        "--host", default="0.0.0.0", help="Local address to listen on [%(default)s]"
    )
    parser.add_argument(
        "--local",
        default="0.0.0.0",
        help="The local IP address to receive multicast frames on [%(default)s]",
    )
    parser.add_argument(
        "--broker", help="The MQTT broker address for local IP lookup [%(default)s]"
    )
    parser.add_argument(
        "--maxsize", type=int, default=1, help="Frame queue size [%(default)s]"
    )
    parser.add_argument(
        "--duration", type=float, default=1.0, help="Test duration [%(default)s]"
    )
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)
    if args.broker is not None:
        args.local = get_local_ip(args.broker)
    _transport, stream = await Stream.open(
        args.port, args.host, args.local, args.maxsize
    )
    await measure(stream, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
