#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Implements HITL testing of Stabilizer data livestream capabilities.
"""
import asyncio
import sys
import time
import argparse
import socket
import struct
import logging

from miniconf import Miniconf

def _get_ip(broker):
    """ Get the IP of the local device.

    Args:
        broker: The broker IP of the test. Used to select an interface to get the IP of.

    Returns:
        The IP as an array of integers.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((broker, 1883))
        address = sock.getsockname()[0]
    finally:
        sock.close()

    return list(map(int, address.split('.')))


def sequence_delta(previous_sequence, next_sequence):
    """ Check the number of items between two sequence numbers. """
    if previous_sequence is None:
        return 0

    delta = next_sequence - (previous_sequence + 1)
    return delta & 0xFFFFFFFF


class StabilizerStream:
    """ Provides access to Stabilizer's livestreamed data. """

    # The magic header half-word at the start of each packet.
    MAGIC_HEADER = 0x057B

    # The struct format of the header.
    HEADER_FORMAT = '<HBBI'

    # All supported formats by this reception script.
    #
    # The items in this dict are functions that will be provided the sample batch size and will
    # return the struct deserialization code to unpack a single batch.
    FORMAT = {
        1: lambda batch_size: f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H'
    }

    def __init__(self, port):
        """ Initialize the stream. """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", port))
        self.socket.settimeout(0.5)


    def clear(self, duration=5):
        """ Clear the socket RX buffer by reading all available data.

        Args:
            duration: The maximum duration in seconds to read data for.
        """
        start = time.time()
        while (time.time() - start) < duration:
            try:
                self.socket.recv(4096)
            except socket.timeout:
                return


    def read_frame(self):
        """ Read a single frame from the stream.

        Returns:
            Yields the (seqnum, data) of the batches available in the frame.
        """
        buf = self.socket.recv(4096)

        # Attempt to parse a block from the buffer.
        if len(buf) < struct.calcsize(self.HEADER_FORMAT):
            return

        # Parse out the packet header
        magic, format_id, batch_size, sequence_number = struct.unpack_from(self.HEADER_FORMAT, buf)
        buf = buf[struct.calcsize(self.HEADER_FORMAT):]

        if magic != self.MAGIC_HEADER:
            logging.warning('Encountered bad magic header: %s', hex(magic))
            return

        frame_format = self.FORMAT[format_id](batch_size)

        batch_count = int(len(buf) / struct.calcsize(frame_format))

        for offset in range(batch_count):
            data = struct.unpack_from(frame_format, buf)
            buf = buf[struct.calcsize(frame_format):]
            yield (sequence_number + offset, data)


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Loopback tests for Stabilizer HITL testing',)
    parser.add_argument('prefix', type=str,
                        help='The MQTT topic prefix of the target')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('--port', '-p', default=2000, type=int,
                        help='The UDP port to use for streaming')

    args = parser.parse_args()

    async def test():
        """ The actual testing being completed. """
        local_ip = _get_ip(args.broker)
        interface = await Miniconf.create(args.prefix, args.broker)
        stream = StabilizerStream(args.port)

        # Configure the stream
        print(f'Configuring stream to target {".".join(map(str, local_ip))}:{args.port}')
        print('')
        await interface.command('stream_target', {'ip': local_ip, 'port': args.port})

        # Verify frame reception
        print('Testing stream reception')
        print('')
        last_sequence = None
        for _ in range(5000):
            for (seqnum, _data) in stream.read_frame():
                assert sequence_delta(last_sequence, seqnum) == 0
                last_sequence = seqnum

        # Disable the stream.
        print('Closing stream')
        print('')
        await interface.command('stream_target', {'ip': [0, 0, 0, 0], 'port': 0})
        stream.clear()

        print('Verifying no further data is received')
        try:
            for _ in stream.read_frame():
                raise Exception('Unexpected data encountered on stream')
        except socket.timeout:
            pass
        print('PASS')


    loop = asyncio.get_event_loop()
    sys.exit(loop.run_until_complete(test()))


if __name__ == '__main__':
    main()
