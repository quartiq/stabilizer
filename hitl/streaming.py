#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Implements HITL testing of Stabilizer data livestream capabilities.
"""
import asyncio
import sys
import argparse
import socket

from miniconf import Miniconf
from stabilizer.stream import StabilizerStream

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
        stream = StabilizerStream(args.port, timeout=0.5)

        # Configure the stream
        print(f'Configuring stream to target {".".join(map(str, local_ip))}:{args.port}')
        print('')
        await interface.command('stream_target', {'ip': local_ip, 'port': args.port}, retain=False)
        await interface.command('telemetry_period', 10, retain=False)

        # Verify frame reception
        print('Testing stream reception')
        print('')
        last_sequence = None
        for _ in range(5000):
            for (seqnum, _data) in stream.read_frame():
                assert sequence_delta(last_sequence, seqnum) == 0, \
                        f'Frame drop detected: 0x{last_sequence:08X} -> 0x{seqnum:08X}'
                last_sequence = seqnum

        # Disable the stream.
        print('Closing stream')
        print('')
        await interface.command('stream_target', {'ip': [0, 0, 0, 0], 'port': 0}, retain=False)
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