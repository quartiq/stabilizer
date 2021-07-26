#!/usr/bin/python3
"""
Author: Ryan Summers

Description: Provides a mechanism for measuring Stabilizer stream data throughput.
"""
import argparse
import socket
import collections
import struct
import time
import logging

# Representation of a single data batch transmitted by Stabilizer.
Packet = collections.namedtuple('Packet', ['index', 'data'])

# The magic header half-word at the start of each packet.
MAGIC_HEADER = 0x057B

# The struct format of the header.
HEADER_FORMAT = '<HBBI'

# All supported formats by this reception script.
#
# The items in this dict are functions that will be provided the sampel batch size and will return
# the struct deserialization code to unpack a single batch.
FORMAT = {
    1: lambda batch_size: f'<{batch_size}H{batch_size}H{batch_size}H{batch_size}H'
}

def parse_packet(buf):
    """ Attempt to parse packets from the received buffer. """
    # Attempt to parse a block from the buffer.
    if len(buf) < struct.calcsize(HEADER_FORMAT):
        return None

    # Parse out the packet header
    magic, format_id, batch_size, sequence_number = struct.unpack_from(HEADER_FORMAT, buf)
    buf = buf[struct.calcsize(HEADER_FORMAT):]

    if magic != MAGIC_HEADER:
        logging.warning('Encountered bad magic header: %s', hex(magic))
        return None

    if format_id not in FORMAT:
        raise Exception(f'Unknown format specifier: {format_id}')

    frame_format = FORMAT[format_id](batch_size)

    batch_count = len(buf) / struct.calcsize(frame_format)

    packets = []
    for offset in range(batch_count):
        data = struct.unpack_from(frame_format, buf)
        buf = buf[struct.calcsize(frame_format):]
        packets.append(Packet(sequence_number + offset, data))

    return packets


class Timer:
    """ A basic timer for measuring elapsed time periods. """

    def __init__(self, period=1.0):
        """ Create the timer with the provided period. """
        self.start_time = time.time()
        self.trigger_time = self.start_time + period
        self.period = period
        self.started = False


    def is_triggered(self):
        """ Check if the timer period has elapsed. """
        now = time.time()
        return now >= self.trigger_time


    def start(self):
        """ Start the timer. """
        self.start_time = time.time()
        self.started = True


    def is_started(self):
        """ Check if the timer has started. """
        return self.started


    def arm(self):
        """ Arm the timer trigger. """
        self.trigger_time = time.time() + self.period


    def elapsed(self):
        """ Get the elapsed time since the timer was started. """
        now = time.time()
        return now - self.start_time


def sequence_delta(previous_sequence, next_sequence):
    """ Check the number of items between two sequence numbers. """
    if previous_sequence is None:
        return 0

    delta = next_sequence - (previous_sequence + 1)
    return delta & 0xFFFFFFFF


def main():
    """ Main program. """
    parser = argparse.ArgumentParser(description='Measure Stabilizer livestream quality')
    parser.add_argument('--port', default=1111, help='The port that stabilizer is streaming to')

    args = parser.parse_args()

    connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connection.bind(("", args.port))

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s')

    last_index = None

    drop_count = 0
    good_blocks = 0
    total_bytes = 0

    timer = Timer()

    while True:
        # Receive any data over UDP and parse it.
        data = connection.recv(1024)
        if data and not timer.is_started():
            timer.start()

        # Handle any received packets.
        total_bytes += len(data)
        packet = parse_packet(data)

        if packet:
            # Handle any dropped packets.
            drop_count += sequence_delta(last_index, packet.index)
            last_index = packet.index
            good_blocks += 1

        # Report the throughput periodically.
        if timer.is_triggered():
            drate = total_bytes * 8 / 1e6 / timer.elapsed()

            print(f'''
Data Rate:       {drate:.3f} Mbps
Received Blocks: {good_blocks}
Dropped blocks:  {drop_count}

Metadata: {total_bytes / 1e6:.3f} MB in {timer.elapsed():.2f} s
----
''')
            timer.arm()


if __name__ == '__main__':
    main()
