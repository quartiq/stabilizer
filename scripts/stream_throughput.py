#!/usr/bin/python3
"""
Author: Ryan Summers

Description: Provides a mechanism for measuring Stabilizer stream data throughput.
"""
import socket
import collections
import struct
import time
import logging

# Representation of a single data batch transmitted by Stabilizer.
Packet = collections.namedtuple('Packet', ['index', 'data'])

Format = collections.namedtuple('Format', ['sample_size_bytes', 'batch_format'])

# All supported formats by this reception script.
FORMAT = {
    0: Format(sample_size_bytes=8,
              batch_format='<{batch_size}H{batch_size}H{batch_size}H{batch_size}H')
}

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


class PacketParser:
    """ Utilize class used for parsing received UDP data. """

    def __init__(self):
        """ Initialize the parser. """
        self.buf = b''
        self.total_bytes = 0


    def ingress(self, data):
        """ Ingress received UDP data. """
        self.total_bytes += len(data)
        self.buf += data


    def parse_all_packets(self):
        """ Parse all received packets from the receive buffer.

        Returns:
            A list of received Packets.
        """
        packets = []
        while True:
            new_packets = self._parse()
            if new_packets:
                packets += new_packets
            else:
                return packets


    def _parse(self):
        """ Attempt to parse packets from the received buffer. """
        # Attempt to parse a block from the buffer.
        if len(self.buf) < 4:
            return None

        start_id, format_id, batch_count, batch_size = struct.unpack_from('<HHHB', self.buf)

        if format_id not in FORMAT:
            raise Exception(f'Unknown format specifier: {format_id}')

        frame_format = FORMAT[format_id]
        required_length = 7 + batch_count * frame_format.sample_size_bytes * batch_size

        if len(self.buf) < required_length:
            return None

        self.buf = self.buf[7:]

        packets = []
        for offset in range(batch_count):
            format_string = frame_format.batch_format.format(batch_size=batch_size)
            data = struct.unpack_from(format_string, self.buf)
            self.buf = self.buf[struct.calcsize(format_string):]
            packets.append(Packet(start_id + offset, data))

        return packets


def check_index(previous_index, next_index):
    """ Check if two indices are sequential. """
    if previous_index == -1:
        return True

    # Handle index roll-over. Indices are only stored in 16-bit numbers.
    if next_index < previous_index:
        next_index += 65536

    expected_index = previous_index + 1

    return next_index == expected_index


def main():
    """ Main program. """
    connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    connection.bind(("", 1111))

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s')

    last_index = -1

    drop_count = 0
    good_blocks = 0

    timer = Timer()
    parser = PacketParser()

    while True:
        # Receive any data over UDP and parse it.
        data = connection.recv(4096 * 4)
        if data and not timer.is_started():
            timer.start()

        parser.ingress(data)

        # Handle any received packets.
        for packet in parser.parse_all_packets():

            # Handle any dropped packets.
            if not check_index(last_index, packet.index):
                print('Drop from ', hex(last_index), hex(packet.index))
                if packet.index < (last_index + 1):
                    dropped = packet.index + 65536 - (last_index + 1)
                else:
                    dropped = packet.index - (last_index + 1)

                drop_count += dropped

            last_index = packet.index
            good_blocks += 1

        # Report the throughput periodically.
        if timer.is_triggered():
            drate = parser.total_bytes * 8 / 1e6 / timer.elapsed()

            print(f'''
Data Rate:       {drate:.3f} Mbps
Received Blocks: {good_blocks}
Dropped blocks:  {drop_count}

Metadata: {parser.total_bytes / 1e6:.3f} MB in {timer.elapsed():.2f} s
----
''')
            timer.arm()


if __name__ == '__main__':
    main()
