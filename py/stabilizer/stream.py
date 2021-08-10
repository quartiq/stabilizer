#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides a means of accessing Stabilizer livestream data.
"""
import socket
import time
import logging
import struct

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

    def __init__(self, port, timeout=None):
        """ Initialize the stream.

        Args:
            port: The UDP port to receive the stream from.
            timeout: The timeout to set on the UDP socket.
        """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(("", port))
        self.total_bytes = 0

        if timeout is not None:
            self.socket.settimeout(timeout)


    def clear(self, duration=1):
        """ Clear the socket RX buffer by reading all available data.

        Args:
            duration: The maximum duration in seconds to read data for.
        """
        time.sleep(duration)

        try:
            while self.socket.recv(4096):
                pass
        except socket.timeout:
            pass


    def get_rx_bytes(self):
        """ Get the number of bytes read from the stream. """
        return self.total_bytes


    def read_frame(self):
        """ Read a single frame from the stream.

        Returns:
            Yields the (seqnum, data) of the batches available in the frame.
        """
        buf = self.socket.recv(4096)
        self.total_bytes += len(buf)

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
