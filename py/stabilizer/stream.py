#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides a means of accessing Stabilizer livestream data.
"""
import asyncio
import collections
import copy
import logging
import select
import socket
import struct

from typing import Mapping, ByteString, List, Callable, Union

import numpy as np

from . import DAC_VOLTS_PER_LSB, ADC_VOLTS_PER_LSB

StreamFrame = collections.namedtuple('StreamFrame', ['sequence_number', 'format', 'traces'])
"""
A single frame received over the livestream interface.

# Args
* `sequence_number` - The starting sample sequence number.
* `format` - The format code of the frame.
* `traces` - A dictionary of traces, each entry of which contain the raw data associated with the
             trace.
"""

# The minimum value of a 16-bit integer.
INT16_MIN = np.iinfo(np.int16).min


def format_adc_dac_data(buf: ByteString, batch_size: int):
    """ Format ADC/DAC data into buffers. """
    frame = np.frombuffer(buf, '<i2')

    frame = frame.reshape(-1, 4, batch_size).swapaxes(0, 1).reshape(4, -1)

    return {
        # ADC codes can be directly converted to voltages because they are natively represented by a
        # i16 that ranges from MIN to MAX.
        'ADC0': frame[0].astype(float) * ADC_VOLTS_PER_LSB,
        'ADC1': frame[1].astype(float) * ADC_VOLTS_PER_LSB,

        # DAC codes are slightly more complicated and require an offset of the initial code to align
        # it with a i16 type. After that, we can convert them to voltages.
        'DAC0': (frame[2] + INT16_MIN).astype(float) * DAC_VOLTS_PER_LSB,
        'DAC1': (frame[3] + INT16_MIN).astype(float) * DAC_VOLTS_PER_LSB,
    }


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
    DEFAULT_FORMATS = {
        1: format_adc_dac_data,
    }

    def __init__(self, port, timeout: int = None):
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

        self._formatters = copy.deepcopy(self.__class__.DEFAULT_FORMATS)


    def clear(self):
        """ Clear the socket RX buffer by reading all available data. """
        while self.frame_available():
            self.read_frame()


    def add_formatter(self, code: int,
                      factory: Callable[[ByteString, int], Mapping[str, List[float]]]):
        """ Add a formatter for recognizing a specific stream format. """
        assert code not in self._formatters, f'Formatter for {code} already exists'
        self._formatters[code] = factory


    def get_rx_bytes(self) -> int:
        """ Get the number of bytes read from the stream. """
        return self.total_bytes


    def read_frame(self) -> Union[StreamFrame, None]:
        """ Read a single frame from the stream.

        Returns:
            Yields the (seqnum, data) of the batches available in the frame.
        """
        buf = self.socket.recv(4096)
        self.total_bytes += len(buf)

        # Attempt to parse a block from the buffer.
        if len(buf) < struct.calcsize(self.HEADER_FORMAT):
            return None

        # Parse out the packet header
        magic, format_id, batch_size, sequence_number = struct.unpack_from(self.HEADER_FORMAT, buf)

        buf = buf[struct.calcsize(self.HEADER_FORMAT):]

        if magic != self.MAGIC_HEADER:
            logging.warning('Encountered bad magic header: %s', hex(magic))
            return None

        traces = self._formatters[format_id](buf, batch_size)

        return StreamFrame(sequence_number * batch_size, format_id, traces)


    def frame_available(self) -> bool:
        """ Check if a frame is available. """
        readable, _, _ = select.select([self.socket], [], [], 0)
        return len(readable) > 0


    async def next_frame(self) -> StreamFrame:
        """ Get the next frame asynchronously. """
        while True:
            while not self.frame_available():
                await asyncio.sleep(0)

            frame = self.read_frame()
            if frame:
                return frame
