#!/usr/bin/python3
"""
Author: Ryan Summers

Description: Provides a mechanism for measuring Stabilizer stream data throughput.
"""
import argparse
import logging
import sys
import time

from stabilizer.stream import StabilizerStream

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
    parser.add_argument('--port', type=int, default=2000,
                        help='The port that stabilizer is streaming to')

    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s.%(msecs)03d %(levelname)-8s %(message)s')

    last_index = None

    drop_count = 0
    good_blocks = 0
    total_bytes = 0

    timer = Timer()

    stream = StabilizerStream(args.port)

    while True:
        # Receive any data over UDP and parse it.
        for (seqnum, _) in stream.read_frame():
            if not timer.is_started():
                timer.start()

            # Handle any dropped packets.
            drop_count += sequence_delta(last_index, seqnum)
            last_index = seqnum
            good_blocks += 1

        # Report the throughput periodically.
        if timer.is_triggered():
            drate = stream.get_rx_bytes() * 8 / 1e6 / timer.elapsed()

            print(f'''
Data Rate:       {drate:.3f} Mbps
Received Blocks: {good_blocks}
Dropped blocks:  {drop_count}

Metadata: {total_bytes / 1e6:.3f} MB in {timer.elapsed():.2f} s
----
''')
            sys.stdout.flush()
            timer.arm()


if __name__ == '__main__':
    main()
