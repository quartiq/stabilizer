#!/usr/bin/python3
"""
Author: Leibniz University Hannover, Institute of Quantum Optics, Étienne Wodey
        Vertigo Designs, Ryan Summers

Description: Provides a mechanism to configure dual-iir IIR filters using a high-level API.
"""
import abc
import argparse
import asyncio
import collections
import logging

from math import pi

from miniconf import Miniconf

#pylint: disable=invalid-name

# The base Stabilizer tick rate in Hz.
STABILIZER_TICK_RATE = 100e6

# The maximum output scale of the Stabilizer DACs.
DAC_MAX_SCALE = 4.096 * 2.5


Filter = collections.namedtuple('Filter', ['help', 'arguments', 'handler'])
Argument = collections.namedtuple('Argument', ['positionals', 'keywords'])

def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


def _voltage_to_machine_units(voltage):
    """ Convert a voltage to IIR machine units.

    Args:
        voltage: The voltage to convert

    Returns:
        The IIR machine-units associated with the voltage.
    """
    assert abs(voltage) <= DAC_MAX_SCALE, 'Voltage out-of-range'
    return int(voltage / DAC_MAX_SCALE * 0x7FFF)


class Filter(abc.ABC):

    def name(self):
        return self.__class__.__name__

    @property
    @abc.abstractmethod
    def DESCRIPTION(self):
        """ Description of the filter. """


    @property
    @abc.abstractmethod
    def ARGUMENTS(self):
        """ A list of command-line arguments for the filter. """

    @classmethod
    @abc.abstractmethod
    def calculate_coefficients(cls, sampling_period, args):
        """ Calculate IIR filter coefficients.

        Note:
            Calculations largely taken using the derivations in page 9 of
            https://arxiv.org/pdf/1508.06319.pdf

        Args:
            sampling_period: The period between discrete samples of the input signal.
            args: The filter command-line arguments.

        Returns:
            [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a Stabilizer IIR filter
            configuration.
        """


class Lowpass(Filter):
    DESCRIPTION = "Gain-limited low-pass filter"

    ARGUMENTS = [
        add_argument('--f0', required=True, type=float, help='Corner frequency (Hz)'),
        add_argument('--K', required=True, type=float, help='Lowpass filter gain'),
    ]

    @classmethod
    def calculate_coefficients(cls, sampling_period, args):
        f0_bar = pi * args.f0 * sampling_period

        a1 = (1 - f0_bar) / (1 + f0_bar)
        b0 = args.K * (f0_bar / (1 + f0_bar))
        b1 = args.K * f0_bar / (1 + f0_bar)

        return [b0, b1, 0, a1, 0]


class Highpass(Filter):
    DESCRIPTION = "Gain-limited high-pass filter"

    ARGUMENTS = [
        add_argument('--f0', required=True, type=float, help='Corner frequency (Hz)'),
        add_argument('--K', required=True, type=float, help='Highpass filter gain'),
    ]

    @classmethod
    def calculate_coefficients(cls, sampling_period, args):
        f0_bar = pi * args.f0 * sampling_period

        a1 = (1 - f0_bar) / (1 + f0_bar)
        b0 = args.K * (f0_bar / (1 + f0_bar))
        b1 = - args.K / (1 + f0_bar)

        return [b0, b1, 0, a1, 0]


class Allpass(Filter):
    DESCRIPTION = "Gain-limited all-pass filter"

    ARGUMENTS = [
        add_argument('--f0', required=True, type=float, help='Corner frequency (Hz)'),
        add_argument('--K', required=True, type=float, help='Highpass filter gain '),
    ]

    @classmethod
    def calculate_coefficients(cls, sampling_period, args):
        f0_bar = pi * args.f0 * sampling_period

        a1 = (1 - f0_bar) / (1 + f0_bar)

        b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
        b1 = - args.K

        return [b0, b1, 0, a1, 0]


class Notch(Filter):
    DESCRIPTION = "Notch filter"

    ARGUMENTS = [
        add_argument('--f0', required=True, type=float, help='Corner frequency (Hz)'),
        add_argument('--Q', required=True, type=float, help='Filter quality factor'),
        add_argument('--K', required=True, type=float, help='Filter gain'),
    ]

    @classmethod
    def calculate_coefficients(cls, sampling_period, args):
        f0_bar = pi * args.f0 * sampling_period

        denominator = (1 + f0_bar / args.Q + f0_bar ** 2)

        a1 = 2 * (1 - f0_bar ** 2) / denominator
        a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
        b0 = args.K * (1 + f0_bar ** 2) / denominator
        b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
        b2 = args.K * (1 + f0_bar ** 2) / denominator

        return [b0, b1, b2, a1, a2]


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Configure Stabilizer dual-iir filter parameters')
    parser.add_argument('--broker', '-b', type=str, default='mqtt',
                        help='The MQTT broker to use to communicate with Stabilizer')
    parser.add_argument('--prefix', '-p', type=str, required=True,
                        help='The Stabilizer device prefix to use for communication. E.g. '
                        'dt/sinara/dual-iir/00-11-22-33-44-55')
    parser.add_argument('--channel', '-c', type=int, choices=[0, 1], required=True,
                        help='The filter channel to configure.')
    parser.add_argument('--sample-ticks', type=int, default=128,
                        help='The number of Stabilizer hardware ticks between each sample')

    parser.add_argument('--y-min', type=float, default=-DAC_MAX_SCALE,
                        help='The channel minimum output level (Volts)')
    parser.add_argument('--y-max', type=float, default=DAC_MAX_SCALE,
                        help='The channel maximum output level (Volts)')
    parser.add_argument('--y-offset', type=float, default=0,
                        help='The channel output offset level (Volts)')


    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(help='Filter-specific design parameters', dest='filter_type')

    FILTERS = {
        'lowpass': Lowpass,
        'highpass': Highpass,
        'allpass': Allpass,
        'notch': Notch
    }

    for (filter_name, filt) in FILTERS.items():
        subparser = subparsers.add_parser(filter_name, help=filt.DESCRIPTION)
        for arg in filt.ARGUMENTS:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    # Calculate the IIR coefficients for the filter.
    sampling_period = args.sample_ticks / STABILIZER_TICK_RATE
    coefficients = FILTERS[args.filter_type].calculate_coefficients(sampling_period, args)

    async def configure():
        logging.info('Connecting to broker')
        interface = await Miniconf.create(args.prefix, args.broker)

        # Set the filter coefficients.
        # TODO: Handle higher-order cascades.
        await interface.command(f'iir_ch/{args.channel}/0', {
            'ba': coefficients,
            'y_min': _voltage_to_machine_units(args.y_min),
            'y_max': _voltage_to_machine_units(args.y_max),
            'y_offset': _voltage_to_machine_units(args.y_offset)
        })

    asyncio.run(configure())


if __name__ == '__main__':
    main()
