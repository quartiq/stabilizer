#!/usr/bin/python3
"""
Author: Leibniz University Hannover, Institute of Quantum Optics, Étienne Wodey
        Vertigo Designs, Ryan Summers

Description: Provides a mechanism to configure dual-iir IIR filters using a high-level API.
"""
import argparse
import asyncio
import collections
import logging

from math import pi

import stabilizer
from miniconf import Miniconf

#pylint: disable=invalid-name

# The base Stabilizer tick rate in Hz.
STABILIZER_TICK_RATE = 100e6


Filter = collections.namedtuple('Filter', ['help', 'arguments', 'calculate_coefficients'])
Argument = collections.namedtuple('Argument', ['positionals', 'keywords'])

def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


def get_filters():
    """ Get a dictionary of all available filters. """
    return {
        'lowpass': Filter(help='Gain-limited low-pass filter',
                          arguments=[
                              add_argument('--f0', required=True, type=float,
                                           help='Corner frequency (Hz)'),
                              add_argument('--K', required=True, type=float,
                                           help='Lowpass filter gain'),
                          ],
                          calculate_coefficients=calculate_lowpass_coefficients),
        'highpass': Filter(help='Gain-limited high-pass filter',
                           arguments=[
                               add_argument('--f0', required=True, type=float,
                                            help='Corner frequency (Hz)'),
                               add_argument('--K', required=True, type=float,
                                            help='Highpass filter gain'),
                           ],
                           calculate_coefficients=calculate_highpass_coefficients),
        'allpass': Filter(help='Gain-limited all-pass filter',
                          arguments=[
                              add_argument('--f0', required=True, type=float,
                                           help='Corner frequency (Hz)'),
                              add_argument('--K', required=True, type=float,
                                           help='Allpass filter gain'),
                          ],
                          calculate_coefficients=calculate_allpass_coefficients),
        'notch': Filter(help='Notch filter',
                        arguments=[
                            add_argument('--f0', required=True, type=float,
                                         help='Corner frequency (Hz)'),
                            add_argument('--Q', required=True, type=float,
                                         help='Filter quality factor'),
                            add_argument('--K', required=True, type=float,
                                         help='Filter gain'),
                        ],
                        calculate_coefficients=calculate_notch_coefficients),
        'pid': Filter(help='PID controller',
                      arguments=[
                          add_argument('--Kp', required=True, type=float,
                                       help='Proportional (P) gain at 1 Hz'),
                          add_argument('--Ki', required=True, type=float,
                                       help='Integral (I) gain at 1 Hz'),
                          add_argument('--Kd', default=0, type=float,
                                       help='Derivative (D) gain at 1 Hz'),
                      ],
                      calculate_coefficients=calculate_pid_coefficients),
        'pii': Filter(help='PII controller',
                      arguments=[
                          add_argument('--Kp', required=True, type=float,
                                       help='Proportional (P) gain at 1 Hz'),
                          add_argument('--Ki', required=True, type=float,
                                       help='Integral (I) gain at 1 Hz'),
                          add_argument('--Kii', required=True, type=float,
                                       help='Integral Squared (I^2) gain at 1 Hz'),
                      ],
                      calculate_coefficients=calculate_pii_coefficients),
    }


def calculate_lowpass_coefficients(sampling_period, args):
    """ Calculate low-pass IIR filter coefficients.

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
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = args.K * f0_bar / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def calculate_highpass_coefficients(sampling_period, args):
    """ Calculate high-pass IIR filter coefficients.

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
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = - args.K / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def calculate_allpass_coefficients(sampling_period, args):
    """ Calculate all-pass IIR filter coefficients.

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
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)

    b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
    b1 = - args.K

    return [b0, b1, 0, a1, 0]


def calculate_notch_coefficients(sampling_period, args):
    """ Calculate notch IIR filter coefficients.

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
    f0_bar = pi * args.f0 * sampling_period

    denominator = (1 + f0_bar / args.Q + f0_bar ** 2)

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
    b0 = args.K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
    b2 = args.K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]


def calculate_pid_coefficients(sampling_period, args):
    """ Calculate PID IIR filter coefficients.

    # Note
        These equations are taken from the PID-IIR primer written by Robert Jördens at
        https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw

    Args:
        sampling_period: The period between discrete samples of the input signal.
        args: The filter command-line arguments.

    Returns:
        [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a Stabilizer IIR filter
        configuration.
    """

    k_p = args.Kp
    k_i = args.Ki * 2 * pi * sampling_period
    k_d = args.Kd / (2 * pi) / sampling_period

    b0 = k_p + k_i + k_d
    b1 = - (k_p + 2 * k_d)
    b2 = k_d

    # For PID controllers, a1 and a2 are always fixed.
    return [b0, b1, b2, 1, 0]


def calculate_pii_coefficients(sampling_period, args):
    """ Calculate PII IIR filter coefficients.

    # Note
        These equations are taken from the PID-IIR primer written by Robert Jördens at
        https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw

    Args:
        sampling_period: The period between discrete samples of the input signal.
        args: The filter command-line arguments.

    Returns:
        [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a Stabilizer IIR filter
        configuration.
    """
    # a^2 * y = (Ki2 b^0 + Ki b^1 + Kp b^2) * x
    # y0 -2y1 + y2 = ((Ki2, 0, 0) + (Ki, -Ki, 0) + (Kp, -2Kp, Kp)) * x
    # (1)y0 + (-2)y1 + (1)y2 = (Ki2) x0 + (Ki) x0 + (-Ki) x1 + (Kp) x0 + (-2Kp) x1 + (Kp) x2
    # (1)y0 + (-2)y1 + (1)y2 = (Ki2 + Ki + Kp) x0 + (-Ki + -2Kp) x1 + (Kp) x2
    k_p = args.Kp
    k_i = args.Ki * 2 * pi * sampling_period
    k_i2 = args.Kii * (2 * pi * sampling_period) ** 2

    b0 = k_i2 + k_i + k_p
    b1 = - (k_i + 2 * k_p)
    b2 = k_p

    # For PII controllers, a1 and a2 are always fixed.
    return [b0, b1, b2, 2, -1]


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

    parser.add_argument('--y-min', type=float, default=-stabilizer.DAC_MAX_SCALE,
                        help='The channel minimum output level (Volts)')
    parser.add_argument('--y-max', type=float, default=stabilizer.DAC_MAX_SCALE,
                        help='The channel maximum output level (Volts)')
    parser.add_argument('--y-offset', type=float, default=0,
                        help='The channel output offset level (Volts)')


    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(help='Filter-specific design parameters', dest='filter_type')

    filters = get_filters()

    for (filter_name, filt) in filters.items():
        subparser = subparsers.add_parser(filter_name, help=filt.help)
        for arg in filt.arguments:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    # Calculate the IIR coefficients for the filter.
    sampling_period = args.sample_ticks / STABILIZER_TICK_RATE
    coefficients = filters[args.filter_type].calculate_coefficients(sampling_period, args)

    async def configure():
        logging.info('Connecting to broker')
        interface = await Miniconf.create(args.prefix, args.broker)

        # Set the filter coefficients.
        # Note: In the future, we will need to Handle higher-order cascades.
        await interface.command(f'iir_ch/{args.channel}/0', {
            'ba': coefficients,
            'y_min': stabilizer.voltage_to_machine_units(args.y_min),
            'y_max': stabilizer.voltage_to_machine_units(args.y_max),
            'y_offset': stabilizer.voltage_to_machine_units(args.y_offset)
        })

    asyncio.run(configure())


if __name__ == '__main__':
    main()
