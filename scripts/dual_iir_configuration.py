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

from miniconf import Miniconf

import stabilizer

#pylint: disable=invalid-name

# The base Stabilizer tick rate in Hz.
STABILIZER_TICK_RATE = 100e6

# Generic type for containing a command-line argument. Use `add_argument` for simple construction.
Argument = collections.namedtuple('Argument', ['positionals', 'keywords'])

""" Represents a generic filter that can be represented by an IIR filter.

# Fields
    * `help`: This field specifies helpful human-readable information that will be presented to
        users on the command line.
    * `arguments`: A list of `Argument` objects representing available command-line arguments for
    the filter. Use the `add_argument()` function to easily parse options as they would be provided
        to argparse.
    * `calculate_coefficients`: A function, provided two argumetns, that returns the IIR
    coefficients. See below for more information on this function.

# Coefficients Calculation Function

    Description:
        This function takes in two input arguments and returns the IIR filter coefficients for
        Stabilizer to represent the necessary filter.

    Args:
        sampling_period: The period between discrete samples of the input signal.
        args: The filter command-line arguments. Any filter-related arguments may be accessed via
        their name. E.g. `args.K`.

    Returns:
        [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a Stabilizer IIR filter
        configuration.
"""
Filter = collections.namedtuple('Filter', ['help', 'arguments', 'calculate_coefficients'])

def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


def get_filters():
    """ Get a dictionary of all available filters.

    Note:
        Calculations coefficient largely taken using the derivations in page 9 of
        https://arxiv.org/pdf/1508.06319.pdf

        PII/PID coefficient equations are taken from the PID-IIR primer written by Robert Jördens at
        https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw
    """
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
                          add_argument('--Kp', default=0, type=float,
                                       help='Proportional (P) gain at 1 Hz'),
                          add_argument('--Ki', default=0, type=float,
                                       help='Integral (I) gain at 1 Hz'),
                          add_argument('--Kii', default=0, type=float,
                                       help='Integral Squared (I^2) gain at 1 Hz'),
                          add_argument('--Kd', default=0, type=float,
                                       help='Derivative (D) gain at 1 Hz'),
                          add_argument('--Kdd', default=0, type=float,
                                       help='Derivative Squared (D^2) gain at 1 Hz'),
                      ],
                      calculate_coefficients=calculate_pid_coefficients),
    }


def calculate_lowpass_coefficients(sampling_period, args):
    """ Calculate low-pass IIR filter coefficients. """
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = args.K * f0_bar / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def calculate_highpass_coefficients(sampling_period, args):
    """ Calculate high-pass IIR filter coefficients. """
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = - args.K / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def calculate_allpass_coefficients(sampling_period, args):
    """ Calculate all-pass IIR filter coefficients. """
    f0_bar = pi * args.f0 * sampling_period

    a1 = (1 - f0_bar) / (1 + f0_bar)

    b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
    b1 = - args.K

    return [b0, b1, 0, a1, 0]


def calculate_notch_coefficients(sampling_period, args):
    """ Calculate notch IIR filter coefficients. """
    f0_bar = pi * args.f0 * sampling_period

    denominator = (1 + f0_bar / args.Q + f0_bar ** 2)

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
    b0 = args.K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
    b2 = args.K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]


def calculate_pid_coefficients(sampling_period, args):
    """ Calculate PID IIR filter coefficients. """

    # First, determine the lowest feed-back rank we can use.
    if args.Kii != 0:
        assert args.Kdd == 0, 'IIR filters with both I^2 and D^2 coefficients are unsupported'
        assert args.Kd == 0, 'IIR filters with both I^2 and D coefficients are unsupported'
        feedback_kernel = 2
    elif args.Ki != 0:
        assert args.Kdd == 0, 'IIR filters with both I and D^2 coefficients are unsupported'
        feedback_kernel = 1
    else:
        feedback_kernel = 0

    KERNELS = [
        [1, 0, 0],
        [1, -1, 0],
        [1, -2, 1]
    ]

    digital_conversion = (2 * pi) / sampling_period

    FEEDFORWARD_COEFFICIENTS = [
        [args.Kp, args.Kd / digital_conversion, args.Kdd / (digital_conversion ** 2)],
        [args.Ki * digital_conversion, args.Kp, args.Kd / digital_conversion],
        [args.Kii * (digital_conversion ** 2), args.Ki * digital_conversion, args.Kp],
    ]

    # We now select the type of kernel using the rank of the feedback rank.
    # a-coefficients are defined purely by the feedback kernel.
    a_coefficients = KERNELS[feedback_kernel]

    b_coefficients = [0, 0, 0]
    for (kernel, gain) in zip(KERNELS, FEEDFORWARD_COEFFICIENTS[feedback_kernel]):
        for index, kernel_value in enumerate(kernel):
            b_coefficients[index] += kernel_value * gain

    # Note: Normalization is redundant because a0 is defined to be 1 in all cases. Because of this,
    # normalization is skipped.
    b_norm = b_coefficients
    a_norm = a_coefficients

    return [b_norm[0], b_norm[1], b_norm[2], -1 * a_norm[1], -1 * a_norm[2]]


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(
        description='Configure Stabilizer dual-iir filter parameters. Note: This script assumes '
                    'an AFE input gain of 1.')
    parser.add_argument('--broker', '-b', type=str, default='mqtt',
                        help='The MQTT broker to use to communicate with Stabilizer')
    parser.add_argument('--prefix', '-p', type=str, required=True,
                        help='The Stabilizer device prefix to use for communication. E.g. '
                        'dt/sinara/dual-iir/00-11-22-33-44-55')
    parser.add_argument('--channel', '-c', type=int, choices=[0, 1], required=True,
                        help='The filter channel to configure.')
    parser.add_argument('--sample-ticks', type=int, default=128,
                        help='The number of Stabilizer hardware ticks between each sample')

    parser.add_argument('--x-offset', type=float, default=0,
                        help='The channel input offset level (Volts)')
    parser.add_argument('--y-min', type=float, default=-stabilizer.DAC_FULL_SCALE,
                        help='The channel minimum output level (Volts)')
    parser.add_argument('--y-max', type=float, default=stabilizer.DAC_FULL_SCALE,
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

    # The feed-forward gain of the IIR filter is equivalent to the summation of the 'b' components
    # of the filter.
    forward_gain = sum(coefficients[:3])

    async def configure():
        logging.info('Connecting to broker')
        interface = await Miniconf.create(args.prefix, args.broker)

        # Set the filter coefficients.
        # Note: In the future, we will need to Handle higher-order cascades.
        await interface.command(f'iir_ch/{args.channel}/0', {
            'ba': coefficients,
            'y_min': stabilizer.voltage_to_machine_units(args.y_min),
            'y_max': stabilizer.voltage_to_machine_units(args.y_max),
            'y_offset': stabilizer.voltage_to_machine_units(
                args.y_offset + forward_gain * args.x_offset)
        })

    asyncio.run(configure())


if __name__ == '__main__':
    main()
