#!/usr/bin/python3
"""
Authors:
    Étienne Wodey, Leibniz University Hannover, Institute of Quantum Optics
    Ryan Summers, Vertigo Designs
    Robert Jördens, QUARTIQ

Description: Algorithms to generate biquad (second order IIR) coefficients.
"""
import argparse
import asyncio
import collections
import logging

from math import pi, inf

import miniconf

import stabilizer

logger = logging.getLogger(__name__)

# Disable pylint warnings about a0, b1 etc
#pylint: disable=invalid-name


# Generic type for containing a command-line argument.
# Use `add_argument` for simple construction.
Argument = collections.namedtuple("Argument", ["positionals", "keywords"])


def add_argument(*args, **kwargs):
    """ Convert arguments into an Argument tuple. """
    return Argument(args, kwargs)


# Represents a generic filter that can be implemented by a biquad.
#
# Fields
#     * `help`: This field specifies helpful human-readable information that
#       will be presented to users on the command line.
#     * `arguments`: A list of `Argument` objects representing available
#       command-line arguments for the filter. Use the `add_argument()`
#       function to easily parse options as they would be provided to
#       argparse.
#     * `coefficients`: A function, provided with parsed arguments, that
#       returns the IIR coefficients. See below for more information on this
#       function.
#
# # Coefficients Calculation Function
#     Description:
#       This function takes in two input arguments and returns the IIR filter
#       coefficients for Stabilizer to represent the necessary filter.
#
#     Args:
#       args: The filter command-line arguments. Any filter-related arguments
#       may be accessed via their name. E.g. `args.K`.
#
#     Returns:
#       [b0, b1, b2, -a1, -a2] IIR coefficients to be programmed into a
#       Stabilizer IIR filter configuration.
Filter = collections.namedtuple(
    "Filter", ["help", "arguments", "coefficients"])


def get_filters():
    """ Get a dictionary of all available filters.

    Note:
        Calculations coefficient largely taken using the derivations in
        page 9 of https://arxiv.org/pdf/1508.06319.pdf

        PII/PID coefficient equations are taken from the PID-IIR primer
        written by Robert Jördens at https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw
    """
    return {
        "lowpass": Filter(help="Gain-limited low-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Lowpass filter gain"),
                          ],
                          coefficients=lowpass_coefficients),
        "highpass": Filter(help="Gain-limited high-pass filter",
                           arguments=[
                               add_argument("--f0", required=True, type=float,
                                            help="Corner frequency (Hz)"),
                               add_argument("--K", required=True, type=float,
                                            help="Highpass filter gain"),
                           ],
                           coefficients=highpass_coefficients),
        "allpass": Filter(help="Gain-limited all-pass filter",
                          arguments=[
                              add_argument("--f0", required=True, type=float,
                                           help="Corner frequency (Hz)"),
                              add_argument("--K", required=True, type=float,
                                           help="Allpass filter gain"),
                          ],
                          coefficients=allpass_coefficients),
        "notch": Filter(help="Notch filter",
                        arguments=[
                            add_argument("--f0", required=True, type=float,
                                         help="Corner frequency (Hz)"),
                            add_argument("--Q", required=True, type=float,
                                         help="Filter quality factor"),
                            add_argument("--K", required=True, type=float,
                                         help="Filter gain"),
                        ],
                        coefficients=notch_coefficients),
        "pid": Filter(help="PID controller. Gains at 1 Hz and often negative.",
                      arguments=[
                          add_argument("--Kii", default=0, type=float,
                                       help="Double Integrator (I^2) gain"),
                          add_argument("--Kii_limit", default=inf, type=float,
                                       help="Integral gain limit"),
                          add_argument("--Ki", default=0, type=float,
                                       help="Integrator (I) gain"),
                          add_argument("--Ki_limit", default=inf, type=float,
                                       help="Integral gain limit"),
                          add_argument("--Kp", default=0, type=float,
                                       help="Proportional (P) gain"),
                          add_argument("--Kd", default=0, type=float,
                                       help="Derivative (D) gain"),
                          add_argument("--Kd_limit", default=inf, type=float,
                                       help="Derivative gain limit"),
                          add_argument("--Kdd", default=0, type=float,
                                       help="Double Derivative (D^2) gain"),
                          add_argument("--Kdd_limit", default=inf, type=float,
                                       help="Derivative gain limit"),
                      ],
                      coefficients=pid_coefficients),
    }


def lowpass_coefficients(args):
    """Calculate low-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = args.K * f0_bar / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def highpass_coefficients(args):
    """Calculate high-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)
    b0 = args.K * (f0_bar / (1 + f0_bar))
    b1 = - args.K / (1 + f0_bar)

    return [b0, b1, 0, a1, 0]


def allpass_coefficients(args):
    """Calculate all-pass IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    a1 = (1 - f0_bar) / (1 + f0_bar)

    b0 = args.K * (1 - f0_bar) / (1 + f0_bar)
    b1 = - args.K

    return [b0, b1, 0, a1, 0]


def notch_coefficients(args):
    """Calculate notch IIR filter coefficients."""
    f0_bar = pi * args.f0 * args.sample_period

    denominator = (1 + f0_bar / args.Q + f0_bar ** 2)

    a1 = 2 * (1 - f0_bar ** 2) / denominator
    a2 = - (1 - f0_bar / args.Q + f0_bar ** 2) / denominator
    b0 = args.K * (1 + f0_bar ** 2) / denominator
    b1 = - (2 * args.K * (1 - f0_bar ** 2)) / denominator
    b2 = args.K * (1 + f0_bar ** 2) / denominator

    return [b0, b1, b2, a1, a2]


def pid_coefficients(args):
    """Calculate PID IIR filter coefficients."""

    # Determine filter order
    if args.Kii != 0:
        assert (args.Kdd, args.Kd, args.Kdd_limit, args.Kd_limit) == \
        (0, 0, float('inf'), float('inf')), \
            "IIR filters I^2 and D or D^2 gain/limit are unsupported"
        order = 2
    elif args.Ki != 0:
        assert (args.Kdd, args.Kdd_limit) == (0, float('inf')), \
            "IIR filters with I and D^2 gain/limit are unsupported"
        order = 1
    else:
        order = 0

    kernels = [
        [1, 0, 0],
        [1, -1, 0],
        [1, -2, 1]
    ]

    gains = [args.Kii, args.Ki, args.Kp, args.Kd, args.Kdd]
    limits = [args.Kii/args.Kii_limit, args.Ki/args.Ki_limit,
              1, args.Kd/args.Kd_limit, args.Kdd/args.Kdd_limit]
    w = 2*pi*args.sample_period
    b = [sum(gains[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]

    a = [sum(limits[2 - order + i] * w**(order - i) * kernels[i][j]
             for i in range(3)) for j in range(3)]
    b = [i/a[0] for i in b]
    a = [i/a[0] for i in a]
    assert a[0] == 1
    return b + [-ai for ai in a[1:]]


def _main():
    parser = argparse.ArgumentParser(
        description="Configure Stabilizer dual-iir filter parameters."
                    "Note: This script assumes an AFE input gain of 1.")
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='Increase logging verbosity')
    parser.add_argument("--broker", "-b", type=str, default="mqtt",
                        help="The MQTT broker to use to communicate with "
                        "Stabilizer (%(default)s)")
    parser.add_argument("--prefix", "-p", type=str,
                        default="dt/sinara/dual-iir/+",
                        help="The Stabilizer device prefix in MQTT, "
                        "wildcards allowed as long as the match is unique "
                        "(%(default)s)")
    parser.add_argument("--no-discover", "-d", action="store_true",
                        help="Do not discover Stabilizer device prefix.")

    parser.add_argument("--channel", "-c", type=int, choices=[0, 1],
                        required=True, help="The filter channel to configure.")
    parser.add_argument("--sample-period", type=float,
                        default=stabilizer.SAMPLE_PERIOD,
                        help="Sample period in seconds (%(default)s s)")

    parser.add_argument("--x-offset", type=float, default=0,
                        help="The channel input offset (%(default)s V)")
    parser.add_argument("--y-min", type=float,
                        default=-stabilizer.DAC_FULL_SCALE,
                        help="The channel minimum output (%(default)s V)")
    parser.add_argument("--y-max", type=float,
                        default=stabilizer.DAC_FULL_SCALE,
                        help="The channel maximum output (%(default)s V)")
    parser.add_argument("--y-offset", type=float, default=0,
                        help="The channel output offset (%(default)s V)")

    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(
        help="Filter-specific design parameters", dest="filter_type",
        required=True)

    filters = get_filters()

    for (filter_name, filt) in filters.items():
        subparser = subparsers.add_parser(filter_name, help=filt.help)
        for arg in filt.arguments:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        level=logging.WARN - 10*args.verbose)

    # Calculate the IIR coefficients for the filter.
    coefficients = filters[args.filter_type].coefficients(args)

    # The feed-forward gain of the IIR filter is the summation
    # of the "b" components of the filter.
    forward_gain = sum(coefficients[:3])
    if forward_gain == 0 and args.x_offset != 0:
        logger.warning("Filter has no DC gain but x_offset is non-zero")

    if args.no_discover:
        prefix = args.prefix
    else:
        devices = asyncio.run(miniconf.discover(args.broker, args.prefix))
        if not devices:
            raise ValueError("No prefixes discovered.")
        if len(devices) > 1:
            raise ValueError(f"Multiple prefixes discovered ({devices})."
                             "Please specify a more specific --prefix")
        prefix = devices[0]
        logger.info("Automatically using detected device prefix: %s", prefix)

    async def configure():
        logger.info("Connecting to broker")
        interface = await miniconf.Miniconf.create(prefix, args.broker)

        # Set the filter coefficients.
        # Note: In the future, we will need to Handle higher-order cascades.
        await interface.command(f"iir_ch/{args.channel}/0", {
            "ba": coefficients,
            "y_min": stabilizer.voltage_to_machine_units(args.y_min),
            "y_max": stabilizer.voltage_to_machine_units(args.y_max),
            "y_offset": stabilizer.voltage_to_machine_units(
                args.y_offset + forward_gain * args.x_offset)
        })

    asyncio.run(configure())


if __name__ == "__main__":
    _main()
