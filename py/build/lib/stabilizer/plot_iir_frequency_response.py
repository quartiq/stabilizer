#!/usr/bin/python3
"""
Small tool to show the frequency response of biquad IIR filters.
"""
import argparse
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

import stabilizer
from stabilizer.iir_coefficients import get_filters

# disable warnings about short variable names and similar code
#pylint: disable=invalid-name, duplicate-code, redefined-builtin



def _main():
    parser = argparse.ArgumentParser(
        description="Plot frequency response for filter parameters"
    )
    parser.add_argument(
        "--sample-period",
        "-s",
        type=float,
        default=stabilizer.SAMPLE_PERIOD,
        help="Sample period in seconds (%(default)s s)",
    )

    # Next, add subparsers and their arguments.
    subparsers = parser.add_subparsers(
        help="Filter-specific design parameters", dest="filter_type", required=True
    )

    filters = get_filters()

    for name, filter in get_filters().items():
        subparser = subparsers.add_parser(name, help=filter.help)
        for arg in filter.arguments:
            subparser.add_argument(*arg.positionals, **arg.keywords)

    args = parser.parse_args()

    # Calculate the IIR coefficients for the filter.
    coefficients = filters[args.filter_type].coefficients(args)

    print(coefficients)

    # The feed-forward gain of the IIR filter is the summation
    # of the "b" components of the filter.
    forward_gain = sum(coefficients[:3])
    if forward_gain == 0 and args.x_offset != 0:
        print("Filter has no DC gain but x_offset is non-zero")

    f = np.logspace(-8.5, 0, 1024, endpoint=False)*(.5/args.sample_period)
    f, h = signal.freqz(
        coefficients[:3],
        np.r_[1, [-c for c in coefficients[3:]]],
        worN=f,
        fs=1 / args.sample_period,
    )
    _, ax = plt.subplots()
    ax.plot(f, 20 * np.log10(np.absolute(h)))
    ax.set_xscale("log")
    ax.grid()
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title("Filter response")
    plt.show()


if __name__ == "__main__":
    _main()
