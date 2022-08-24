#!/usr/bin/python3
"""
Small tool to show the frequency response of biquad IIR filters.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import argparse

from iir_coefficients import get_filters
import stabilizer


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

    for (filter_name, filt) in filters.items():
        subparser = subparsers.add_parser(filter_name, help=filt.help)
        for arg in filt.arguments:
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

    f = np.logspace(-7, np.log10(0.5 / args.sample_period), 1024, endpoint=False)
    f, h = signal.freqz(
        coefficients[:3],
        [1] + [-c for c in coefficients[3:]],
        worN=f,
        fs=1 / args.sample_period,
    )
    fig, ax = plt.subplots()
    ax.margins(0, 0.1)
    ax.plot(f, 20 * np.log10(abs(h)))
    ax.set_xscale("log")
    ax.grid()
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title("Filter response")
    plt.show()


if __name__ == "__main__":
    _main()
