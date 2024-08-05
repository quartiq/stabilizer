#!/usr/bin/python3
"""
Class to implement a biquad IIR filter in software.

Author:  Giacomo  Bisson
"""

import numpy as np
import argparse
import stabilizer
import stabilizer.iir_coefficients as iir_coefficients


class IirBiquadFilter:
    """
    Class to implement a biquad IIR filter in software.
    """

    def __init__(self, filter_type, f0, K=1, Q=1):
        """
        Initialize the IIR biquad filter.

        :param filter_type: The type of filter to use.
        """

        class FilterArgs:
            def __init__(self, f0=f0, K=K, Q=Q, sample_period=stabilizer.SAMPLE_PERIOD):
                self.f0 = f0
                self.K = K
                self.Q = Q
                self.sample_period = sample_period

        filter_args = FilterArgs()

        self.filters = iir_coefficients.get_filters()
        if filter_type not in self.filters:
            raise ValueError(f"Unknown filter type: {filter_type}")

        if filter_type == "lowpass":
            self.coefficients = iir_coefficients.lowpass_coefficients(filter_args)

        elif filter_type == "highpass":
            self.coefficients = iir_coefficients.highpass_coefficients(filter_args)

        elif filter_type == "allpass":
            self.coefficients = iir_coefficients.allpass_coefficients(filter_args)

        elif filter_type == "notch":
            self.coefficients = iir_coefficients.notch_coefficients(filter_args)

    def apply_filter(self, x, min_values=-10, max_values=10) -> np.ndarray:
        """
        Apply the IIR biquad filter to the input signal x.

        :param x: The input samples.
        :param min_value: The minimum value for clamping.
        :param max_value: The maximum value for clamping.
        :return: The filtered output sample.
        """
        b0, b1, b2, a1, a2 = self.coefficients

        y = np.zeros_like(x)

        y[0] = np.clip(b0 * x[0], min_values, max_values)
        y[1] = np.clip(b0 * x[1] + b1 * x[0] - a1 * y[0], min_values, max_values)

        # Calculate the filtered output
        for i in range(2, len(x)):
            y[i] = np.clip(
                b0 * x[i]
                + b1 * x[i - 1]
                + b2 * x[i - 2]
                - a1 * y[i - 1]
                - a2 * y[i - 2],
                min_values,
                max_values,
            )

        return y
