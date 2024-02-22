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
import logging

from math import pi, inf

import miniconf
import json

import stabilizer

logger = logging.getLogger(__name__)

# Disable pylint warnings about a0, b1 etc
#pylint: disable=invalid-name

default_args = {
    "Kp": 0,
    "Ki": 0,
    "Kd": 0,
    "aom_frequency": 80e6,
    "attn_out": 0.5,
    "attn_in": 0.5,
    "sample_period": stabilizer.SAMPLE_PERIOD,
    "x_offset": 0,
    "y_min": stabilizer.DAC_FULL_SCALE,
    "y_max": stabilizer.DAC_FULL_SCALE,
    "y_offset": 0,
    "Ki_limit": inf,
    "Kd_limit": inf,
    "Kii": 0,
    "Kii_limit": inf,
    "Kdd": 0,
    "Kdd_limit": inf,
}


def pid_coefficients(args):
    """
    Calculate PID IIR filter coefficients.
    
    Note:
        Calculations coefficient largely taken using the derivations in
        page 9 of https://arxiv.org/pdf/1508.06319.pdf

        PII/PID coefficient equations are taken from the PID-IIR primer
        written by Robert Jördens at https://hackmd.io/IACbwcOTSt6Adj3_F9bKuw
    """

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
    parser.add_argument("--broker", "-b", type=str, default="10.255.6.4",
                        help="The MQTT broker to use to communicate with "
                        "Stabilizer. Default: (%(default)s)")
    parser.add_argument("--address", "-a", type=str, default="+", help="MAC address of the Stabilizer board")
    parser.add_argument("--channel", "-c", type=int, choices=[0, 1],
                        required=True, help="The Stabilizer channel to configure.")
    
    parser.add_argument("--Kp", type=float, default=0,
                        help="Proportional (P) gain")
    parser.add_argument("--Ki", type=float, default=0,
                        help="Integrator (I) gain")
    parser.add_argument("--Kd", type=float, default=0,
                        help="Derivative (D) gain")
    parser.add_argument("--aom-frequency", "-f", type=float, default=80e6,
                        help="Aom centre frequency (Hz) ")
    parser.add_argument("--attn-out", type=float, default=0.5,
                        help="Output attenuation (dB) ")
    parser.add_argument("--attn-in", type=float, default=16.5,
                        help="Input attenuation (dB) ")
    
    parser.add_argument("--sample-period", type=float, default= 2**9 * 10e-9,
                        help="Sample period in seconds. ")

    parser.add_argument("--prefix", "-p", type=str,
                        default="dt/sinara/fnc",
                        help="The Stabilizer device prefix path in MQTT, "
                        "wildcards allowed as long as the match is unique "
                        "Default: (%(default)s)")
    parser.add_argument("--no-discover", "-d", action="store_true",
                        help="Do not discover Stabilizer device prefix.")

    parser.add_argument("--x-offset", type=float, default=0,
                        help="The channel input offset (V)")
    parser.add_argument("--y-min", type=float, default=-stabilizer.DAC_FULL_SCALE,
                        help="The channel minimum output (V)")
    parser.add_argument("--y-max", type=float, default=stabilizer.DAC_FULL_SCALE,
                        help="The channel maximum output (V)")
    parser.add_argument("--y-offset", type=float, default=0,
                        help="The channel output offset (V)")
    
    parser.add_argument("--Kii", type=float, default=0,
                        help="Double Integrator (I^2) gain")
    parser.add_argument("--Kii_limit", type=float, default=inf,
                        help="Integral gain limit")
    parser.add_argument("--Ki_limit", type=float, default=inf,
                        help="Integral gain limit")
    parser.add_argument("--Kd_limit", type=float, default=inf,
                        help="Derivative gain limit")
    parser.add_argument("--Kdd", type=float, default=0,
                        help="Double Derivative (D^2) gain")
    parser.add_argument("--Kdd_limit", type=float, default=inf,
                        help="Derivative gain limit")

    args = parser.parse_args()

    device_path = "{}/{}".format(args.prefix, args.address)

    print(device_path)

    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        level=logging.WARN - 10*args.verbose)

    # Calculate the IIR coefficients for the filter.
    coefficients = pid_coefficients(args)

    # The feed-forward gain of the IIR filter is the summation
    # of the "b" components of the filter.
    forward_gain = sum(coefficients[:3])
    if forward_gain == 0 and args.x_offset != 0:
        logger.warning("Filter has no DC gain but x_offset is non-zero")

    if not (0.5 <= args.attn_out <= 31.5):
        logger.warning("Output attenuation out of range, setting to default 0.5 dB")
        args.attn_out = 0.5
    if not (0.5 <= args.attn_in <= 31.5):
        logger.warning("Input attenuation out of range, setting to default 0.5 dB")
        args.attn_in = 0.5

    if args.no_discover:
        prefix = device_path
    else:
        devices = asyncio.run(miniconf.discover(args.broker, device_path))
        if not devices:
            raise ValueError("No prefixes discovered.")
        if len(devices) > 1:
            raise ValueError(f"Multiple prefixes discovered ({devices})."
                             "Please specify a more specific --prefix")
        prefix = devices.pop()
        logger.info("Automatically using detected device prefix: %s", prefix)

    filename = "fnc_values/{}.json".format(device_path.replace("/", "_"))

    print(coefficients)

    # try:
    #     with open(filename, 'r') as fp:
    #         json_dict = json.load(fp)
    # except:
    #     json_dict = {}
    #     args_dict = vars(args)
    #     json_dict["ch{}".format(args.channel)] = {key: args_dict[key] for key in default_args.keys()}
    #     json_dict["ch{}".format(1-args.channel)] = {key: default_args[key] for key in default_args.keys()}

    #     with open(filename, 'w') as fp:
    #         json.dump(json_dict, fp)
    

    async def configure():
        logger.info("Connecting to broker")
        interface = await miniconf.Miniconf.create(prefix, args.broker)

        # Set the filter coefficients.
        # Note: In the future, we will need to Handle higher-order cascades.
        await interface.set(f"/iir_ch/{args.channel}/0", {
            "ba": coefficients,
            "y_min": stabilizer.voltage_to_machine_units(args.y_min),
            "y_max": stabilizer.voltage_to_machine_units(args.y_max),
            "y_offset": stabilizer.voltage_to_machine_units(
                args.y_offset + forward_gain * args.x_offset)
        }, retain=True)
        await interface.set(f"/aom_centre_f", args.aom_frequency, retain=True)

        await interface.set(f"/output_attenuation", args.attn_out, retain=True)
        await interface.set(f"/input_attenuation", args.attn_in, retain=True)

    asyncio.run(configure())


if __name__ == "__main__":
    _main()
