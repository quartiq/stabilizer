#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Loop-back integration tests for Stabilizer hardware
"""
import argparse
import asyncio
import sys

import miniconf
from stabilizer import voltage_to_machine_units
from stabilizer.telemetry import Telemetry

# The minimum allowable loopback voltage error (difference between output set point and input
# measured value).
MINIMUM_VOLTAGE_ERROR = 0.010

def static_iir_output(output_voltage):
    """ Generate IIR configuration for a static output voltage.

    Args:
        output_voltage: The desired static IIR output voltage.

    Returns
        The IIR configuration to send over Miniconf.
    """
    machine_units = voltage_to_machine_units(output_voltage)
    return {
        'y_min': machine_units,
        'y_max': machine_units,
        'y_offset': 0,
        'ba': [1, 0, 0, 0, 0],
    }


async def test_loopback(stabilizer, telemetry_queue, set_point, gain=1, channel=0):
    """ Test loopback operation of Stabilizer.

    Note:
        Loopback is tested by configuring DACs for static output and verifying telemetry reports the
        ADCs are measuring those values. Output OUTx should be connected in a loopback configuration
        to INx on the device.

    Args:
        stabilizer: The miniconf configuration interface.
        telemetry: a helper utility to read inbound telemetry.
        set_point: The desired output voltage to test.
        channel: The loopback channel to test on. Either 0 or 1.
        gain: The desired AFE gain.
    """
    print(f'Testing loopback for Vout = {set_point:.2f}, Gain = x{gain}')
    print('---------------------------------')
    # Configure the AFE and IIRs to output at the set point
    await stabilizer.set(f'/afe/{channel}', f'G{gain}')
    await stabilizer.set(f'/iir_ch/{channel}/0', static_iir_output(set_point))

    # Configure signal generators to not affect the test.
    await stabilizer.set('/signal_generator/0/amplitude', 0)

    # Wait for telemetry to update.
    await asyncio.sleep(5.0)

    # Verify the ADCs are receiving the setpoint voltage.
    tolerance = max(0.05 * set_point, MINIMUM_VOLTAGE_ERROR)
    latest_values = await telemetry_queue.get()
    print(f'Latest telemtry: {latest_values}')

    assert abs(latest_values['adcs'][channel] - set_point) < tolerance
    print('PASS')
    print('')


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Loopback tests for Stabilizer HITL testing',)
    parser.add_argument('prefix', type=str, nargs='?',
                        help='The MQTT topic prefix of the target')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')

    args = parser.parse_args()

    async def test():
        """ The actual testing being completed. """
        prefix = args.prefix
        if not args.prefix:
            devices = await miniconf.discover(args.broker, 'dt/sinara/dual-iir/+', 1)
            if not devices:
                raise Exception('No Stabilizer (Dual-iir) devices found')
            assert len(devices) == 1, \
                f'Multiple Stabilizers found: {devices}. Please specify one with --prefix'

            prefix = devices.pop()

        tele = await Telemetry.create(prefix, args.broker)

        stabilizer = await miniconf.Miniconf.create(prefix, args.broker)

        # Disable IIR holds and configure the telemetry rate.
        await stabilizer.set('/allow_hold', False)
        await stabilizer.set('/force_hold', False)
        await stabilizer.set('/telemetry_period', 1)

        # Test loopback with a static 1V output of the DACs.
        await test_loopback(stabilizer, tele.queue, 1.0)

        # Repeat test with AFE = 2x
        await test_loopback(stabilizer, tele.queue, 1.0, gain=2)

        # Test with 0V output
        await test_loopback(stabilizer, tele.queue, 0.0)

    sys.exit(asyncio.run(test()))


if __name__ == '__main__':
    main()
