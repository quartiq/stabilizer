#!/usr/bin/python3
"""
Author: Vertigo Designs, Ryan Summers

Description: Loop-back integration tests for Stabilizer hardware
"""
import argparse
import asyncio
import json
import sys

from gmqtt import Client as MqttClient
from miniconf import Miniconf
import stabilizer

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
    machine_units = stabilizer.voltage_to_machine_units(output_voltage)
    return {
        'y_min': machine_units,
        'y_max': machine_units,
        'y_offset': 0,
        'ba': [1, 0, 0, 0, 0],
    }


async def test_loopback(miniconf, telemetry_queue, set_point, gain=1, channel=0):
    """ Test loopback operation of Stabilizer.

    Note:
        Loopback is tested by configuring DACs for static output and verifying telemetry reports the
        ADCs are measuring those values. Output OUTx should be connected in a loopback configuration
        to INx on the device.

    Args:
        miniconf: The miniconf configuration interface.
        telemetry: a helper utility to read inbound telemetry.
        set_point: The desired output voltage to test.
        channel: The loopback channel to test on. Either 0 or 1.
        gain: The desired AFE gain.
    """
    print(f'Testing loopback for Vout = {set_point:.2f}, Gain = x{gain}')
    print('---------------------------------')
    # Configure the AFE and IIRs to output at the set point
    await miniconf.command(f'afe/{channel}', f'G{gain}', retain=False)
    await miniconf.command(f'iir_ch/{channel}/0', static_iir_output(set_point), retain=False)

    # Configure signal generators to not affect the test.
    await miniconf.command('signal_generator/0/amplitude', 0, retain=False)

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
    parser.add_argument('prefix', type=str,
                        help='The MQTT topic prefix of the target')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')

    args = parser.parse_args()

    async def test():
        """ The actual testing being completed. """
        tele = await stabilizer.TelemetryReader.create(args.prefix, args.broker)

        interface = await Miniconf.create(args.prefix, args.broker)

        # Disable IIR holds and configure the telemetry rate.
        await interface.command('allow_hold', False, retain=False)
        await interface.command('force_hold', False, retain=False)
        await interface.command('telemetry_period', 1, retain=False)

        # Test loopback with a static 1V output of the DACs.
        await test_loopback(interface, tele.queue, 1.0)

        # Repeat test with AFE = 2x
        await test_loopback(interface, tele.queue, 1.0, gain=2)

        # Test with 0V output
        await test_loopback(interface, tele.queue, 0.0)

        tele.cancel()

    sys.exit(asyncio.run(test()))


if __name__ == '__main__':
    main()
