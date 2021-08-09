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

# The minimum allowable loopback voltage error (difference between output set point and input
# measured value).
MINIMUM_VOLTAGE_ERROR = 0.010

def _voltage_to_machine_units(voltage):
    """ Convert a voltage to IIR machine units.

    Args:
        voltage: The voltage to convert

    Returns:
        The IIR machine-units associated with the voltage.
    """
    dac_range = 4.096 * 2.5
    assert abs(voltage) <= dac_range, 'Voltage out-of-range'
    return voltage / dac_range * 0x7FFF


def static_iir_output(output_voltage):
    """ Generate IIR configuration for a static output voltage.

    Args:
        output_voltage: The desired static IIR output voltage.

    Returns
        The IIR configuration to send over Miniconf.
    """
    machine_units = _voltage_to_machine_units(output_voltage)
    return {
        'y_min': machine_units,
        'y_max': machine_units,
        'y_offset': 0,
        'ba': [1, 0, 0, 0, 0],
    }


class TelemetryReader:
    """ Helper utility to read Stabilizer telemetry. """

    @classmethod
    async def create(cls, prefix, broker, queue):
        """Create a connection to the broker and an MQTT device using it."""
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, prefix, queue)


    def __init__(self, client, prefix, queue):
        """ Constructor. """
        self.client = client
        self._telemetry = []
        self.client.on_message = self.handle_telemetry
        self._telemetry_topic = f'{prefix}/telemetry'
        self.client.subscribe(self._telemetry_topic)
        self.queue = queue


    def handle_telemetry(self, _client, topic, payload, _qos, _properties):
        """ Handle incoming telemetry messages over MQTT. """
        assert topic == self._telemetry_topic
        self.queue.put_nowait(json.loads(payload))


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

    telemetry_queue = asyncio.LifoQueue()

    async def telemetry():
        await TelemetryReader.create(args.prefix, args.broker, telemetry_queue)
        try:
            while True:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass


    telemetry_task = asyncio.Task(telemetry())

    async def test():
        """ The actual testing being completed. """
        interface = await Miniconf.create(args.prefix, args.broker)

        # Disable IIR holds and configure the telemetry rate.
        await interface.command('allow_hold', False, retain=False)
        await interface.command('force_hold', False, retain=False)
        await interface.command('telemetry_period', 1, retain=False)

        # Test loopback with a static 1V output of the DACs.
        await test_loopback(interface, telemetry_queue, 1.0)

        # Repeat test with AFE = 2x
        await test_loopback(interface, telemetry_queue, 1.0, gain=2)

        # Test with 0V output
        await test_loopback(interface, telemetry_queue, 0.0)

        telemetry_task.cancel()

    loop = asyncio.get_event_loop()
    sys.exit(loop.run_until_complete(test()))


if __name__ == '__main__':
    main()
