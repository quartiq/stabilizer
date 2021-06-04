#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers
        Robert Jördens

Description: Provides an API for controlling Miniconf devices over MQTT.
"""
import argparse
import asyncio
import json
import logging
import sys
import uuid

from gmqtt import Client as MqttClient

LOGGER = logging.getLogger(__name__)


class Miniconf:
    """An asynchronous API for controlling Miniconf devices using MQTT."""

    @classmethod
    async def create(cls, prefix, broker):
        """Create a connection to the broker and a Miniconf device using it."""
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, prefix)

    def __init__(self, client, prefix):
        """Constructor.

        Args:
            client: A connected MQTT5 client.
            prefix: The MQTT toptic prefix of the device to control.
        """
        self.request_id = 0
        self.client = client
        self.prefix = prefix
        self.inflight = {}
        self.client.on_message = self._handle_response
        self.response_topic = f'{prefix}/response/{uuid.uuid1().hex}'
        self.client.subscribe(self.response_topic)

    def _handle_response(self, _client, topic, payload, _qos, properties):
        """Callback function for when messages are received over MQTT.

        Args:
            _client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
            _qos: The quality-of-service level of the received packet
            properties: A dictionary of properties associated with the message.
        """
        if topic == self.response_topic:
            # Extract request_id corrleation data from the properties
            request_id = int.from_bytes(
                properties['correlation_data'][0], 'big')

            self.inflight[request_id].set_result(json.loads(payload))
            del self.inflight[request_id]
        else:
            LOGGER.warn('Unexpected message on "%s"', topic)

    async def command(self, path, value, retain=True):
        """Write the provided data to the specified path.

        Args:
            path: The path to write the message to.
            value: The value to write to the path.
            retain: Retain the MQTT message changing the setting
                by the broker.

        Returns:
            The response to the command as a dictionary.
        """
        topic = f'{self.prefix}/settings/{path}'

        fut = asyncio.get_running_loop().create_future()

        # Assign unique correlation data for response dispatch
        assert self.request_id not in self.inflight
        self.inflight[self.request_id] = fut
        correlation_data = self.request_id.to_bytes(4, 'big')
        self.request_id += 1

        payload = json.dumps(value)
        LOGGER.info('Sending "%s" to "%s"', value, topic)

        self.client.publish(
            topic, payload=payload, qos=0, retain=retain,
            response_topic=self.response_topic,
            correlation_data=correlation_data)

        return await fut


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(
        description='Miniconf command line interface.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''Examples:
%(prog)s dt/sinara/dual-iir/00-11-22-33-aa-bb iir_ch/0/0=\
'{"y_min":-32767,"y_max":32767,"y_offset":0,"ba":[1.0,0,0,0,0]}'
%(prog)s dt/sinara/lockin/00-11-22-33-aa-bb afe/0='"G2"'\
''')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='Increase logging verbosity')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('--no-retain', '-n', default=False,
                        action='store_true',
                        help='Do not retain the affected settings')
    parser.add_argument('prefix', type=str,
                        help='The MQTT topic prefix of the target')
    parser.add_argument('settings', metavar="PATH=VALUE", nargs='+',
                        help='JSON encoded values for settings path keys.')

    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        level=logging.WARN - 10*args.verbose)

    loop = asyncio.get_event_loop()

    async def configure_settings():
        interface = await Miniconf.create(args.prefix, args.broker)
        for setting in args.settings:
            path, value = setting.split("=", 1)
            response = await interface.command(path, json.loads(value),
                                               not args.no_retain)
            print(f'{path}: {response}')
            if response['code'] != 0:
                return response['code']
        return 0

    sys.exit(loop.run_until_complete(configure_settings()))


if __name__ == '__main__':
    main()
