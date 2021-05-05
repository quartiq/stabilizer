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
        self.uuid = uuid.uuid1()
        self.request_id = 0
        self.client = client
        self.prefix = prefix
        self.inflight = {}
        self.client.on_message = self._handle_response
        self.client.subscribe(f'{prefix}/response/{self.uuid.hex}')
        self.logger = logging.getLogger(__name__)

    def _handle_response(self, _client, _topic, payload, _qos, properties):
        """Callback function for when messages are received over MQTT.

        Args:
            _client: The MQTT client.
            _topic: The topic that the message was received on.
            payload: The payload of the message.
            _qos: The quality-of-service level of the received packet
            properties: A dictionary of properties associated with the message.
        """
        # Extract corrleation data from the properties
        correlation_data = json.loads(properties['correlation_data'][0].decode('ascii'))

        # Get the request ID from the correlation data
        request_id = correlation_data['request_id']

        self.inflight[request_id].set_result(json.loads(payload))
        del self.inflight[request_id]


    async def command(self, path, value):
        """Write the provided data to the specified path.

        Args:
            path: The path to write the message to.
            value: The value to write to the path.

        Returns:
            (code, msg) tuple as a response to the command. `code` is zero for success and `msg` is
            a use-readable message indicating further information.
        """
        setting_topic = f'{self.prefix}/settings/{path}'
        response_topic = f'{self.prefix}/response/{self.uuid.hex}'

        # Assign a unique identifier to this update request.
        request_id = self.request_id
        self.request_id += 1
        assert request_id not in self.inflight, 'Invalid ID encountered'

        correlation_data = json.dumps({
            'request_id': request_id,
        }).encode('ascii')

        value = json.dumps(value)
        self.logger.info('Sending %s to "%s"', value, setting_topic)
        fut = asyncio.get_running_loop().create_future()

        self.inflight[request_id] = fut
        self.client.publish(setting_topic, payload=value, qos=0, retain=True,
                            response_topic=response_topic,
                            correlation_data=correlation_data)
        return await fut


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(
        description='Miniconf command line interface.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''Examples:
%(prog)s dt/sinara/stabilizer afe/0='"G2"' iir_ch/0/0=\
'{"y_min": -32767, "y_max": 32767, "y_offset": 0, "ba": [1.0, 0, 0, 0, 0]}'
''')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='Increase logging verbosity')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('prefix', type=str,
                        help='The MQTT topic prefix of the target')
    parser.add_argument('settings', metavar="KEY=VALUE", nargs='+',
                        help='JSON encoded values for settings path keys.')

    args = parser.parse_args()

    logging.basicConfig(
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        level=logging.WARN - 10*args.verbose)

    loop = asyncio.get_event_loop()

    async def configure_settings():
        interface = await Miniconf.create(args.prefix, args.broker)
        for key_value in args.settings:
            path, value = key_value.split("=", 1)
            response = await interface.command(path, json.loads(value))
            print(f'{path}: {response}')
            if response['code'] != 0:
                return response['code']
        return 0

    sys.exit(loop.run_until_complete(configure_settings()))


if __name__ == '__main__':
    main()
