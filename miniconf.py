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

from gmqtt import Client as MqttClient

logger = logging.getLogger(__name__)


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
        self.client = client
        self.prefix = prefix
        self.inflight = {}
        self.client.on_message = self._handle_response
        self.client.subscribe(f'{prefix}/response/#')

    def _handle_response(self, _client, topic, payload, *_args, **_kwargs):
        """Callback function for when messages are received over MQTT.

        Args:
            _client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
        """
        if topic not in self.inflight:
            # TODO use correlation_data to distinguish clients and requests
            logger.warning('Unexpected response on topic: %s', topic)
            return

        self.inflight[topic].set_result(payload.decode('ascii'))
        del self.inflight[topic]

    async def command(self, path, value):
        """Write the provided data to the specified path.

        Args:
            path: The path to write the message to.
            value: The value to write to the path.

        Returns:
            The received response to the command.
        """
        setting_topic = f'{self.prefix}/settings/{path}'
        response_topic = f'{self.prefix}/response/{path}'
        if response_topic in self.inflight:
            # TODO use correlation_data to distinguish clients and requests
            raise NotImplementedError(
                    'Only one in-flight message per topic is supported')

        value = json.dumps(value)
        logger.info('Sending %s to "%s"', value, setting_topic)
        fut = asyncio.get_running_loop().create_future()
        self.inflight[response_topic] = fut
        self.client.publish(setting_topic, payload=value, qos=0, retain=True,
                            response_topic=response_topic)
        return await fut


def main():
    parser = argparse.ArgumentParser(
            description='Miniconf command line interface.',
            epilog='''Example:
            %(prog)s -v -b mqtt dt/sinara/stabilizer afe/0 '"G10"'
            ''')
    parser.add_argument('-v', '--verbose', action='count', default=0,
                        help='Increase logging verbosity')
    parser.add_argument('--broker', '-b', default='mqtt', type=str,
                        help='The MQTT broker address')
    parser.add_argument('prefix', type=str,
                        help='The MQTT topic prefix of the target')
    parser.add_argument('path', type=str,
                        help='The setting path to configure')
    parser.add_argument('value', type=str,
                        help='The value of setting in JSON format')

    args = parser.parse_args()

    logging.basicConfig(
            format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
            level=logging.WARN - 10*args.verbose)

    loop = asyncio.get_event_loop()

    async def configure_settings():
        interface = await Miniconf.create(args.prefix, args.broker)
        response = await interface.command(args.path, json.loads(args.value))
        print(f"Response: {response}")

    loop.run_until_complete(configure_settings())


if __name__ == '__main__':
    main()
