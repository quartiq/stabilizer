#!/usr/bin/python
"""
Author: Vertigo Designs, Ryan Summers

Description: Provides an API for controlling Stabilizer over Miniconf (MQTT).
"""
import argparse
import asyncio
import json
import logging

from gmqtt  import Client as MqttClient


class MiniconfApi:
    """ An asynchronous API for controlling Miniconf devices using the MQTT control interface. """

    @classmethod
    async def create(cls, identifier, broker):
        """ Create a connection to MQTT for communication with the device. """
        client = MqttClient(client_id='')
        await client.connect(broker)
        return cls(client, identifier)


    def __init__(self, client, identifier):
        """ Consructor.

        Args:
            client: A connected MQTT5 client.
            identifier: The ID of the device to control.
        """
        self.client = client
        self.identifier = identifier
        self.client.on_message = self._handle_response
        self.inflight_settings = dict()
        self.logger = logging.getLogger('stabilizer.miniconf')

        self.client.subscribe(f'{identifier}/feedback/#')


    def _handle_response(self, _client, topic, payload, *_args, **_kwargs):
        """ Callback function for when messages are received over MQTT.

        Args:
            _client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
        """
        if topic not in self.inflight_settings:
            self.logger.warning('Unknown response topic: %s', topic)
            return

        # Indicate a response was received for the provided topic.
        self.inflight_settings[topic].set_result(payload.decode('ascii'))


    async def command(self, path, value):
        """ Write the provided data to the specified path.

        Args:
            setting: The path to write the message to.
            value: The value to write to the path.

        Returns:
            The received response to the command.
        """
        setting_topic = f'{self.identifier}/{path}'
        response_topic = f'{self.identifier}/feedback/{path}'
        assert response_topic not in self.inflight_settings, \
            'Only one in-flight message per topic is supported'

        self.logger.debug('Sending %s to "%s"', value, setting_topic)
        self.inflight_settings[response_topic] = asyncio.get_running_loop().create_future()
        self.client.publish(setting_topic, payload=value, qos=0, retain=False,
                            response_topic=response_topic)

        response = await self.inflight_settings[response_topic]
        del self.inflight_settings[response_topic]

        return response


async def configure_settings(args):
    """ Configure an RF channel. """
    logger = logging.getLogger('stabilizer')

    # Establish a communication interface with stabilizer.
    interface = await MiniconfApi.create(args.stabilizer, args.broker)

    request = None

    # In the exceptional case that this is a terminal value, there is no key available and only a
    # single value.
    if len(args.values) == 1 and '=' not in args.values[0]:
        if args.values[0][0].isalpha():
            request = args.values[0]
        else:
            request = json.loads(args.values[0])
    else:
        # Convert all of the values into a key-value list.
        request = dict()
        for pair in args.values:
            key, value = pair.split('=')
            request[str(key)] = json.loads(value)
    logger.debug('Parsed request: %s', request)

    response = await interface.command(f'settings/{args.setting}', json.dumps(request))
    logger.info(response)


def main():
    """ Main program entry point. """
    parser = argparse.ArgumentParser(description='Stabilizer settings modification utility')
    parser.add_argument('--stabilizer', type=str, default='stabilizer',
                        help='The identifier of the stabilizer to configure')
    parser.add_argument('--setting', required=True, type=str, help='The setting path to configure')
    parser.add_argument('--broker', default='10.34.16.1', type=str, help='The MQTT broker address')
    parser.add_argument('values', nargs='+', type=str,
                        help='The value of settings. key=value list or a single value is accepted.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    logger = logging.getLogger('stabilizer')
    logger.setLevel(logging.INFO)

    if args.verbose:
        logger.setLevel(logging.DEBUG)

    logging.basicConfig(format='%(asctime)s [%(levelname)s] %(message)s')

    loop = asyncio.get_event_loop()
    loop.run_until_complete(configure_settings(args))


if __name__ == '__main__':
    main()
