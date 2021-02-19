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
        self.response_topic = f'{identifier}/feedback'
        self.client = client
        self.identifier = identifier
        self.command_complete = asyncio.Event()
        self.client.on_message = self._handle_response
        self.response = None
        self.logger = logging.getLogger('stabilizer.miniconf')

        self.client.subscribe(self.response_topic)


    def _handle_response(self, client, topic, payload, *_args, **_kwargs):
        """ Callback function for when messages are received over MQTT.

        Args:
            client: The MQTT client.
            topic: The topic that the message was received on.
            payload: The payload of the message.
        """
        if topic != self.response_topic:
            raise Exception(f'Unknown topic: {topic}')

        # Indicate a response was received.
        self.response = payload.decode('ascii')
        self.command_complete.set()


    async def _command(self, topic, message):
        """ Send a command to a topic.

        Args:
            topic: The topic to send the message to.
            message: The message to send to the provided topic.

        Returns:
            The received response to the command.
        """
        self.command_complete.clear()
        self.logger.debug('Sending "%s" to "%s"', message, topic)
        self.client.publish(topic, payload=message, qos=0, retain=False,
                            response_topic=self.response_topic)
        await self.command_complete.wait()

        response = self.response
        self.response = None

        return response


    async def set_setting(self, setting, message):
        """ Change the provided setting with the provided data. """
        return await self._command(f'{self.identifier}/settings/{setting}', message)


    async def commit(self):
        """ Commit staged settings to become active. """
        return await self._command(f'{self.identifier}/commit', 'commit')


async def configure_settings(args):
    """ Configure an RF channel. """
    logger = logging.getLogger('stabilizer')

    # Establish a communication interface with stabilizer.
    interface = await MiniconfApi.create(args.stabilizer, args.broker)

    request = None

    # In the exceptional case that this is a terminal value, there is no key available and only a
    # single value.
    if len(args.values) == 1 and '=' not in args.values[0]:
        request = json.loads(args.values[0])
    else:
        # Convert all of the values into a key-value list.
        request = dict()
        for pair in args.values:
            key, value = pair.split('=')
            request[str(key)] = json.loads(value)
    logger.debug('Parsed request: %s', request)

    response = await interface.set_setting(args.setting, json.dumps(request))
    logger.info(response)

    if args.commit:
        response = await interface.commit()
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
    parser.add_argument('--commit', action='store_true',
                        help='Specified true to commit after updating settings.')
    parser.add_argument('-v', '--verbose', action='store_true', help='Enable verbose logging')

    args = parser.parse_args()

    logger = logging.getLogger('stabilizer')
    logger.setLevel(logging.INFO)

    if args.verbose:
        logger.setLevel(logging.DEBUG)
        logging.basicConfig(level=logging.DEBUG)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(configure_settings(args))


if __name__ == '__main__':
    main()
