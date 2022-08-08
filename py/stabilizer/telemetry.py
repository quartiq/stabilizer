"""
Stabilizer MQTT Telemetry client implementation
"""

import asyncio
import logging
import json

from gmqtt import Client as MqttClient


_logger = logging.getLogger(__name__)

class Telemetry:
    """
    Stabilizer MQTT Telemetry receiver.

    This connects to the MQTT broker where Stabilizer publishes its telemetry
    messages. The client subscribes to the telemetry topic and puts the
    messages into a queue (accessible through the `queue` attribute).

    When the queue is full the oldest messages will be dropped.
    """
    @classmethod
    async def create(cls, prefix, broker, maxsize=1):
        """
        Connect to the MQTT broker at `broker` and start listening for Stabilizer
        telemetry messages under the prefix `prefix`.
        """
        client = MqttClient(client_id="")
        await client.connect(broker)
        return cls(client, prefix, maxsize)

    def __init__(self, client, prefix, maxsize):
        self.queue = asyncio.Queue(maxsize)
        self.client = client
        self.client.on_message = self._handle_telemetry
        self._telemetry_topic = f'{prefix}/telemetry'
        self.client.subscribe(self._telemetry_topic)

    def _handle_telemetry(self, _client, topic, payload, _qos, _properties):
        assert topic == self._telemetry_topic
        while self.queue.full():
            _logger.debug("Discarding oldest telemetry message")
            self.queue.get_nowait()
        self.queue.put_nowait(json.loads(payload))

    async def get(self):
        """Retrieve the latest telemetry message in the queue or wait for a new one"""
        await self.queue.get()

    async def get_nowait(self):
        """Retrieve the latest telemetry message already in the queue"""
        await self.queue.get_nowait()
