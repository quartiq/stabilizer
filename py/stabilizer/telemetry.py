import asyncio
import argparse
import logging
import json

from gmqtt import Client as MqttClient


_logger = logging.getLogger(__name__)


class TelemetryReader:
    @classmethod
    async def create(cls, prefix, broker, maxsize=1):
        client = MqttClient(client_id="")
        await client.connect(broker)
        return cls(client, prefix, maxsize)

    def __init__(self, client, prefix, maxsize):
        self.queue = asyncio.Queue(maxsize)
        self.client = client
        self.client.on_message = self.handle_telemetry
        self._telemetry_topic = f'{prefix}/telemetry'
        self.client.subscribe(self._telemetry_topic)

    def handle_telemetry(self, _client, topic, payload, _qos, _properties):
        assert topic == self._telemetry_topic
        while self.queue.full():
            _logger.debug("Discarding oldest telemetry message")
            self.queue.get_nowait()
        self.queue.put_nowait(json.loads(payload))
