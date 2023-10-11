#!/usr/bin/python3
"""HITL testing of Stabilizer data livestream capabilities"""

import asyncio
import logging
import ipaddress
import argparse

from miniconf import Miniconf
from stabilizer.stream import measure, StabilizerStream, get_local_ip

logger = logging.getLogger(__name__)


async def _main():
    parser = argparse.ArgumentParser(description="Stabilizer Stream HITL test")
    parser.add_argument("prefix", type=str,
                        help="The MQTT topic prefix of the target")
    parser.add_argument("--broker", "-b", default="mqtt", type=str,
                        help="The MQTT broker address")
    parser.add_argument("--ip", default="0.0.0.0",
                        help="The IP address to listen on")
    parser.add_argument("--port", type=int, default=9293,
                        help="Local port to listen on")
    parser.add_argument("--duration", type=float, default=10.,
                        help="Test duration")
    parser.add_argument("--max-loss", type=float, default=5e-2,
                        help="Maximum loss for success")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    conf = await Miniconf.create(args.prefix, args.broker)

    stream_target = [int(x) for x in args.ip.split('.')]
    if ipaddress.ip_address(args.ip).is_unspecified:
        stream_target = get_local_ip(args.broker)

    logger.info("Starting stream")
    await conf.set(
        "/stream_target", {
            "ip": stream_target,
            "port": args.port
        }, retain=False)

    try:
        logger.info("Testing stream reception")
        _transport, stream = await StabilizerStream.open(args.ip,
                                                         args.port,
                                                         args.broker)
        loss = await measure(stream, args.duration)
        if loss > args.max_loss:
            raise RuntimeError("High frame loss", loss)
    finally:
        logger.info("Stopping stream")
        await conf.set(
            "/stream_target", {"ip": [0, 0, 0, 0], "port": 0}, retain=False)

    logger.info("Draining queue")
    await asyncio.sleep(.1)
    while not stream.queue.empty():
        stream.queue.get_nowait()

    logger.info("Verifying no further frames are received")
    await asyncio.sleep(1.)
    if not stream.queue.empty():
        raise RuntimeError("Unexpected frames received")

    print("PASS")


if __name__ == "__main__":
    asyncio.run(_main())
