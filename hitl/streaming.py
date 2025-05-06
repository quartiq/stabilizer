#!/usr/bin/python3
"""HITL testing of Stabilizer data stream capabilities"""

import asyncio
import logging
import ipaddress
import argparse
import sys
import os

import miniconf
from miniconf.common import MQTTv5, one
from stabilizer.stream import measure, Stream, get_local_ip

logger = logging.getLogger(__name__)

if sys.platform.lower() == "win32" or os.name.lower() == "nt":
    from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

    set_event_loop_policy(WindowsSelectorEventLoopPolicy())


async def _main():
    parser = argparse.ArgumentParser(description="Stabilizer Stream HITL test")
    parser.add_argument(
        "prefix",
        help="The MQTT topic prefix of the target",
    )
    parser.add_argument(
        "--broker", "-b", default="mqtt", type=str, help="The MQTT broker address"
    )
    parser.add_argument("--addr", default="0.0.0.0", help="The IP address to listen on")
    parser.add_argument(
        "--port", type=int, default=9293, help="Local port to listen on"
    )
    parser.add_argument("--duration", type=float, default=10.0, help="Test duration")
    parser.add_argument(
        "--max-loss", type=float, default=5e-2, help="Maximum loss for success"
    )
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)

    async with miniconf.Client(
        args.broker,
        protocol=MQTTv5,
        logger=logging.getLogger("aiomqtt-client"),
    ) as client:
        prefix, _alive = one(await miniconf.discover(client, args.prefix))
        conf = miniconf.Miniconf(client, prefix)
        if ipaddress.ip_address(args.addr).is_unspecified:
            args.addr = get_local_ip(args.broker)
        if ipaddress.ip_address(args.addr).is_multicast:
            local = get_local_ip(args.broker)
        else:
            local = "0.0.0.0"

        logger.info("Starting stream")
        await conf.set("/stream", f"{args.addr}:{args.port}")

        try:
            _transport, stream = await Stream.open(
                args.port, addr=args.addr, local=local
            )
            logger.info("Testing stream reception")
            loss = await measure(stream, args.duration)
            if loss > args.max_loss:
                raise RuntimeError("High frame loss", loss)
        finally:
            logger.info("Stopping stream")
            await conf.set("/stream", "0.0.0.0:0")

        logger.info("Draining queue")
        await asyncio.sleep(0.1)
        while not stream.queue.empty():
            stream.queue.get_nowait()

        logger.info("Verifying no further frames are received")
        await asyncio.sleep(1.0)
        if not stream.queue.empty():
            raise RuntimeError("Unexpected frames received")

        print("PASS")


if __name__ == "__main__":
    asyncio.run(_main())
