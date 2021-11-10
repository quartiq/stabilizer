#!/usr/bin/python3
"""HITL testing of Stabilizer data livestream capabilities"""

import asyncio
import logging
import socket
import argparse

from miniconf import Miniconf
from stabilizer.stream import measure, StabilizerStream

logger = logging.getLogger(__name__)


def _get_ip(remote):
    """Get the local IP of a connection to the to a remote host."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect((remote, 1883))
        address = sock.getsockname()[0]
    finally:
        sock.close()

    return list(map(int, address.split(".")))


async def _main():
    parser = argparse.ArgumentParser(description="Stabilizer Stream HITL test")
    parser.add_argument("prefix", type=str,
                        help="The MQTT topic prefix of the target")
    parser.add_argument("--broker", "-b", default="mqtt", type=str,
                        help="The MQTT broker address")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Local address to listen on")
    parser.add_argument("--port", type=int, default=9293,
                        help="Local port to listen on")
    parser.add_argument("--duration", type=float, default=10.,
                        help="Test duration")
    parser.add_argument("--max-loss", type=float, default=5e-2,
                        help="Maximum loss for success")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    conf = await Miniconf.create(args.prefix, args.broker)
    local_ip = _get_ip(args.broker)

    logger.info("Starting stream")
    await conf.command(
        "stream_target", {"ip": local_ip, "port": args.port}, retain=False)

    try:
        logger.info("Testing stream reception")
        stream = await StabilizerStream.open((args.host, args.port))
        loss = await measure(stream, args.duration)
        if loss > args.max_loss:
            raise RuntimeError("High frame loss", loss)
    finally:
        logger.info("Stopping stream")
        await conf.command(
            "stream_target", {"ip": [0, 0, 0, 0], "port": 0}, retain=False)

    logger.info("Draining queue")
    await asyncio.sleep(.1)
    while not stream.queue.empty():
        stream.queue.get_nowait()

    try:
        logger.info("Verifying no further frames are received")
        await asyncio.wait_for(stream.queue.get(), timeout=1.)
    except asyncio.TimeoutError:
        pass
    else:
        raise RuntimeError("Unexpected frames received")

    print("PASS")


if __name__ == "__main__":
    asyncio.run(_main())
