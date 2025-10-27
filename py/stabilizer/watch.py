"""Stabilizer FLS event watcher and recorder"""

# pylint: disable=logging-fstring-interpolation,too-many-statements

import asyncio
import argparse
import logging
import json
import time

import miniconf
from miniconf.common import MQTTv5, one
from stabilizer.stream import Frame, Stream, get_local_ip, wrap

from . import stream as _

_logger = logging.getLogger(__name__)


async def main():
    """FLS watcher"""
    parser = argparse.ArgumentParser(
        description="Watch telemetry messages for digital_input, blank, and slip "
        "events and dump buffered stream data around those triggers into files."
    )
    parser.add_argument(
        "-v", "--verbose", action="count", default=0, help="Increase logging verbosity"
    )
    parser.add_argument(
        "--broker",
        "-b",
        default="mqtt",
        help="The MQTT broker address to use to use (%(default)s)",
    )
    parser.add_argument(
        "--prefix",
        "-p",
        default="dt/sinara/fls/+",
        help="The MQTT topic prefix (%(default)s)",
    )
    parser.add_argument(
        "--channel",
        "-c",
        type=int,
        choices=[0, 1],
        default=0,
        help="The channel to operate on",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9293,
        help="Local port to listen on for streaming data",
    )
    parser.add_argument(
        "--buffer",
        "-n",
        default=1 << 12,
        type=int,
        help="number of packets to buffer (%(default)s)",
    )
    parser.add_argument(
        "--dump", "-r", default="fls_event", help="file name prefix, (%(default)s)"
    )
    parser.add_argument(
        "--max",
        "-m",
        default=20,
        type=int,
        help="max number of telemetry messages with active trigger"
        " before exitin to limit the disk usage. (%(default)s)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.WARN - 10 * args.verbose,
    )

    local_ip = get_local_ip(args.broker)

    async def dump(ev):
        transport, stream = await Stream.open(args.port, local_ip, maxsize=args.buffer)
        try:
            while True:
                await ev.wait()
                name = f"{args.dump}_{int(time.time())}.raw"
                _logger.warning(f"trigger start {name}")
                count = 0
                with open(name, "wb") as f:
                    while ev.is_set():
                        fr = await stream.queue.get()
                        f.write(Frame.header_fmt.pack(*fr.header))
                        f.write(fr.body)
                        count += 1
                _logger.warning(f"trigger end, written {count} frames")
        finally:
            transport.close()

    async def watch(ev):
        async with miniconf.Client(
            args.broker,
            protocol=MQTTv5,
            logger=logging.getLogger("aiomqtt-client"),
        ) as client:
            prefix, _alive = one(await miniconf.discover(client, args.prefix))
            conf = miniconf.Miniconf(client, prefix)
            await conf.set("/stream", f"{local_ip}:{args.port}")
            await conf.close()

            topic = f"{prefix}/telemetry"
            await client.subscribe(topic)
            try:
                last = None
                n = 0
                async for tele in client.messages:
                    if n > args.max:
                        break
                    tele = json.loads(tele.payload)
                    if trigger(tele, last, args.channel):
                        ev.set()
                        n += 1
                    else:
                        ev.clear()
                    last = tele
            finally:
                ev.clear()
                await client.unsubscribe(topic)
                conf = miniconf.Miniconf(client, prefix)
                await conf.set("/stream", "0.0.0.0:0")
                await conf.close()

    def trigger(new, last, channel):
        if last is None:
            return False
        if channel is None:
            channels = [0, 1]
        else:
            channels = [channel]
        for ch in channels:
            if wrap(new["raw"][ch]["holds"] - last["raw"][ch]["holds"]) > 0:
                continue
            blanks = wrap(new["raw"][ch]["blanks"] - last["raw"][ch]["blanks"])
            if blanks > 0:
                _logger.info(f"trigger ch{ch} on {blanks} blanks")
                return True
            slips = wrap(new["raw"][ch]["slips"] - last["raw"][ch]["slips"])
            if slips > 0:
                _logger.info(f"trigger ch{ch} on {slips} slips")
                return True
        return False

    ev = asyncio.Event()
    await asyncio.gather(dump(ev), watch(ev))


if __name__ == "__main__":
    asyncio.run(main())
