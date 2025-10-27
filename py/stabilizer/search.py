"""Stabilizer FLS signal searcher"""

# pylint: disable=logging-fstring-interpolation,too-many-statements,too-many-locals,duplicate-code


import asyncio
import argparse
import logging

import numpy as np
from tqdm.asyncio import tqdm

import miniconf
from miniconf.common import MQTTv5, one
from stabilizer.stream import Stream, get_local_ip

from . import stream as _

_logger = logging.getLogger(__name__)


async def main():
    """FLS Signal searcher"""
    parser = argparse.ArgumentParser(
        description="Search demodulation frequency range and identify "
        "highest power input frequency"
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
        "--threshold",
        "-t",
        default=-15,
        type=float,
        help="log2 of tone power threshold",
    )

    args = parser.parse_args()
    logging.basicConfig(
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        level=logging.WARN - 10 * args.verbose,
    )

    async with miniconf.Client(
        args.broker,
        protocol=MQTTv5,
        logger=logging.getLogger("aiomqtt-client"),
    ) as client:
        prefix, _alive = one(await miniconf.discover(client, args.prefix))

        conf = miniconf.Miniconf(client, prefix)
        local_ip = get_local_ip(args.broker)
        _transport, stream = await Stream.open(args.port, local_ip)

        f2w = (1 << 32) / 500e6
        f2s = 4 * 8 * 128 / 400e6
        f_demod = 0x200000 / f2w

        async def measure(f, nframe=1):
            await conf.set(f"/ch/{args.channel}/input/freq", int(f * f2w))
            # settle
            # await asyncio.sleep(0.1)
            # discard pending frames
            while True:
                try:
                    stream.queue.get_nowait()
                except asyncio.QueueEmpty:
                    break
            # discard one more
            await stream.queue.get()
            demod = []
            for _ in range(nframe):
                frame = await stream.queue.get()
                demod.append(frame.demod()[:, args.channel])
            demod = np.concatenate(demod)
            p = np.log2(np.square(demod.astype(np.int64)).mean() / (1 << 62))
            iq = demod.astype(np.float64).ravel().view(np.complex128)
            df = np.angle(iq[1:] * iq[:-1].conj()).mean() / (2 * np.pi * f2s)
            return f + df, p

        try:
            await conf.set("/stream", f"{local_ip}:{args.port}")
            fp = []
            with tqdm(np.arange(1e6, 240e6, 40e3)) as pbar:
                async for f in pbar:
                    ff, p = await measure(f)
                    fp.append((ff, p))
                    if p > args.threshold:
                        _logger.info(f"{ff:g} Hz: {p:g} 3dB")
                    else:
                        _logger.debug(f"{ff:g} Hz: {p:g} 3dB")
            if not fp:
                raise ValueError("no tones found above threshold")
            f0, _p0 = max(fp, key=lambda k: k[1])
            fu, pu = await measure(f0 + 2 * f_demod)
            _fl, pl = await measure(f0 - 2 * f_demod)
            if pu > pl:
                f0 = fu
            f0, p0 = await measure(f0, nframe=100)
            w = round(int(f0 * f2w))
            _logger.warning(
                f"final {f0:f} Hz, {p0:g} 3dB, ch/{args.channel}/input/freq={w:#x}"
            )
            await conf.set(f"/ch/{args.channel}/input/freq", w)
        finally:
            await conf.set("/stream", "0.0.0.0:0")


if __name__ == "__main__":
    asyncio.run(main())
