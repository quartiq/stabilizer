"""MPLL calibration/sweep/searcher"""

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
    """MPLL Sweep"""
    parser = argparse.ArgumentParser(description="Sweep modulation frequency")
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
        default="dt/sinara/mpll/+",
        help="The MQTT topic prefix (%(default)s)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=9293,
        help="Local port to listen on for streaming data (%(default)s)",
    )
    parser.add_argument(
        "--fmin",
        type=float,
        default=7e3,
        help="Low frequency (Hz) (%(default)s)",
    )
    parser.add_argument(
        "--fmax",
        type=float,
        default=300e3,
        help="High frequency (Hz) (%(default)s)",
    )
    parser.add_argument(
        "--sweep",
        type=float,
        default=10e3,
        help="Sweep (Hz per s) (%(default)s)",
    )
    parser.add_argument(
        "--amplitude",
        type=float,
        default=1.0,
        help="Amplitude (V) (%(default)s)",
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

        frames = []
        t = 1.28e-6
        u = 1 << 32
        try:
            await conf.set("/stream", f"{local_ip}:{args.port}")
            await conf.set("/activate", False)
            await conf.set("/mpll/offset", None)
            await conf.set("/mpll/repr", "Ba")
            await conf.set("/mpll/iir/Ba/max", args.fmin)
            await conf.set("/mpll/iir/Ba/min", args.fmin)
            await conf.set("/mpll/iir/Ba/ba", [[0, 0, 0], [1, -1, 0]])
            await conf.set("/mpll/amplitude/0", args.amplitude)
            await conf.set("/activate", True)
            await conf.set("/mpll/iir/Ba/max", args.fmax)
            await conf.set("/mpll/iir/Ba/u", args.sweep * t * 8)
            while True:
                frame = await stream.queue.get()
                body = frame.to_mu()
                frames.append(body)
                f = body["frequency"] / (u * t)
                demod = (body["demod"][:, :, 0] + 1j * body["demod"][:, :, 1]) * (
                    20.48 / 10 / u * 2
                )
                _logger.info(
                    "%g kHz: %g V @ %g turns",
                    f.mean() / 1e3,
                    np.absolute(demod[:, 0]).mean(),
                    body["phase"].mean() / u,
                )
                if f[-1] > args.fmax - 1:
                    break
        finally:
            await conf.set("/stream", "0.0.0.0:0")
            await conf.set("/mpll/repr", "Pid")

        body = np.concatenate(frames)
        _logger.warning("%s batches", body.shape[0])
        demod = (
            (body["demod"][:, :, 0] + 1j * body["demod"][:, :, 1])
            * (20.48 / 10 / u * 2)
            * np.exp(-20j * np.pi / u * body["frequency"][:, None])
        )
        mean = demod[:, 0].mean()
        angle = np.angle(mean) / (2 * np.pi)
        _logger.warning("IQ mean %g V @ %g turns", np.absolute(mean), angle)
        # algebraic least squares circle
        x = demod[:, 0]
        b = np.vstack((x.real**2 + x.imag**2, x.real, x.imag, np.ones_like(x.real)))
        _bu, bs, bv = np.linalg.svd(b.T, full_matrices=False)
        # assert np.fabs(bs[-1]) < 1 / 10.0
        b = bv[-1]  # right singular vector to lowest singular value
        center = -(b[1] + 1j * b[2]) / (2 * b[0])
        distance2 = (center * center.conj()).real
        radius = np.sqrt(distance2 - b[3] / b[0])
        angle = np.angle(center) / (2 * np.pi)
        _logger.warning(
            "IQ resonance %g V radius, center %g V @ %g turns",
            radius,
            np.sqrt(distance2),
            angle,
        )

        power = np.absolute(demod[:, 0]) ** 2
        fmean = (body["frequency"] * power).sum() / (power.sum() * t * u)
        _logger.warning("frequency mean %g kHz", fmean / 1e3)

        await conf.set("/activate", False)
        await conf.set("/mpll/iir/Pid/order", "I")
        await conf.set("/mpll/iir/Pid/gains/p", -1e3)
        await conf.set("/mpll/iir/Pid/gains/i", -5e5)
        # await conf.set("/mpll/iir/Pid/gains/d", -0.25)
        # await conf.set("/mpll/iir/Pid/limits/d", -150e3)
        await conf.set("/mpll/iir/Pid/setpoint", angle)
        await conf.set("/mpll/iir/Pid/min", fmean)
        await conf.set("/mpll/iir/Pid/max", fmean)
        await conf.set("/activate", True)
        await conf.set("/mpll/iir/Pid/min", args.fmin)
        await conf.set("/mpll/iir/Pid/max", args.fmax)


if __name__ == "__main__":
    asyncio.run(main())
