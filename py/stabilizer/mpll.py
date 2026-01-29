"""MPLL calibration/sweep/searcher"""

# pylint: disable=logging-fstring-interpolation,too-many-statements,too-many-locals,duplicate-code


import asyncio
import argparse
import logging

import numpy as np

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
        "--plot", type=str, default=None, help="Plot filename (%(default)s)"
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
            orig_stream = await conf.get("/stream")
            await conf.set("/stream", f"{local_ip}:{args.port}")
            await conf.set("/activate", False)
            await conf.set("/mpll/repr", "Ba")
            await conf.set("/mpll/iir/Ba/max", args.fmin)
            await conf.set("/mpll/iir/Ba/min", args.fmin)
            await conf.set("/mpll/iir/Ba/ba", [[0, 0, 0], [1, -1, 0]])
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
                _logger.debug(
                    "phase %s p %s d",
                    body["phase"] / u,
                    np.angle(demod[:, 0]) / (2 * np.pi),
                )
                if f[-1] > args.fmax - 1:
                    break
        finally:
            await conf.set("/stream", orig_stream)
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
        g = demod[:, 0]
        s = body["frequency"] * (2j * np.pi / u)
        b = np.array([g * s, g / s, -1 + 0 * g, -1j + 0 * g])
        b = np.linalg.pinv(np.hstack((b.real, b.imag)).T) @ -np.hstack((g.real, g.imag))
        q = b[2] + 1j * b[3]
        g1 = q / (s * b[0] + b[1] / s + 1)
        angle = np.angle(q) / (2 * np.pi)
        fmean = np.sqrt(b[1] / b[0]) / (2 * np.pi * t)
        _logger.warning(
            "IQ resonance %g V, %g kHz, %g turns, %g kHz FWHM",
            np.absolute(q),
            fmean / 1e3,
            angle,
            fmean / 1e3 / b[0],
        )

        if args.plot is not None:
            from matplotlib import pyplot as plt

            fig, ax = plt.subplots(1, 2, figsize=(10, 6))
            f = s.imag / (2 * np.pi * t)
            ax[0].semilogx(f, g.real)
            ax[0].semilogx(f, g.imag)
            ax[0].semilogx(f, g1.real)
            ax[0].semilogx(f, g1.imag)
            ax[0].grid()
            ax[1].plot(g.real, g.imag)
            ax[1].plot(g1.real, g1.imag)
            ax[1].set_aspect("equal")
            ax[1].grid()
            fig.savefig(args.plot)

        await conf.set("/activate", False)
        await conf.set("/mpll/iir/Pid/order", "I")
        await conf.set("/mpll/iir/Pid/gains/p", -3e3)
        await conf.set("/mpll/iir/Pid/gains/i", -7e6)
        await conf.set("/mpll/iir/Pid/setpoint", angle)
        await conf.set("/mpll/iir/Pid/min", fmean)
        await conf.set("/mpll/iir/Pid/max", fmean)
        await conf.set("/activate", True)
        await conf.set("/mpll/iir/Pid/min", args.fmin)
        await conf.set("/mpll/iir/Pid/max", args.fmax)


if __name__ == "__main__":
    asyncio.run(main())
