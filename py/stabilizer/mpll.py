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
        "--fstart",
        type=float,
        default=7e3,
        help="Start frequency (Hz) (%(default)s)",
    )
    parser.add_argument(
        "--fstop",
        type=float,
        default=300e3,
        help="Stop frequency (Hz) (%(default)s)",
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
            await asyncio.sleep(1)
            await conf.set("/activate", False)
            await conf.set("/mpll/repr", "Ba")
            await conf.set("/mpll/iir/Ba/max", args.fstart)
            await conf.set("/mpll/iir/Ba/min", args.fstart)
            await conf.set("/mpll/iir/Ba/ba", [[0, 0, 0], [1, -1, 0]])
            await conf.set("/activate", True)
            if args.sweep >= 0:
                await conf.set("/mpll/iir/Ba/max", args.fstop)
            else:
                await conf.set("/mpll/iir/Ba/min", args.fstop)
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
                if abs(f[-1] - args.fstop) <= 1:
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

        if args.plot is not None:
            from matplotlib import pyplot as plt

            fig, ax = plt.subplots(2, 2, figsize=(10, 6))

        for ch in reversed(range(2)):  # ch0 last to set fmean/angle
            s = body["frequency"] / u
            g = demod[:, ch]
            b = np.array([g * s, g / s, 1 / s, 1j / s, 1 + 0 * s, 1j + 0 * s])
            b = (
                np.linalg.pinv(np.hstack((b.real * g.real, b.imag * g.imag)).T)
                @ np.hstack((g.real, g.imag)) ** 2
            )
            # f1b = 1/(2*b[0])
            r2 = b[4] ** 2 + b[5] ** 2
            f1 = -(b[2] * b[4] + b[3] * b[5]) / r2
            d1 = (b[2] * b[5] - b[3] * b[4]) / r2
            q1 = 2 * (-b[5] + 1j * b[4]) * f1 / d1
            b1 = (b[2] / d1 - b[5]) / b[4]
            a1 = (-2 * b[1] / d1 - b1) * b1
            x1 = (s - f1) / d1
            g1 = q1 * (1 + 1j * x1) / (x1**2 + a1)

            angle = np.angle(q1) / (2 * np.pi)
            fmean = f1 / t
            width = d1 / t
            _logger.warning(
                "ch%i IQ resonance %g V, %g kHz, %g turns, %g kHz FWHM",
                ch,
                np.absolute(q1),
                fmean / 1e3,
                angle,
                width / 1e3,
            )

            if args.plot is not None:
                ax[ch, 0].semilogx(s / t, g.real)
                ax[ch, 0].semilogx(s / t, g.imag)
                ax[ch, 0].semilogx(s / t, g1.real)
                ax[ch, 0].semilogx(s / t, g1.imag)
                ax[ch, 0].grid()
                ax[ch, 1].plot(g.real, g.imag)
                ax[ch, 1].plot(g1.real, g1.imag)
                ax[ch, 1].set_aspect("equal")
                ax[ch, 1].grid()

        if args.plot is not None:
            fig.savefig(args.plot)

        await conf.set("/activate", False)
        await conf.set("/mpll/iir/Pid/order", "I")
        await conf.set("/mpll/iir/Pid/gains/p", -3e3)
        await conf.set("/mpll/iir/Pid/gains/i", -7e6)
        await conf.set("/mpll/iir/Pid/setpoint", angle)
        await conf.set("/mpll/iir/Pid/min", fmean)
        await conf.set("/mpll/iir/Pid/max", fmean)
        await conf.set("/activate", True)
        await conf.set("/mpll/iir/Pid/min", min(args.fstart, args.fstop))
        await conf.set("/mpll/iir/Pid/max", max(args.fstart, args.fstop))


if __name__ == "__main__":
    asyncio.run(main())
