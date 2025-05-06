#!/usr/bin/python3
"""
Authors:
  * Vertigo Designs, Ryan Summers
  * QUARTIQ GmbH, Robert Jördens

Description: Loop-back integration tests for Stabilizer hardware
"""
import argparse
import asyncio
import sys
import logging
import json
import os

from miniconf.common import MQTTv5, one
from miniconf import Client, Miniconf, discover

if sys.platform.lower() == "win32" or os.name.lower() == "nt":
    from asyncio import set_event_loop_policy, WindowsSelectorEventLoopPolicy

    set_event_loop_policy(WindowsSelectorEventLoopPolicy())


async def test_loopback(stabilizer, telemetry_queue, set_point, gain=1, channel=0):
    """Test loopback operation of Stabilizer.

    Connect ADC and DAC of the respective channel and ensure no mezzanine present.
    """
    print(f"Testing PID loopback for set point = {set_point:.2f}, gain = x{gain}")
    print("---------------------------------")
    for k, v in {
        "gain": f"G{gain}",
        # "biquad/0/typ": "Raw",
        # "biquad/0/repr/Raw": {
        #     "ba": [0, 0, 0, 0, 0],
        #     "u": 0,
        #     "min": u,
        #     "max": u,
        # },
        "biquad/0/typ": "Pid",
        "biquad/0/repr/Pid/order": "I",
        "biquad/0/repr/Pid/gain/i": -1000,
        "biquad/0/repr/Pid/gain/p": -0.1,
        "biquad/0/repr/Pid/min": -10,
        "biquad/0/repr/Pid/max": 10,
        "biquad/0/repr/Pid/setpoint": set_point * gain,
        "source/amplitude": 0,
        "source/offset": 0,
        "run": "Run",
    }.items():
        await stabilizer.set(f"/ch/{channel}/{k}", v)
    await stabilizer.set(f"/trigger", True)
    await telemetry_queue.__anext__()  # discard
    latest_values = json.loads((await telemetry_queue.__anext__()).payload)
    print(f"Latest telemtry: {latest_values}")
    assert abs(latest_values["adcs"][channel] - set_point) < max(0.01 * set_point, 0.01)
    print("PASS")
    print("")


def main():
    """Main program entry point."""
    parser = argparse.ArgumentParser(
        description="Loopback tests for Stabilizer HITL testing",
    )
    parser.add_argument(
        "prefix", type=str, nargs="?", help="The MQTT topic prefix of the target"
    )
    parser.add_argument(
        "--broker", "-b", default="mqtt", type=str, help="The MQTT broker address"
    )

    args = parser.parse_args()

    async def test():
        """The actual testing being completed."""
        async with Client(
            args.broker, protocol=MQTTv5, logger=logging.getLogger("aiomqtt-client")
        ) as client:
            if not args.prefix:
                prefix, _alive = one(await discover(client, "dt/sinara/dual-iir/+"))
                logging.info("Found device prefix: %s", prefix)
            else:
                prefix = args.prefix

            stabilizer = Miniconf(client, prefix)

            async with Client(
                args.broker,
                protocol=MQTTv5,
                queue_type=asyncio.LifoQueue,
                max_queued_incoming_messages=1,
            ) as tele:
                await tele.subscribe(f"{prefix}/telemetry")
                await stabilizer.set("/telemetry_period", 1)
                await test_loopback(stabilizer, tele.messages, 1.0)
                await test_loopback(stabilizer, tele.messages, 1.0, gain=2)
                await test_loopback(stabilizer, tele.messages, 0.0)

    sys.exit(asyncio.run(test()))


if __name__ == "__main__":
    main()
