#!/usr/bin/python3
# Small script that continuously publishes on a selected mqtt topic.
import argparse
from time import sleep
from paho.mqtt.client import Client as MqttClient


def main():
    """Main program entry point."""
    parser = argparse.ArgumentParser(description="record thermostat-eem data")
    parser.add_argument(
        "--broker", "-b", type=str, default="10.42.0.1", help="The MQTT broker."
    )
    parser.add_argument(
        "--topic",
        "-t",
        type=str,
        default="dt/sinara/thermostat-eem/80-1f-12-63-84-1b/interlock",
        help="The topic on which to publish"
        "dt/sinara/thermostat-eem/80-1f-12-63-84-1b/interlock",
    )
    parser.add_argument(
        "--delay", "-d", type=float, default=0.1, help="delay between publishes"
    )

    args = parser.parse_args()

    print(args.topic)

    client = MqttClient(client_id="11")
    client.connect(args.broker)
    while True:
        sleep(args.delay)
        client.publish(args.topic, 1)
        print(f"published onto {args.topic}")


if __name__ == "__main__":
    main()
