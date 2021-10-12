# Miniconf Run-time Settings
Stabilizer supports run-time settings configuration using MQTT.

Settings can be stored in the MQTT broker so that they are automatically applied whenever
Stabilizer reboots and connects. This is referred to as "retained" settings. Broker implementations
may optionally store these retained settings as well such that they will be reapplied between
restarts of the MQTT broker.

Settings are specific to a device. Any settings configured for one Stabilizer will not be applied
to another. Disambiguation of devices is done by using Stabilizer's MAC address.

Settings are specific to an application. If two identical settings exist for two different
applications, each application maintains its own independent value.

## Installation
Install the Miniconf configuration utilities using a virtual environment:
```
python -m venv --system-site-packages vpy

# Refer to https://docs.python.org/3/tutorial/venv.html for more information on activating the
# virtual environment. This command is different on different platforms.
./vpy/Scripts/activate
```

Next, install prerequisite packages
```
python -m pip install -r scripts/requirements.txt
```

To use `miniconf`, execute it as follows:
```
python -m miniconf --help
```

Miniconf also exposes a programmatic Python API, so it's possible to write automation scripting of
Stabilizer as well.

## Usage
The Miniconf Python utility utilizes a unique "device prefix". The device prefix is always of the
form `dt/sinara/<app>/<mac-address>`, where `<app>` is the name of the application and
`<mac-address>` is the MAC address of the device, formatted with delimiting dashes.

Settings have a `path` and a `value` being configured. The `value` parameter is JSON-encoded data
and the `path` value is a path-like string.

As an example, for configuring `dual-iir`'s `stream_target`, the following information would be
used:
* `path` = `stream_target`
* `value` = `{"ip": [192, 168, 0, 1], "port": 4000}`

```
python -m miniconf --broker 10.34.16.10 dt/sinara/dual-iir/00-11-22-33-44-55 stream_target='{"ip": [10, 34, 16, 123], "port": 4000}'

Where `10.34.16.10` is the MQTT broker address that matches the one configured in the source code and `10.34.16.123` and `4000` are the desire stream target IP and port.
```

The prefix can be found for a specific device by looking at the topic on which telemetry that is
being published.

Refer to the [application documentation](overview.md#applications) for the exact settings and values exposed
for each application.

The rules for constructing `path` values are documented in [`miniconf`'s
documentation](https://github.com/quartiq/miniconf#settings-paths)

Refer to the documentation for [Miniconf](firmware/miniconf/enum.Error.html) for a
description of the possible error codes that Miniconf may return if the settings update was
unsuccessful.

# Telemetry

Stabilizer applications publish telemetry utilizes MQTT for managing run-time settings configurations as well as live telemetry
reporting.

Telemetry is defined as low rate, general health information. It is not intended for high throughput
or efficiency. Telemetry is generally used to determine that the device is functioning nominally.

Stabilizer applications publish telemetry over MQTT at a set rate. Telemetry data units are defined
by the application. All telemetry is reported using standard JSON format.

Telemetry is intended for low-bandwidth monitoring. It is not intended to transfer large amounts of
data and uses a minimal amount of bandwidth. Telemetry is published using "best effort" semantics -
individual messages may be dropped or Stabilizer may fail to publish telemetry due to internal
buffering requirements.

In its most basic form, telemetry publishes the latest ADC input voltages, DAC output voltages, and
digital input states.

Refer to the respective [application documentation](overview.md#applications) for more information on telemetry.

# Livestream

Stabilizer supports livestream capabilities for streaming real-time data over UDP. The livestream is
intended to be a high-bandwidth mechanism to transfer large amounts of data from Stabilizer to a
host computer for further analysis.

Livestreamed data is sent with "best effort" - it's possible that data may be lost either due to
network congestion or by Stabilizer.

Refer to the the respective [application documentation](overview.md#applications) for more information.
