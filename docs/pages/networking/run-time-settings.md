---
title: Run-Time Settings
layout: default
permalink: /networking/run-time-settings
parent: Networking
---

# Settings Configuration

Stabilizer allows for run-time settings configurations using the `miniconf.py` utility script. This
script is in the root of the Stabilizer github repository.

## Setup

In order to use `miniconf.py`, run the following command:
```
python -m pip install gmqtt
```

## Usage
The `miniconf.py` script utilizes a unique "device prefix". The device prefix is always of the
form `dt/sinara/<app>/<mac-address>`, where `<app>` is the name of the application and
`<mac-address>` is the MAC address of the device, formatted with delimiting dashes.

Settings have a `path` and a `value` being configured. The `value` parameter is JSON-encoded data
and the `path` value is a path-like string.

As an example, for configuring `dual-iir`'s `stream_target`, the following information would be
used:
* `path` = `stream_target`
* `value` = `{"ip": [192, 168, 0, 1], "port": 4000}`

```
python miniconf.py --broker localhost dt/sinara/dual-iir/00-11-22-33-44-55 stream_target='{"ip": [192, 168, 0, 1], "port": 4000}'
```

The prefix can be found for a specific device by looking at the topic on which telemetry that is
being published.

Refer to the [application documentation](/#applications) for the exact settings and values exposed
for each application.

The rules for constructing `path` values are documented in [`miniconf`'s
documentation](https://github.com/quartiq/miniconf#settings-paths)

Refer to the documentation for [Miniconf]({{site.baseurl}}/firmware/miniconf/enum.Error.html) for a
description of the possible error codes that `miniconf.py` may return if the settings update was
unsuccessful.
