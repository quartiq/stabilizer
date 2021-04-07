#!/bin/bash

# Title:
#   Stabilizer hardware-in-the-loop (HITL) test script.
#
# Description:
#   This shell file is executed by the hardware runner in Quartiq's office to exercise the various
#   hardware aspects of Stabilizer.

# Enable shell operating mode flags.
set -eux

# Set up python for testing
python3 -m venv --system-site-packages py

py/bin/pip install -r requirements.txt

# Test pinging Stabilizer. This exercises that:
# * DHCP is functional and an IP has been acquired
# * Stabilizer's network is functioning as intended
# * The stabilizer application is operational
ping -c 5 -w 20 stabilizer-hitl

# Test the MQTT interface.
py/bin/python3 miniconf.py dt/sinara/stabilizer afe/0 '"G2"'
