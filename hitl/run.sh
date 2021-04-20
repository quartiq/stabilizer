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
. py/bin/activate
python3 -m pip install -r requirements.txt

cargo flash --elf target/thumbv7em-none-eabihf/release/dual-iir --chip STM32H743ZITx

sleep 30

# Test pinging Stabilizer. This exercises that:
# * DHCP is functional and an IP has been acquired
# * Stabilizer's network is functioning as intended
# * The stabilizer application is operational
ping -c 5 -w 45 stabilizer-hitl

# Test the MQTT interface.
python3 miniconf.py dt/sinara/stabilizer afe/0='"G2"'
python3 miniconf.py dt/sinara/stabilizer afe/0='"G1"' iir_ch/0/0=\
'{"y_min": -32767, "y_max": 32767, "y_offset": 0, "ba": [1.0, 0, 0, 0, 0]}'
