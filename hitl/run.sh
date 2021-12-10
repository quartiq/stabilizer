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
python3 -m venv --system-site-packages vpy
. vpy/bin/activate

# Install Miniconf utilities for configuring stabilizer.
python3 -m pip install -e py/
python3 -m pip install git+https://github.com/quartiq/miniconf#subdirectory=py/miniconf-mqtt
python3 -m pip install gmqtt

cargo flash --chip STM32H743ZITx --elf target/thumbv7em-none-eabihf/release/dual-iir

# Sleep to allow flashing, booting, DHCP, MQTT
sleep 30

# Test pinging Stabilizer. This exercises that:
# * DHCP is functional and an IP has been acquired
# * Stabilizer's network is functioning as intended
# * The stabilizer application is operational
ping -c 5 -w 20 stabilizer-hitl

# Test the MQTT interface.
python3 -m miniconf dt/sinara/dual-iir/04-91-62-d9-7e-5f afe/0='"G2"'
python3 -m stabilizer.iir_coefficients -p dt/sinara/dual-iir/04-91-62-d9-7e-5f -c 0 -v pid --Ki 10 --Kp 1

# Test the ADC/DACs connected via loopback.
python3 hitl/loopback.py dt/sinara/dual-iir/04-91-62-d9-7e-5f

# Test the livestream capabilities
python3 hitl/streaming.py dt/sinara/dual-iir/04-91-62-d9-7e-5f
