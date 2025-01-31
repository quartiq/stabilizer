#!/bin/bash

# Title:
#   Stabilizer hardware-in-the-loop (HITL) test script.
#
# Description:
#   This shell file is executed by the hardware runner in Quartiq's office to exercise the various
#   hardware aspects of Stabilizer.

# Enable shell operating mode flags.
set -eux

# Stabilizer device prefix for HITL
PREFIX=dt/sinara/dual-iir/04-91-62-d9-7e-5f
PROBE=0483:3754:004C003D3137510D33333639

#PREFIX=dt/sinara/dual-iir/04-91-62-d9-81-ff
#PROBE=0483:3754:0016001B5553500A20393256

# Set up python for testing
python3 -m venv --system-site-packages .venv
. .venv/bin/activate

# Install Miniconf utilities for configuring stabilizer.
python3 -m pip install -U setuptools pip
python3 -m pip install py/

probe-rs download --chip STM32H743ZITx --log-file /dev/null --probe $PROBE target/thumbv7em-none-eabihf/release/dual-iir
probe-rs reset --chip STM32H743ZITx --log-file /dev/null --probe $PROBE --connect-under-reset

# Sleep to allow booting, DHCP, ARP, MQTT etc
sleep 30

# Test pinging Stabilizer. This exercises that:
# * DHCP is functional and an IP has been acquired
# * Stabilizer's network is functioning as intended
# * The stabilizer application is operational
ping -c 5 -w 20 stabilizer-hitl

# Test the MQTT interface. This uses the default broker "mqtt"
python3 -m miniconf $PREFIX '?'
python3 -m miniconf $PREFIX \
    '/ch/0/gain="G2"' 'biquad/0/typ="Pid"' \
    /ch/0/biquad/0/repr/Pid/ki=-10 kp=-0.1 setpoint=0 min=-30000 max=30000 \
    '/ch/0/run="Run"'

# Test the ADC/DACs connected via loopback.
python3 hitl/loopback.py $PREFIX

# Test the stream capabilities
python3 hitl/streaming.py $PREFIX
