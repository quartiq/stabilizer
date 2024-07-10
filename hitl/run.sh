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

# Set up python for testing
python -m venv --system-site-packages .venv
. .venv/bin/activate

# Install Miniconf utilities for configuring stabilizer.
pip install -U setuptools
python -m pip install py/

probe-rs download --chip STM32H743ZITx --log-file /dev/null --probe 0483:3754:004C003D3137510D33333639 target/thumbv7em-none-eabihf/release/dual-iir 
probe-rs reset --chip STM32H743ZITx --log-file /dev/null --probe 0483:3754:004C003D3137510D33333639 --connect-under-reset

# Sleep to allow booting, DHCP, ARP, MQTT etc
sleep 30

# Test pinging Stabilizer. This exercises that:
# * DHCP is functional and an IP has been acquired
# * Stabilizer's network is functioning as intended
# * The stabilizer application is operational
ping -c 5 -w 20 stabilizer-hitl

# Test the MQTT interface. This uses the default broker "mqtt"
python -m miniconf $PREFIX '?'
python -m miniconf $PREFIX '/afe/0="G2"'
python -m stabilizer.iir_coefficients -p $PREFIX -c 0 -v pid --Ki 10 --Kp 1

# Test the ADC/DACs connected via loopback.
python hitl/loopback.py $PREFIX

# Test the livestream capabilities
python hitl/streaming.py $PREFIX
