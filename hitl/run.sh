#!/bin/bash

# Enable shell operating mode flags.
set -eux

PREFIX=dt/sinara/dual-iir/04-91-62-d9-7e-5f
PROBE=0483:3754:004C003D3137510D33333639

#PREFIX=dt/sinara/dual-iir/04-91-62-d9-81-ff
#PROBE=0483:3754:0016001B5553500A20393256

python3 -m venv --system-site-packages .venv
source .venv/bin/activate

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
    '/ch/0/gain="G2"' '/ch/0/biquad/0/typ="Pid"' \
    /ch/0/biquad/0/repr/Pid/setpoint=0 min=-30000 max=30000 gain/i=-10 gain/p=-0.1 \
    '/ch/0/run="Run"'

# Test the ADC/DACs connected via loopback.
python3 hitl/loopback.py $PREFIX

# Test the stream capabilities
python3 hitl/streaming.py $PREFIX
