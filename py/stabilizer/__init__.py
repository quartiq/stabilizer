#!/usr/bin/python3
"""
Author: QUARTIQ GmbH

Description: General utilities for interfacing with Stabilizer using Python.
"""

# The maximum output scale of the Stabilizer DACs.
DAC_MAX_SCALE = 4.096 * 2.5

def voltage_to_machine_units(voltage):
    """ Convert a voltage to IIR machine units.

    Args:
        voltage: The voltage to convert

    Returns:
        The IIR machine-units associated with the voltage.
    """
    assert abs(voltage) <= DAC_MAX_SCALE, 'Voltage out-of-range'
    return int(voltage / DAC_MAX_SCALE * 0x7FFF)
