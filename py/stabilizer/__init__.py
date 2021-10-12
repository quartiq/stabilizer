#!/usr/bin/python3
"""
Author: QUARTIQ GmbH

Description: General utilities for interfacing with Stabilizer using Python.
"""

# The number of DAC LSB codes per volt on Stabilizer outputs.
DAC_LSB_PER_VOLT = (1 << 16) / (4.096 * 5)

# The absolute full-scale output voltage in either positive or negative direction exposed by the
# DAC.
DAC_FULL_SCALE = DAC_LSB_PER_VOLT / (1 << 16) / 2

def voltage_to_machine_units(voltage):
    """ Convert a voltage to machine units.

    Args:
        voltage: The voltage to convert

    Returns:
        The machine-units associated with the voltage.
    """
    code = int(round(voltage * DAC_LSB_PER_VOLT))
    assert abs(code) <= 0x7FFF, 'Voltage out-of-range'
    return code
