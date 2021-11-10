#!/usr/bin/python3
"""Stabilizer data conversion and streaming utilities"""

# The number of DAC LSB codes per volt on Stabilizer outputs.
DAC_LSB_PER_VOLT = (1 << 16) / (4.096 * 5)

# The number of volts per ADC LSB.
ADC_VOLTS_PER_LSB = (5.0 / 2.0 * 4.096)  / (1 << 15)

# The number of volts per DAC LSB.
DAC_VOLTS_PER_LSB = 1 / DAC_LSB_PER_VOLT

# The absolute full-scale output voltage in either positive or negative direction exposed by the
# DAC.
DAC_FULL_SCALE = float(0x7FFF / DAC_LSB_PER_VOLT)

def voltage_to_machine_units(voltage):
    """Convert a voltage to machine units."""
    code = int(round(voltage * DAC_LSB_PER_VOLT))
    if abs(code) > 0x7FFF:
        raise ValueError(f"Voltage out-of-range ({hex(code)}")
    return code
