# Driver debugging notes

- LN channel DAC reference voltages at +4.1,-5.0 V wrt DACGND
- HP channel DAC reference voltages at +5.0,-5.0 V wrt DACGND
- LN DAC output default at -0.9 V wrt DACGND
- HP DAC output default at -5 V wrt DACGND

- At writes, the DACs relay MOSI traffic on the MISO line without delay (But it somehow seems to set the read bit??). I checked on the scope, that the read bit on MOSI is clear.
- At reads, the DACs also instantly relay MOSI traffic on the MISO line, but the append other data at the read cycle (as expected). I checked on the scope, that the read bit is set.

- DAC defaults to synchronous updates, which means we have to change that (which the code is supposed to do) or use the LDAC pin 
- I tried a negative edge on LDAC after loading in new DAC values, but that did not change anything. 
