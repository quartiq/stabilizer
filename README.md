# Stabilizer Firmware

![Flow diagram](stabilizer_pid.svg)

![Hardware](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)

## Features

* dual channel
* SPI ADC
* SPI DAC
* 500 kHz rate, timed sampling
* 2 µs latency, unmatched between channels
* f32 IIR math
* generic biquad (second order) IIR filter
* anti-windup
* derivative kick avoidance

## Limitations/TODOs

* Fixed AFE gains
* The IP and MAC address are [hardcoded](src/main.rs)
* Expose configurable limits
* 100Base-T only
* Digital IO, GPIO header, AFE header, EEM header are not handled

## Hardware

See https://github.com/sinara-hw/Stabilizer

## Minimal bootstrapping documentation

* Clone or download this
* Get a recent openocd, a JTAG adapter ("st-link" or some clone) and
  everything connected and permissions setup
* Get a multiarch `gdb` (or a cross arm gdb and edit `.cargo/config` accordingly)
* Get [rustup](https://rustup.rs/)
* `rustup override add nightly`
* `rustup target add thumbv7em-none-eabihf`
* `openocd -f stabilizer.cfg` and leave it running
* `cargo run --release`


## Protocol

Stabilizer can be configured via newline-delimited JSON over TCP.
It listens on port 1235. [stabilizer.py](stabilizer.py) contains a reference
implementation of the protocol.
