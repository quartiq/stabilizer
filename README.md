# Stabilizer Firmware

![Flow diagram](stabilizer_pid.svg)

![Hardware](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)

## Features

* dual channel
* SPI ADC
* SPI DAC
* fixed AFE gains
* 500 kHz rate, timed
* < 2 µs latency, unmatched
* f32 IIR math
* generic biquad (second order) IIR filter
* anti-windup
* derivative kick avoidance
* configurable output limits

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
