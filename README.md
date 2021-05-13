[![QUARTIQ Matrix Chat](https://img.shields.io/matrix/quartiq:matrix.org)](https://matrix.to/#/#quartiq:matrix.org)
[![Continuous Integration](https://github.com/quartiq/stabilizer/actions/workflows/ci.yml/badge.svg)](https://github.com/quartiq/stabilizer/actions/workflows/ci.yml)
[![Stabilizer HITL [Nightly]](https://github.com/quartiq/hitl/actions/workflows/stabilizer-nightly.yml/badge.svg)](https://github.com/quartiq/hitl/actions/workflows/stabilizer-nightly.yml)

# Stabilizer Firmware

## Hardware

[![Hardware](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)](https://github.com/sinara-hw/Stabilizer)

## Applications

This firmware offers a library of hardware and software functionality targeting the use of the Stabilizer hardware in various digital signal processing applications commonly occurring in Quantum Technology applications.
It provides abstractions over the fast analog inputs and outputs, time stamping, Pounder DDS interfaces and a collection of tailored and optimized digital signal processing algorithms (IIR, FIR, Lockin, PLL, reciprocal PLL, Unwrapper, Lowpass, Cosine-Sine, Atan2).
An application can compose and configure these hardware and software components to implement different use cases.
Several applications are provides by default:

### Dual-IIR

![Flow diagram](stabilizer_pid.svg)

* dual channel
* SPI ADC
* SPI DAC
* up to 800 kHz rate, timed sampling
* down to 2 µs latency
* f32 IIR math
* generic biquad (second order) IIR filter
* anti-windup
* derivative kick avoidance

### Lockin

* Up to 800 kHz sampling
* Up to 400 kHz modulation frequency
* Reciprocal PLL for external reference
* Internal reference
* Adjustable PLL and locking time constants
* Adjustable phase offset and harmonic index
* Different output modes (in-phase, quadrature, magnitude, log2 power, phase, frequency)

## Minimal bootstrapping documentation

* Clone or download this
* Get [rustup](https://rustup.rs/)
* Minimum supported Rust version (MSRV) is 1.52.0
* Install target support: `rustup target add thumbv7em-none-eabihf`
* Install `probe-run`: `cargo install probe-run`
* `cargo run --release --bin dual-iir`
* When using debug (non `--release`) mode, increase the sample interval significantly.
  The added error checking code and missing optimizations may lead to the code
  missing deadlines and panicing.

## Alternative flashing tools

### Cargo-embed

* Install `cargo-embed`: `cargo install cargo-embed`
* Program the device: `cargo embed --bin dual-iir --release`

### GDB/OpenOCD

* Get a recent openocd, a JTAG adapter ("st-link" or some clone) and
  everything connected and permissions setup. Most
  [Nucleo](https://www.digikey.de/short/p41h4v) boards have a
  detachable ST-Link v2 and are cheap.[^swd]
* Get a multiarch `gdb` (or a cross arm gdb and edit `.cargo/config` accordingly)
* `openocd -f stabilizer.cfg` and leave it running
* `cargo run --release`

### USB-DFU

* Get [cargo-binutils](https://github.com/rust-embedded/cargo-binutils/)
* `cargo objcopy --release --bin dual-iir -- -O binary dual-iir.bin` or `arm-none-eabi-objcopy -O binary target/thumbv7em-none-eabihf/release/dual-iir dual-iir.bin`
* Install the DFU USB tool (`dfu-util`)
* Connect to the Micro USB connector below the RJ45
* Short JC2/BOOT
* `dfu-util -a 0 -s 0x08000000:leave -D dual-iir.bin`

### ST-Link virtual mass storage

* Prepare `dual-iir.bin` like above
* Connect the ST-Link debugger
* Copy `dual-iir.bin` to the `NODE_H743ZI` virtual mass storage device

## Protocol

Stabilizer can be configured via MQTT. Refer to
[`miniconf`](https://github.com/quartiq/miniconf) for more information about topics.
A basic command line interface is available in [`miniconf.py`](miniconf.py).
Telemetry is published via MQTT as well.
