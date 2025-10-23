# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

### [UNRELEASED](https://github.com/quartiq/stabilizer/compare/v0.11.0...HEAD) - DATE

### Added

* Support for exponentially swept sine signal source for transfer function
  and distortion measurement
* Support for "cooked" PID biquad and standard biquad filter prototypes from `idsp`
* Defaulting to `s` optimization for debug and release
* `fls`: application for fiber length stabilization and phase/frequency measurement/control

### Changed

* `py`: `StabilizerStream` renamed to `Stream`
* `dual-iir`: settings tree layout changed
* `urukul`: bin target renamed to `dds`

## [v0.11.0](https://github.com/quartiq/stabilizer/compare/v0.10.0...v0.11.0) - 2024-12-02

### Added

* Support for Urukul-AD9912 as a downstream EEM

### Changed

* Various dependencies bumped (among them Miniconf v0.13 -> v0.18)

## [v0.10.0](https://github.com/quartiq/stabilizer/compare/v0.9.0...v0.10.0)

### Added

* Serial terminal is available on USB for settings configurations
* Reboot to DFU support added via the serial terminal for remote bootloading
* The `meta` topic now contains metadata about the compiler, firmware, and hardware similar to
Booster
* Panic information is now persisted after reboot and available via telemetry and the USB serial
console.
* Device operational settings can now be modified, stored and cleared from device flash using the
  USB serial console. Run-time settings are unique to each application, but network settings are
  unified for all applications (i.e. lockin, dual-iir, etc.)

### Changed

* Broker and static IP/DHCP are no longer configured at compile time,
  but is maintained in device flash and can be changed via the USB port.
* MSRV removed. Stabilizer uses latest stable rust.
* The IIR (biquad) filter used for PID action has changed its serialization format.
  See also the `iir_coefficients` Python CLI implementation.
* The stream target is now configures as a `1.2.3.4:4321` string

### Fixed

* Fixed an issue where the device would sometimes not enumerate on Windows

## [v0.9.0](https://github.com/quartiq/stabilizer/compare/v0.8.1...v0.9.0)

### Fixed

* Fixed a defect where powering up with Pounder attached would cause an internal panic.

### Changed

* `idsp` crate update to 0.10: `lockin` now uses a double second order lowpass.
* The `batch_size` field in the the UDP stream frame now contains the number of batches
  not the number of samples per batch. It has been renamed to `batches`.
* All MQTT clients upgraded and APIs updated.
* MQTT broker may now be specified via DNS hostnames
* `hitl/streaming.py` no longer requires a prefix
* Streaming now supports UDP multicast addresses

## [v0.8.1](https://github.com/quartiq/stabilizer/compare/v0.8.0...v0.8.1) - 2022-11-14

* Fixed the python package dependencies

## [v0.8.0](https://github.com/quartiq/stabilizer/compare/v0.7.0...v0.8.0) - 2022-11-07

* [breaking] MSRV bumped to 1.63 (`array_from_fn` via `enum_iterator`)
* [breaking] Bumped dependencies on `miniconf`, `minimq` leading to changes in the `Miniconf` settings API
* Bumped HAL dependency to fix a crucial I2C bug
* Added gain limits for `iir_coefficients`
* Added a IIR transfer function plotting tool
* Fixed a bug in the Python telemetry client that led to no data being returned
* EEM Gpio pins are now available from `setup()`
* Added white noise output to `signal_generator`

## [v0.7.0] - 2022-08-10

### Added

* Optional specification of a static IP address ([#509](https://github.com/quartiq/stabilizer/pull/509))
* Temperature sensor measurement for Pounder and Stabilizer's CPU has been added [#575](https://github.com/quartiq/stabilizer/pull/575)
* External clock support for Pounder
* Auxiliary ADC support for Pounder

### Removed

* Minimum supported Rust version bumped to 1.62
* The `pounder_v1_1` feature has been replaced by the `pounder_v1_0` feature with inverse meaning.
* The `Telemetry` python client has been integrated into the `stabilizer` Python library

### Fixed

* Fixed panics when using a batch size of 1 ([#540](https://github.com/quartiq/stabilizer/issues/540))
* Fixed an issue where startup delays were 1/4 of what they should be ([#524](https://github.com/quartiq/stabilizer/issues/524))
* Fixed Pounder GPIO extender issues by rewriting the `mcp280xx` crate
* Fixed improper calculation of the signal generator phase increment ([#543](https://github.com/quartiq/stabilizer/pull/543))

## [v0.6.0] - 2022-02-11

### Added

* Telemetry ([#341](https://github.com/quartiq/stabilizer/pull/341))
* Logging via RTT (real time tracing) over SWD/JTAG instead of semihosting
  for fast and low-overhead debugging ([#393](https://github.com/quartiq/stabilizer/pull/393) [#391](https://github.com/quartiq/stabilizer/pull/391) [#358](https://github.com/quartiq/stabilizer/pull/358))
* Settings structures now automatically publish after a short delay on boot
  ([#475](https://github.com/quartiq/stabilizer/pull/475))
* Full-rate data streaming (e.g. complete raw ADC/DAC data) via Ethernet/UDP
  ([#414](https://github.com/quartiq/stabilizer/pull/414) [#394](https://github.com/quartiq/stabilizer/pull/394) [#380](https://github.com/quartiq/stabilizer/pull/380))
* Compilation specifically targeting Cortex-M7 instruction set for faster
  code ([#358](https://github.com/quartiq/stabilizer/pull/358))
* Fast double-buffered DMA pattern for less overhead ([#367](https://github.com/quartiq/stabilizer/pull/367))
* Integrated signal generator for stimulus and scan waveform generation ([#434](https://github.com/quartiq/stabilizer/pull/434)
  [#388](https://github.com/quartiq/stabilizer/pull/388))
* Expanded code documentation ([#452](https://github.com/quartiq/stabilizer/pull/452) [#346](https://github.com/quartiq/stabilizer/pull/346))
* Quick-start guide and book ([#431](https://github.com/quartiq/stabilizer/pull/431) [#430](https://github.com/quartiq/stabilizer/pull/430) [#401](https://github.com/quartiq/stabilizer/pull/401) [#404](https://github.com/quartiq/stabilizer/pull/404) [#483](https://github.com/quartiq/stabilizer/pull/483))
* Expanded and revised python API including the computation of IIR coefficients
  from PID gains ([#472](https://github.com/quartiq/stabilizer/pull/472) [#471](https://github.com/quartiq/stabilizer/pull/471) [#451](https://github.com/quartiq/stabilizer/pull/451) [#427](https://github.com/quartiq/stabilizer/pull/427))
* Device MQTT broker connection status indicated via MQTT ([#468](https://github.com/quartiq/stabilizer/pull/468) [#441](https://github.com/quartiq/stabilizer/pull/441))
* The signal generator supports adjusting its phase ([#505](https://github.com/quartiq/stabilizer/pull/505))

### Changed

* Const generics, bumping the MSRV to 1.51.0 ([#355](https://github.com/quartiq/stabilizer/pull/355) etc)
* `lockin-internal` and `lockin-external` have been merged into `lockin` ([#352](https://github.com/quartiq/stabilizer/pull/352))
* Set target CPU to cortex-m7
* Process routine can be executed from fast closely coupled memory with wide
  bus for faster execution ([#420](https://github.com/quartiq/stabilizer/pull/420))
* Relicensed as MIT/Apache ([#419](https://github.com/quartiq/stabilizer/pull/419) [#416](https://github.com/quartiq/stabilizer/pull/416))
* Minimum supported Rust version bumped to 1.57

### Fixed

* Minimized DAC output pulses during boot ([#453](https://github.com/quartiq/stabilizer/pull/453))

## [v0.5.0] - 2021-04-21

### Added

* Batch sample processing
* DMA for ADC and DAC batches
* Pounder profile streaming
* DSP library with lots of optimized algorithms
* Digital input support
* Hardware in the loop continuous integration testing
* Dependency updates
* MQTT settings interface through miniconf/minimq
* Multi-binary support
* DHCP support

### Changed

* Removed JSON-over-TCP interface

### Fixed

* Robust EEPROM MAC address reading slow supply start

## [v0.4.1] - 2020-06-23

### Fixed

* Fix DAC clr/ldac, SPI speed

## [v0.4.0] - 2020-06-22

### Added

* Hardware v1.1 only
* AD9959/Pounder support

### Changed

* HAL port

## [v0.3.0] - 2020-01-20

### Added

* Red LED handling
* EEPROM MAC address reading

### Changed

* Panic handler cleanup
* Dependency updates (smoltcp, rtfm)

## [v0.2.0] - 2019-05-28

### Added

* Initial basic release
* Ethernet support
* IIR filter code
* ADC and DAC SPI support
* ADC/DAC timing and interrupts
* Board configuration, bootstrap

## [v0.1.0] - 2019-03-10

### Added

* First bits of code published

[v0.7.0]: https://github.com/quartiq/stabilizer/compare/v0.6.0...v0.7.0
[v0.6.0]: https://github.com/quartiq/stabilizer/compare/v0.5.0...v0.6.0
[v0.5.0]: https://github.com/quartiq/stabilizer/compare/v0.4.1...v0.5.0
[v0.4.1]: https://github.com/quartiq/stabilizer/compare/v0.4.0...v0.4.1
[v0.4.0]: https://github.com/quartiq/stabilizer/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/quartiq/stabilizer/compare/v0.2.0...v0.3.0
[v0.2.0]: https://github.com/quartiq/stabilizer/compare/v0.1.0...v0.2.0
