# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

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
* Quick-start guide and book ([#431](https://github.com/quartiq/stabilizer/pull/431) [#430](https://github.com/quartiq/stabilizer/pull/430) [#401](https://github.com/quartiq/stabilizer/pull/401) [#404](https://github.com/quartiq/stabilizer/pull/404))
* Expanded and revised python API including the computation of IIR coefficients
  from PID gains ([#472](https://github.com/quartiq/stabilizer/pull/472) [#471](https://github.com/quartiq/stabilizer/pull/471) [#451](https://github.com/quartiq/stabilizer/pull/451) [#427](https://github.com/quartiq/stabilizer/pull/427))
* Device MQTT broker connection status indicated via MQTT ([#468](https://github.com/quartiq/stabilizer/pull/468) [#441](https://github.com/quartiq/stabilizer/pull/441))

### Changed

* Const generics, bumping the MSRV to 1.51.0 ([#355](https://github.com/quartiq/stabilizer/pull/355) etc)
* `lockin-internal` and `lockin-external` have been merged into `lockin` ([#352](https://github.com/quartiq/stabilizer/pull/352))
* Set target CPU to cortex-m7, effectively bumping the MSRV to 1.52.0
* Process routine can be executed from fast closely coupled memory with wide
  bus for faster execution ([#420](https://github.com/quartiq/stabilizer/pull/420))
* Relicensed as MIT/Apache ([#419](https://github.com/quartiq/stabilizer/pull/419) [#416](https://github.com/quartiq/stabilizer/pull/416))

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

[Unreleased]: https://github.com/quartiq/stabilizer/compare/v0.5.0...HEAD
[v0.5.0]: https://github.com/quartiq/stabilizer/compare/v0.4.1...v0.5.0
[v0.4.1]: https://github.com/quartiq/stabilizer/compare/v0.4.0...v0.4.1
[v0.4.0]: https://github.com/quartiq/stabilizer/compare/v0.3.0...v0.4.0
[v0.3.0]: https://github.com/quartiq/stabilizer/compare/v0.2.0...v0.3.0
[v0.2.0]: https://github.com/quartiq/stabilizer/compare/v0.1.0...v0.2.0
