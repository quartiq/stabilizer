# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## [Unreleased]

### Added

* Telemetry
* RTT logging

### Changed

* Const generics bumping the MSRV to 1.51.0
* `lockin-internal` and `lockin-external` have been merged into `lockin`

### Fixed

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
