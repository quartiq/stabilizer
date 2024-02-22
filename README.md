[![QUARTIQ Matrix Chat](https://img.shields.io/matrix/quartiq:matrix.org)](https://matrix.to/#/#quartiq:matrix.org)
[![Continuous Integration](https://github.com/quartiq/stabilizer/actions/workflows/ci.yml/badge.svg)](https://github.com/quartiq/stabilizer/actions/workflows/ci.yml)
[![Stabilizer HITL [Nightly]](https://github.com/quartiq/hitl/actions/workflows/stabilizer-nightly.yml/badge.svg)](https://github.com/quartiq/hitl/actions/workflows/stabilizer-nightly.yml)

# Stabilizer Firmware

## Applications

Check out the [Documentation](https://quartiq.de/stabilizer) for more information on usage,
configuration, and development. 

For OITG-specific information, see below

## Hardware

[![Stabilizer](https://github.com/sinara-hw/Stabilizer/wiki/Stabilizer_v1.0_top_small.jpg)](https://github.com/sinara-hw/Stabilizer)

[![Pounder](https://user-images.githubusercontent.com/1338946/125936814-3664aa2d-a530-4c85-9393-999a7173424e.png)](https://github.com/sinara-hw/Pounder/wiki)

## Notable changes over upstream
* Added `fnc.rs` binary for fibre noise cancellation based on `dual-iir.rs`, along with a corresponding script to push messages specific to an fnc application. Relies on Pounder
* Changed default MQTT broker to `10.255.6.4`. It does not need to be flashed separately unless using a different broker.

### Active branches
Many stabilizer applications in the group rely on branches not up to date with `main`. It might prove useful to keep a track of them here for maintenance. 
