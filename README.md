![download](https://user-images.githubusercontent.com/17088194/202477035-237e7824-ab1d-4d32-a172-004ab3f2cd8f.png)

# Driver Firmware
Embedded software for [Stabilizer](https://github.com/sinara-hw/Stabilizer) + [Driver](https://github.com/sinara-hw/Driver) + [Headboard](https://github.com/sinara-hw/Laser_Module/tree/master/Laser_Module), which together form a system for driving an integrated MOPA laser module.

- using [STM32H7 HAL](https://github.com/stm32-rs/stm32h7xx-hal) 
- [RTIC](https://github.com/rtic-rs/cortex-m-rtic) based task scheduling
- [MQTT](https://mqtt.org/) networking using the [smoltcp](https://github.com/smoltcp-rs/smoltcp) tcp/ip stack, [minimq](https://github.com/quartiq/minimq) for embedded MQTT and [miniconf](https://github.com/quartiq/miniconf) for settings
- signal processing and control based on biquad IIR filters from [idsp](https://github.com/quartiq/idsp)
