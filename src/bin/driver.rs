//! # Driver
//!
//! Firmware for "Driver", an intelligent, high performance laser current driver.
//! Driver is a mezzanine module to Stabilizer, on which the firmware is deployed.
//!
//! This software is currently just the groundwork for the future developments.

#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{fence, Ordering};

use fugit::{ExtU64, TimerInstantU64};
use heapless::String;
use mutex_trait::prelude::*;

use idsp::iir;

use serde::Serialize;
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        design_parameters, driver,
        driver::{
            interlock::{Action, Interlock},
            output, Channel,
        },
        hal,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, I2c1Proxy, SystemTimer, Systick, AFE0,
        AFE1, MONOTONIC_FREQUENCY,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Miniconf,
        NetworkState, NetworkUsers,
    },
};

// The number of samples in each batch process
// DO NOT CHANGE FOR DRIVER
const BATCH_SIZE: usize = 1;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^10 =
// 1024, there are 10.24uS per sample, corresponding to a sampling frequency of 97.66 KHz.
const SAMPLE_TICKS_LOG2: u8 = 10;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    afe: [Gain; 2],

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any non-zero value less than 65536.
    telemetry_period: u16,

    /// Specifies the target for data livestreaming.
    ///
    /// # Path
    /// `stream_target`
    ///
    /// # Value
    /// See [StreamTarget#miniconf]
    stream_target: StreamTarget,

    /// Thermostat interlock settings.
    ///
    ///
    /// # Value
    /// [Interlock]
    thermostat_interlock: Interlock,

    /// Laser interlock reset.
    /// A false -> true transition will reset a tripped laser interlock
    /// if both Driver outputs are disabled at that time.
    /// Note that the laser interlock is in a tripped state after Driver startup.
    ///
    /// # Value
    /// true or false
    reset_laser_interlock: bool,

    /// Low noise channel output settings
    ///
    /// # Value
    /// See [driver::output::LowNoiseSettings]
    pub low_noise: driver::output::LowNoiseSettings,

    /// High power channel output settings
    ///
    /// # Value
    /// See [driver::output::HighPowerSettings]
    pub high_power: driver::output::HighPowerSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // The default telemetry period in seconds.
            telemetry_period: 1,
            stream_target: StreamTarget::default(),
            thermostat_interlock: Interlock::default(),
            reset_laser_interlock: false,
            low_noise: driver::output::LowNoiseSettings::default(),
            high_power: driver::output::HighPowerSettings::default(),
        }
    }
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Monitor {
    current: [f32; 2],
    voltage: [f32; 2],
    cpu_temp: f32,
    header_temp: f32,
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct LowNoise {
    adc0: f32,    // Stabilizer ADC0 feedback signal
    current: f32, // Output current set by control loop
}

#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Telemetry {
    monitor: Monitor,
    low_noise: LowNoise,
    // no high_power since it is fully defined by settings
    header_adc_data: [u16; 8], // Todo this will be moved into photodiode currents/pressure sensor data
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        telemetry: Telemetry,
        internal_adc: driver::internal_adc::InternalAdc,
        header_adc: driver::ltc2320::Ltc2320,
        header_adc_data: [u16; 8],
        output_state: [output::sm::StateMachine<output::Output<I2c1Proxy>>; 2],
        interlock_handle: Option<trip_interlock::SpawnHandle>,
        laser_interlock_pin: hal::gpio::Pin<'B', 13, hal::gpio::Output>,
    }

    #[local]
    struct Local {
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        iir_state: [iir::Vec5<f32>; 2],
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        header_adc_conversion_scheduled: TimerInstantU64<MONOTONIC_FREQUENCY>, // auxillary local variable for exact scheduling
        driver_dac: [driver::dac::Dac<driver::Spi1Proxy>; 2],
        lm75: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        // Configure the microcontroller
        let (mut stabilizer, mezzanine) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
            option_env!("BROKER")
                .unwrap_or("10.34.16.10")
                .parse()
                .unwrap(),
        );

        let generator = network
            .configure_streaming(StreamFormat::AdcDacData, BATCH_SIZE as _);

        let settings = Settings::default();

        let driver = mezzanine.get_driver();

        let shared = Shared {
            network,
            settings,
            telemetry: Telemetry::default(),
            internal_adc: driver.internal_adc,
            header_adc: driver.ltc2320,
            header_adc_data: [0u16; 8],
            output_state: driver.output_sm,
            interlock_handle: None,
            laser_interlock_pin: driver.laser_interlock_pin,
        };

        let mut local = Local {
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[0.; 5]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            header_adc_conversion_scheduled: stabilizer.systick.now(),
            driver_dac: driver.dac,
            lm75: driver.lm75,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Spawn a settings update for default settings.
        settings_update::spawn(None).unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        header_adc_start_conversion::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    #[task(priority = 1, local=[sampling_timer])]
    fn start(c: start::Context) {
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// Performs an IIR processing step on a sample from Stabilizer ADC0 and generates a new output
    /// sample for the Driver LowNoise channel. The output is also available at Stabilizer DAC0 at 1V/1A.
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, telemetry, output_state, header_adc_data], priority=4)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            output_state,
            header_adc_data,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
        } = c.local;

        (settings, telemetry, output_state, header_adc_data).lock(
            |settings, telemetry, output, header_adc_data| {
                let digital_inputs =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];

                let hold = settings.low_noise.force_hold
                    || (digital_inputs[1] && settings.low_noise.allow_hold);

                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    // Perform processing for  ADC channel 0 --> Driver channel 0 (LowNoise)
                    let x = f32::from(AdcCode(adc0[0]))
                        / settings.afe[driver::Channel::LowNoise as usize]
                            .as_multiplier(); // get adc sample in volt * AFE gain for equivalent input voltage
                    let iir = if output[driver::Channel::LowNoise as usize]
                        .is_enabled()
                    {
                        settings.low_noise.iir
                    } else {
                        *output[driver::Channel::LowNoise as usize].iir()
                    };
                    let y = iir.update(&mut iir_state[0], x, hold);

                    // Convert to DAC code. Output 1V/1A Driver output current. Bounds must be ensured by filter limits!
                    dac0[0] = DacCode::try_from(y).unwrap().0;

                    // Set Driver current.
                    write_dac_spi::spawn(driver::Channel::LowNoise, y).unwrap();

                    // Driver TODO: Rework Telemetry and Streaming.

                    // Stream the data.
                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];
                    const N: usize = BATCH_SIZE * core::mem::size_of::<i16>()
                        / core::mem::size_of::<MaybeUninit<u8>>();
                    generator.add::<_, { N * 4 }>(|buf| {
                        for (data, buf) in adc_samples
                            .iter()
                            .chain(dac_samples.iter())
                            .zip(buf.chunks_exact_mut(N))
                        {
                            let data = unsafe {
                                core::slice::from_raw_parts(
                                    data.as_ptr() as *const MaybeUninit<u8>,
                                    N,
                                )
                            };
                            buf.copy_from_slice(data)
                        }
                    });
                    // Update telemetry measurements.
                    telemetry.low_noise.adc0 = x;
                    telemetry.low_noise.current = y;
                    // Todo: The raw photodiode samples should be converted to eqivalent photocurrent(?)
                    // and incorporated into the signal processing.
                    telemetry.header_adc_data = *header_adc_data;

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);
                });
            },
        );
    }

    #[idle(shared=[network])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged(path) => {
                    settings_update::spawn(Some(path)).unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, local=[afes], shared=[network, settings, interlock_handle, output_state, laser_interlock_pin])]
    fn settings_update(
        mut c: settings_update::Context,
        path: Option<String<64>>,
    ) {
        let new_settings =
            c.shared.network.lock(|net| *net.miniconf.settings());
        let old_settings = c.shared.settings.lock(|current| *current);

        c.shared.interlock_handle.lock(|handle| {
            if let Some(action) = new_settings.thermostat_interlock.action(
                path.as_ref()
                    .map(|path| path.as_str().trim_start_matches("interlock/")),
                handle.is_some(),
                &old_settings.thermostat_interlock,
            ) {
                *handle = match action {
                   Action::Spawn(millis) => {
                        log::info!("Interlock armed");
                        Some(trip_interlock::spawn_after(millis).unwrap())
                    }
                   Action::Reschedule(millis) => {
                        handle
                            .take()
                            .unwrap()
                            .reschedule_after(millis)
                        .map_err(|e|
                            log::error!(
                                "Cannot reschedule. Interlock already tripped! {:?}"
                            , e))
                        .ok()
                    } // return `None` if rescheduled too late aka interlock already tripped
                   Action::Cancel => {
                        let _ = handle.take().unwrap().cancel().map_err(|e|
                            log::error!(
                                "Cannot cancel. Interlock already tripped! {:?}"
                            , e));
                        None
                    } // Ignore error if cancelled too late. This is ok since we don't want to cancel if the interlock already tripped. 
                };
            }
        });

        c.shared.settings.lock(|current| *current = new_settings);
        c.local.afes.0.set_gain(new_settings.afe[0]);
        c.local.afes.1.set_gain(new_settings.afe[1]);

        let target = new_settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));

        for (i, (&new_output_enabled, old_output_enabled)) in [
            new_settings.low_noise.output_enabled,
            new_settings.high_power.output_enabled,
        ]
        .iter()
        .zip([
            old_settings.low_noise.output_enabled,
            old_settings.high_power.output_enabled,
        ])
        .enumerate()
        {
            if new_output_enabled != old_output_enabled {
                if let Ok(Some(delay)) = c.shared.output_state.lock(|output| {
                    output[i].set_enable(new_output_enabled).map_err(|e| {
                        log::error!(
                            "Cannot enable/disable output {:?}! {:?}",
                            i,
                            e
                        )
                    })
                }) {
                    handle_output_tick::spawn_after(
                        delay.convert(),
                        i.try_into().unwrap(),
                    )
                    .unwrap();
                }
            }
        }
        if c.shared.output_state.lock(|output| {
            output[driver::Channel::HighPower as usize].is_enabled()
        }) {
            write_dac_spi::spawn(
                driver::Channel::HighPower,
                new_settings.high_power.current,
            )
            .unwrap();
        };
        // Only set interlock pin high if all channels disabled and false -> true transition.
        if !old_settings.reset_laser_interlock
            && new_settings.reset_laser_interlock
            && c.shared.output_state.lock(|output| {
                !output[driver::Channel::HighPower as usize].is_enabled()
                    && !output[driver::Channel::LowNoise as usize].is_enabled()
            })
        {
            c.shared.laser_interlock_pin.lock(|pin| pin.set_high());
        }
    }

    #[task(priority = 1, shared=[network, settings, telemetry, internal_adc], local=[cpu_temp_sensor, lm75])]
    fn telemetry(mut c: telemetry::Context) {
        let mut telemetry: Telemetry =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        telemetry.monitor.cpu_temp =
            c.local.cpu_temp_sensor.get_temperature().unwrap();
        // Todo: uncomment once we have Hardware
        // telemetry.monitor.header_temp =
        //     c.local.lm75.read_temperature().unwrap();
        (telemetry.monitor.current, telemetry.monitor.voltage) =
            c.shared.internal_adc.lock(|adc| {
                (
                    [
                        adc.read_output_current(Channel::LowNoise),
                        adc.read_output_current(Channel::HighPower),
                    ],
                    [
                        adc.read_output_voltage(Channel::LowNoise),
                        adc.read_output_voltage(Channel::HighPower),
                    ],
                )
            });
        let telemetry_period = c
            .shared
            .settings
            .lock(|settings| (settings.telemetry_period));

        c.shared
            .network
            .lock(|net| net.telemetry.publish(&telemetry));
        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
    }

    #[task(priority = 3, shared=[header_adc], local=[header_adc_conversion_scheduled])]
    fn header_adc_start_conversion(
        mut c: header_adc_start_conversion::Context,
    ) {
        c.shared
            .header_adc
            .lock(|ltc| ltc.start_conversion())
            .unwrap(); // panic if header_adc timing is not met
        *c.local.header_adc_conversion_scheduled +=
            design_parameters::HEADER_ADC_PERIOD.convert(); // update time at which the next conversion is scheduled
        header_adc_start_conversion::Monotonic::spawn_at(
            *c.local.header_adc_conversion_scheduled,
        )
        .unwrap();
    }

    #[task(binds = QUADSPI, priority = 3, shared=[header_adc, header_adc_data])]
    fn header_adc_transfer_done(c: header_adc_transfer_done::Context) {
        (c.shared.header_adc, c.shared.header_adc_data).lock(|ltc, data| {
            ltc.handle_transfer_done(data);
        });
    }

    // Task for handling the Powerup/-down sequence
    #[task(priority = 3, capacity = 2, shared=[output_state, settings])]
    fn handle_output_tick(
        mut c: handle_output_tick::Context,
        channel: driver::Channel,
    ) {
        let ramp_target = c.shared.settings.lock(|settings| match channel {
            driver::Channel::LowNoise => settings.low_noise.iir.y_offset,
            driver::Channel::HighPower => settings.high_power.current,
        });

        c.shared.output_state.lock(|state| {
            state[channel as usize]
                .handle_tick(&ramp_target)
                .map(|del| {
                    handle_output_tick::Monotonic::spawn_after(
                        del.convert(),
                        channel,
                    )
                    .unwrap()
                });
            if channel == driver::Channel::HighPower {
                write_dac_spi::spawn(
                    channel,
                    state[channel as usize].iir().y_offset,
                )
                .unwrap();
            }
        });
    }

    // Driver DAC SPI write task. This effectively makes the slow, blocking SPI writes non-blocking
    // for higher priority tasks.
    #[task(priority = 2, capacity = 2, local=[driver_dac])]
    fn write_dac_spi(
        c: write_dac_spi::Context,
        channel: driver::Channel,
        current: f32,
    ) {
        c.local.driver_dac[channel as usize].set(current).unwrap()
    }

    #[task(priority = 1, shared=[interlock_handle])]
    fn trip_interlock(mut c: trip_interlock::Context) {
        c.shared.interlock_handle.lock(|handle| *handle = None);
        log::error!("interlock tripped");
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 5)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 5)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }

    #[task(binds = SPI4, priority = 5)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 5)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }
}
