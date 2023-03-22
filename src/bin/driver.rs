//! # Driver
//!
//! Firmware for "Driver", an intelligent, high performance laser current driver mezzanine for Stabilizer.
//! [Hardware repository](https://github.com/sinara-hw/Driver)
//! Driver can be used in combination with the [Headboard](https://github.com/sinara-hw/Laser_Module/tree/master/Laser_Module),
//! a PCB that is physically close to a laser module and features various laser protection methods like relays as
//! well as an ADC for sampling photodiode signals.
//!
//! # Features
//! * A high power output whose current can be configured to constant values via [driver::HighPowerSettings].
//! * A low noise output whose current can be modulated using feedback from a Stabilizer input channel.
//!   The parameters can be configured via [driver::LowNoiseSettings]. The sampling frequency is set to 97.66 kHz.
//! * Output current and voltage [Monitor]ing.
//! * A state machine that performs various tests on the outputs and safely connects them to the laser diodes
//!   in a safe way using the relays on the Headboard. The state transitions happen automatically when the output is enabled.
//! * A laser interlock that can trip for a number of [Reason]s and reset via [Settings]. The interlock simply shorts
//!   the laser diodes on the Headboard using a relay. Without the Headboard, the interlock has no effect!
//! * An [Alarm] functionality.
//!
//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ops::Range;
use core::sync::atomic::{fence, Ordering};
use fugit::{Duration, ExtU64, TimerInstantU64};
use mutex_trait::prelude::*;

use idsp::iir;
use serde::{Deserialize, Serialize};
use stabilizer::hardware::driver::{LaserInterlock, Reason};
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        design_parameters, driver,
        driver::{output, Channel},
        hal,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, I2c1Proxy, SystemTimer, Systick, AFE0,
        AFE1, MONOTONIC_FREQUENCY,
    },
    net::{
        alarm,
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

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Miniconf)]
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
    #[miniconf(defer)]
    afe: [Gain; 2],

    /// Specifies the telemetry output period in seconds.
    ///
    /// # Path
    /// `telemetry_period`
    ///
    /// # Value
    /// Any positive, non-zero f32. Will be rounded to milliseconds.
    telemetry_period: f32,

    /// Specifies the target for data livestreaming.
    ///
    /// # Path
    /// `stream_target`
    ///
    /// # Value
    /// See [StreamTarget#miniconf]
    stream_target: StreamTarget,

    /// Alarm settings.
    /// Alarm for safe co-operation with other device like
    /// [Thermostat-EEM](https://github.com/quartiq/thermostat-eem).
    /// See [Alarm] for more details.
    ///
    /// # Value
    /// [Alarm]
    #[miniconf(defer)]
    alarm: alarm::Settings,

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
    /// See [driver::LowNoiseSettings]
    #[miniconf(defer)]
    pub low_noise: driver::LowNoiseSettings,

    /// High power channel output settings
    ///
    /// # Value
    /// See [driver::HighPowerSettings]
    #[miniconf(defer)]
    pub high_power: driver::HighPowerSettings,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // The default telemetry period in seconds.
            telemetry_period: 1.0,
            stream_target: StreamTarget::default(),
            alarm: alarm::Settings::default(),
            reset_laser_interlock: false,
            low_noise: driver::LowNoiseSettings::default(),
            high_power: driver::HighPowerSettings::default(),
        }
    }
}

/// Monitoring telemetry.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct Monitor {
    /// The measured output current on the low noise (<0>) and high power (<1>) channels. (A)
    /// Driver samples the voltage across its output current sense resistor to measure
    /// the actual output current.
    /// The current will be negative if the channel is configured as a current sink.
    current: [f32; 2],
    /// The measured output voltage on the low noise (<0>) and high power (<1>) channels. (V)
    /// The voltage will be negative if the channel is configured as a current sink.
    voltage: [f32; 2],
    /// The measured CPU temperature. (°C)
    cpu_temp: f32,
    /// The temperature on the Driver PCB. (°C)
    driver_temp: f32,
    /// The measured temperature of the Headboard temperature sensor. (°C)
    header_temp: f32,
}

/// Telemetry of low noise channel signals.
#[derive(Serialize, Copy, Clone, Default, Debug)]
pub struct LowNoise {
    /// Stabilizer ADC0 feedback signal. Can be used to modulate the low noise current.
    adc0: f32,
    /// The output current of the low noise channel. Controled by the [LowNoise] IIR filter.
    /// Note that this is only the set value, not the actually measured monitor current.
    current: f32,
}

#[derive(Serialize, Clone, Default, Debug)]
pub struct Telemetry {
    /// Refer to [Monitor]
    monitor: Monitor,
    /// Refer to [LowNoise]
    low_noise: LowNoise,
    // no high_power since it is fully defined by settings
    /// Data from the Headboard ADC.
    headboard_adc_data: [u16; 8], // Todo this will be moved into photodiode currents/pressure sensor data
    /// The state of the laser interlock. `None` if the interlock is not tripped or
    /// `Some(`[Reason]`)` if it is tripped.
    interlock_tripped: Option<Reason>,
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use stabilizer::{hardware::driver::Condition, net::alarm::Change};

    use super::*;

    /// Period for checking the Driver output interlock voltage/current levels.
    pub const DRIVER_MONITOR_PERIOD: Duration<u64, 1, 10000> =
        Duration::<u64, 1, 10000>::millis(5); // 5 ms

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        network: NetworkUsers<Settings, Telemetry>,
        settings: Settings,
        telemetry: Telemetry,
        header_adc: driver::ltc2320::Ltc2320,
        header_adc_data: [u16; 8],
        output_state: [output::sm::StateMachine<output::Output<I2c1Proxy>>; 2],
        ln_output_enabled: bool, // variable to relay output_en info to the process task without having to lock output_state
        ramp_iir: iir::IIR<f32>, // variable to relay the ramp iir to the process task without having to lock output_state
        alarm_handle: Option<trip_alarm::SpawnHandle>,
        laser_interlock: LaserInterlock,
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
        channel_range: [Range<f32>; 2],
        driver_temp: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
        header_temp: lm75::Lm75<I2c1Proxy, lm75::ic::Lm75>,
        internal_adc: driver::internal_adc::InternalAdc,
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
            header_adc: driver.ltc2320,
            header_adc_data: [0u16; 8],
            output_state: driver.output_sm,
            ln_output_enabled: false,
            ramp_iir: iir::IIR::default(),
            alarm_handle: None,
            laser_interlock: driver.laser_interlock,
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
            channel_range: [driver.dac[0].range(), driver.dac[1].range()],
            driver_dac: driver.dac,
            driver_temp: driver.driver_lm75,
            header_temp: driver.header_lm75,
            internal_adc: driver.internal_adc,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Check that output delays are sufficiently long for internal adc measurements to be updated.
        assert!(
            driver::output::Output::<I2c1Proxy>::SET_DELAY
                >= 2 * DRIVER_MONITOR_PERIOD
        );
        assert!(
            driver::relay::Relay::<I2c1Proxy>::K0_DELAY
                >= 2 * DRIVER_MONITOR_PERIOD
        );
        assert!(
            driver::relay::Relay::<I2c1Proxy>::K1_DELAY
                >= 2 * DRIVER_MONITOR_PERIOD
        );

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        header_adc_start_conversion::spawn().unwrap();
        start::spawn_after(100.millis()).unwrap();
        monitor_output::spawn_after(DRIVER_MONITOR_PERIOD).unwrap();

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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, telemetry, ln_output_enabled, ramp_iir, header_adc_data], priority=4)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            header_adc_data,
            ln_output_enabled,
            ramp_iir,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
        } = c.local;

        (
            settings,
            telemetry,
            header_adc_data,
            ln_output_enabled,
            ramp_iir,
        )
            .lock(
                |settings,
                 telemetry,
                 headboard_adc_data,
                 ln_output_enabled,
                 ramp_iir| {
                    let digital_inputs = [
                        digital_inputs.0.is_high(),
                        digital_inputs.1.is_high(),
                    ];

                    let hold = settings.low_noise.force_hold
                        || (digital_inputs[1] && settings.low_noise.allow_hold);

                    (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                        // Preserve instruction and data ordering w.r.t. DMA flag access.
                        fence(Ordering::SeqCst);

                        // Perform processing for  ADC channel 0 --> Driver channel 0 (LowNoise)
                        let x = f32::from(AdcCode(adc0[0]))
                            / settings.afe[driver::Channel::LowNoise as usize]
                                .as_multiplier(); // get adc sample in volt * AFE gain for equivalent input voltage
                        let iir = if *ln_output_enabled {
                            &settings.low_noise.iir
                        } else {
                            ramp_iir
                        };
                        let y = iir.update(&mut iir_state[0], x, hold);

                        // Convert to DAC code. Output 1V/1A Driver output current. Bounds must be ensured by filter limits!
                        dac0[0] = DacCode::try_from(y).unwrap().0;

                        // Set Driver current.
                        write_dac_spi::spawn(driver::Channel::LowNoise, y)
                            .unwrap();

                        // Stream the data.
                        let adc_samples = [adc0, adc1];
                        let dac_samples = [dac0, dac1];
                        const N: usize = BATCH_SIZE
                            * core::mem::size_of::<i16>()
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
                        telemetry.headboard_adc_data = *headboard_adc_data;

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
                NetworkState::SettingsChanged(_) => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::AlarmChanged(alarm) => {
                    alarm_update::spawn(Change::Alarm(alarm)).unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, shared=[alarm_handle, settings])]
    fn alarm_update(mut c: alarm_update::Context, change: Change) {
        let settings = c.shared.settings.lock(|settings| *settings);

        c.shared.alarm_handle.lock(|handle| {
            if let Some(action) =
                settings.alarm.action(change, handle.is_some())
            {
                *handle = match action {
                    alarm::Action::Spawn(millis) => {
                        log::info!("Alarm armed");
                        Some(
                            trip_alarm::spawn_after(
                                millis,
                                Reason::AlarmTimeout,
                            )
                            .unwrap(),
                        )
                    }
                    alarm::Action::Trip => {
                        log::info!("Alarm tripped");
                        let _ = handle.take().unwrap().cancel().map_err(|e| {
                            log::error!(
                                "Cannot trip. Alarm already tripped! {:?}",
                                e
                            )
                        });
                        Some(
                            trip_alarm::spawn_after(0.millis(), Reason::Alarm)
                                .unwrap(),
                        )
                    }
                    alarm::Action::Reschedule(millis) => handle
                        .take()
                        .unwrap()
                        .reschedule_after(millis)
                        .map_err(|e| {
                            log::error!(
                                "Cannot reschedule. Alarm already tripped! {:?}"
                            , e)
                        })
                        .ok(), // return `None` if rescheduled too late aka alarm already tripped
                    alarm::Action::Cancel => {
                        log::info!("Alarm disarmed");
                        let _ = handle.take().unwrap().cancel().map_err(|e| {
                            log::error!(
                                "Cannot disarm. Alarm already tripped! {:?}",
                                e
                            )
                        });
                        None
                    } // Ignore error if cancelled too late. This is ok since we don't want to cancel if the alarm already tripped.
                };
            }
        });
    }

    #[task(priority = 1, local=[afes, channel_range], shared=[network, settings, alarm_handle, output_state, laser_interlock])]
    fn settings_update(mut c: settings_update::Context) {
        let mut new_settings =
            c.shared.network.lock(|net| *net.miniconf.settings());
        let old_settings = c.shared.settings.lock(|current| *current);

        if old_settings.alarm.armed != new_settings.alarm.armed {
            alarm_update::spawn(Change::Armed).unwrap()
        }

        // Only set interlock pin high if all channels disabled and false -> true transition.
        if !old_settings.reset_laser_interlock
            && new_settings.reset_laser_interlock
        {
            if c.shared.output_state.lock(|output| {
                output[driver::Channel::HighPower as usize].is_enabled()
            }) {
                log::error!("Cannot reset laser interlock while HighPower channel is enabled. Disable channel first.")
            } else if c.shared.output_state.lock(|output| {
                output[driver::Channel::LowNoise as usize].is_enabled()
            }) {
                log::error!("Cannot reset laser interlock while LowNoise channel is enabled. Disable channel first.")
            } else {
                log::info!("Laser interlock reset.");
                c.shared.laser_interlock.lock(|ilock| ilock.set(None));
                c.shared.alarm_handle.lock(|handle| {
                    if let Some(millis) =
                        new_settings.alarm.rearm(handle.is_some())
                    {
                        log::info!("Alarm re-armed.");
                        *handle = Some(
                            trip_alarm::spawn_after(
                                millis,
                                Reason::AlarmTimeout,
                            )
                            .unwrap(),
                        )
                    }
                });
            }
        }

        // check if the output currents/limits are within the allowed range and clamp them if not
        new_settings.high_power.current =
            new_settings.high_power.current.clamp(
                c.local.channel_range[1].start,
                c.local.channel_range[1].end,
            );
        new_settings.low_noise.iir.y_min =
            new_settings.low_noise.iir.y_min.clamp(
                c.local.channel_range[0].start,
                c.local.channel_range[0].end,
            );
        new_settings.low_noise.iir.y_max =
            new_settings.low_noise.iir.y_max.clamp(
                c.local.channel_range[0].start,
                c.local.channel_range[0].end,
            );

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
    }

    #[task(priority = 1, shared=[network, settings, telemetry, laser_interlock], local=[cpu_temp_sensor, driver_temp, header_temp])]
    fn telemetry(mut c: telemetry::Context) {
        let mut telemetry: Telemetry =
            c.shared.telemetry.lock(|telemetry| telemetry.clone());

        telemetry.monitor.cpu_temp =
            c.local.cpu_temp_sensor.get_temperature().unwrap();
        telemetry.monitor.driver_temp =
            c.local.driver_temp.read_temperature().unwrap();

        #[cfg(feature = "ai_artiq_laser_module")]
        {
            telemetry.monitor.header_temp =
                c.local.header_temp.read_temperature().unwrap();
        }
        telemetry.interlock_tripped =
            c.shared.laser_interlock.lock(|ilock| ilock.reason());
        let telemetry_period = c
            .shared
            .settings
            .lock(|settings| (settings.telemetry_period));

        c.shared
            .network
            .lock(|net| net.telemetry.publish(&telemetry));
        // Schedule the telemetry task in the future.
        telemetry::spawn_after(((telemetry_period * 1000.0) as u64).millis())
            .unwrap();
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::spawn_after(1.secs()).unwrap();
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
        header_adc_start_conversion::spawn_at(
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
    #[task(priority = 1, capacity = 2, shared=[output_state, ln_output_enabled, ramp_iir, settings, laser_interlock, telemetry])]
    fn handle_output_tick(
        mut c: handle_output_tick::Context,
        channel: driver::Channel,
    ) {
        let ramp_target = c.shared.settings.lock(|settings| match channel {
            // clamp the ramp target to the iir limits to prevent ramping to an invalid current
            driver::Channel::LowNoise => settings.low_noise.iir.y_offset.clamp(
                settings.low_noise.iir.y_min,
                settings.low_noise.iir.y_max,
            ),
            driver::Channel::HighPower => settings.high_power.current,
        });
        let reads = c.shared.telemetry.lock(|tele| {
            [
                tele.monitor.voltage[channel as usize],
                tele.monitor.current[channel as usize],
            ]
        });
        let (oe, iir) = (c.shared.output_state, c.shared.laser_interlock).lock(
            |state, ilock| {
                let (delay, result) = state[channel as usize].handle_tick(
                    &ramp_target,
                    &channel,
                    &reads,
                );
                if let Some(delay) = delay {
                    handle_output_tick::spawn_after(delay.convert(), channel)
                        .unwrap();
                };
                if let Some(result) = result {
                    ilock.set(Some(Reason::Selftest(result)));
                }

                if channel == driver::Channel::HighPower {
                    write_dac_spi::spawn(
                        channel,
                        state[channel as usize].iir().y_offset,
                    )
                    .unwrap();
                }
                (
                    state[driver::Channel::LowNoise as usize].is_enabled(),
                    *state[driver::Channel::LowNoise as usize].iir(),
                )
            },
        );
        // Store the output enabled and current ramp iir in separate shared variables. Theses will be used by the
        // process task to perform the current ramp if the output is beeing ramped up.
        // We cannot directly use the output state since it contains the output relays and is therefore locked
        // for a long time during output tick handling, when the relays are toggled.
        // It is crutial that the information in output_enabled and the ramp_iir is not out of sync with the actual
        // output_state. This is ensured since the state machine always goes through transitions that make use of
        // the handle_output_tick() task during enabling and disabling.
        (c.shared.ln_output_enabled, c.shared.ramp_iir).lock(
            |ln_output_enabled, ramp_iir| {
                *ln_output_enabled = oe;
                *ramp_iir = iir;
            },
        )
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

    #[task(priority = 1, shared=[alarm_handle, laser_interlock])]
    fn trip_alarm(mut c: trip_alarm::Context, reason: Reason) {
        c.shared.alarm_handle.lock(|handle| *handle = None);
        log::error!("Alarm! {:?} Laser Interlock tripped.", reason);
        c.shared
            .laser_interlock
            .lock(|ilock| ilock.set(Some(reason)));
    }

    #[task(priority = 1, shared=[telemetry, settings, laser_interlock, output_state], local=[internal_adc])]
    fn monitor_output(mut c: monitor_output::Context) {
        let (interlock_current, interlock_voltage) =
            c.shared.settings.lock(|settings| {
                (
                    [
                        settings.low_noise.interlock_current,
                        settings.high_power.interlock_current,
                    ],
                    [
                        settings.low_noise.interlock_voltage,
                        settings.high_power.interlock_voltage,
                    ],
                )
            });
        let output_enabled = c
            .shared
            .output_state
            .lock(|output| [output[0].is_enabled(), output[1].is_enabled()]);
        let adc = c.local.internal_adc;
        let (current_reads, voltage_reads) = (
            [
                adc.read_output_current(Channel::LowNoise),
                adc.read_output_current(Channel::HighPower),
            ],
            [
                adc.read_output_voltage(Channel::LowNoise),
                adc.read_output_voltage(Channel::HighPower),
            ],
        );
        let interlock_asserted = c
            .shared
            .laser_interlock
            .lock(|ilock| ilock.reason().is_none());
        for (i, ((interlock_current, read), output_en)) in interlock_current
            .iter()
            .zip(current_reads.iter())
            .zip(output_enabled.iter())
            .enumerate()
        {
            if (read > interlock_current) & interlock_asserted & output_en {
                let ch = Channel::try_from(i).unwrap();
                c.shared.laser_interlock.lock(|ilock| {
                    ilock.set(Some(Reason::Overcurrent(Condition {
                        channel: ch,
                        threshold: *interlock_current,
                        read: *read,
                    })))
                });
            }
        }
        for (i, ((interlock_voltage, read), output_en)) in interlock_voltage
            .iter()
            .zip((voltage_reads).iter())
            .zip((output_enabled).iter())
            .enumerate()
        {
            if (read > interlock_voltage) & interlock_asserted & output_en {
                let ch = Channel::try_from(i).unwrap();
                c.shared.laser_interlock.lock(|ilock| {
                    ilock.set(Some(Reason::Overvoltage(Condition {
                        channel: ch,
                        threshold: *interlock_voltage,
                        read: *read,
                    })))
                });
            }
        }
        c.shared.telemetry.lock(|tele| {
            tele.monitor.current = current_reads;
            tele.monitor.voltage = voltage_reads;
        });
        monitor_output::spawn_after(DRIVER_MONITOR_PERIOD).unwrap();
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
