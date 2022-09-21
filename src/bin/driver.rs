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
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

const SCALE: f32 = i16::MAX as _;

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

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    iir_ch: [iir::IIR<f32>; 2],

    /// Specified true if DI1 should be used as a "hold" input.
    ///
    /// # Path
    /// `allow_hold`
    ///
    /// # Value
    /// "true" or "false"
    allow_hold: bool,

    /// Specified true if "hold" should be forced regardless of DI1 state and hold allowance.
    ///
    /// # Path
    /// `force_hold`
    ///
    /// # Value
    /// "true" or "false"
    force_hold: bool,

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

    /// Interlock settings.
    ///
    /// # Path
    /// `interlock`
    ///
    /// # Value
    /// [Interlock]
    interlock: Interlock,

    /// Output enabled. `True` to enable, `False` to disable.
    ///
    /// # Path
    /// `output_enabled`
    ///
    /// # Value
    /// [bool]
    output_enabled: [bool; 2],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [iir::IIR::new(0., -SCALE, SCALE); 2],
            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 1,

            stream_target: StreamTarget::default(),

            interlock: Interlock::default(),

            output_enabled: [false; 2],
        }
    }
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
        telemetry: TelemetryBuffer,
        internal_adc: driver::internal_adc::InternalAdc,
        header_adc: driver::ltc2320::Ltc2320,
        header_adc_data: [u16; 8],
        // driver_dac: [Dac; 2]
        output_state: [output::sm::StateMachine<output::Output<I2c1Proxy>>; 2],
        interlock_handle: Option<trip_interlock::SpawnHandle>,
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
            telemetry: TelemetryBuffer::default(),
            internal_adc: driver.internal_adc,
            header_adc: driver.ltc2320,
            header_adc_data: [0u16; 8],
            output_state: driver.output_sm,
            interlock_handle: None,
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

        // mock LN output enable
        // let del = shared.relay_state[0].enable().unwrap();
        // handle_relay::spawn_after(del.convert(), Channel::LowNoise).unwrap();

        (shared, local, init::Monotonics(stabilizer.systick))
    }

    #[task(priority = 1, local=[sampling_timer])]
    fn start(c: start::Context) {
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// # Note
    /// Processing time for the DSP application code is bounded by the following constraints:
    ///
    /// DSP application code starts after the ADC has generated a batch of samples and must be
    /// completed by the time the next batch of ADC samples has been acquired (plus the FIFO buffer
    /// time). If this constraint is not met, firmware will panic due to an ADC input overrun.
    ///
    /// The DSP application code must also fill out the next DAC output buffer in time such that the
    /// DAC can switch to it when it has completed the current buffer. If this constraint is not met
    /// it's possible that old DAC codes will be generated on the output and the output samples will
    /// be delayed by 1 batch.
    ///
    /// Because the ADC and DAC operate at the same rate, these two constraints actually implement
    /// the same time bounds, meeting one also means the other is also met.
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, telemetry, output_state], priority=4)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            output_state,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
        } = c.local;

        (settings, telemetry, output_state).lock(
            |settings, telemetry, output| {
                let digital_inputs =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                telemetry.digital_inputs = digital_inputs;

                let hold = settings.force_hold
                    || (digital_inputs[1] && settings.allow_hold);

                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    let adc_samples = [adc0, adc1];
                    let dac_samples = [dac0, dac1];

                    // Preserve instruction and data ordering w.r.t. DMA flag access.
                    fence(Ordering::SeqCst);

                    // Perform processing for  ADC channel 0 --> Driver channel 0 (LowNoise)
                    adc_samples[0]
                        .iter()
                        .zip(dac_samples[0].iter_mut())
                        .map(|(ai, di)| {
                            let x = f32::from(*ai); // get adc sample in volt
                            let iir = if output[0].is_enabled() {
                                settings.iir_ch[0]
                            } else {
                                *output[0].iir()
                            };
                            let y = iir.update(&mut iir_state[0], x, hold);

                            // Convert to DAC code. Output 1V/1A Driver output current. Bounds must be ensured by filter limits!
                            *di = DacCode::try_from(y).unwrap().0;

                            // Set Driver current.
                            write_dac_spi::spawn(driver::Channel::LowNoise, y)
                                .unwrap();
                        })
                        .last();

                    // Stream the data.
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
                    telemetry.adcs = [
                        AdcCode(adc_samples[0][0]),
                        AdcCode(adc_samples[1][0]),
                    ];

                    telemetry.dacs = [
                        DacCode(dac_samples[0][0]),
                        DacCode(dac_samples[1][0]),
                    ];

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

    #[task(priority = 1, local=[afes], shared=[network, settings, interlock_handle, output_state])]
    fn settings_update(
        mut c: settings_update::Context,
        path: Option<String<64>>,
    ) {
        let new_settings =
            c.shared.network.lock(|net| *net.miniconf.settings());
        let old_settings = c.shared.settings.lock(|current| *current);

        c.shared.interlock_handle.lock(|handle| {
            if let Some(action) = new_settings.interlock.action(
                path.as_ref()
                    .map(|path| path.as_str().trim_start_matches("interlock/")),
                handle.is_some(),
                &old_settings.interlock,
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
                    } // ignore error if cancelled too late
                };
            }
        });

        c.shared.settings.lock(|current| *current = new_settings);
        c.local.afes.0.set_gain(new_settings.afe[0]);
        c.local.afes.1.set_gain(new_settings.afe[1]);

        let target = new_settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));

        c.shared.output_state.lock(|output| {
            for (i, (&new_output_enabled, old_output_enabled)) in new_settings
                .output_enabled
                .iter()
                .zip(old_settings.output_enabled)
                .enumerate()
            {
                if new_output_enabled != old_output_enabled {
                    if let Ok(Some(delay)) =
                        output[i].set_enable(new_output_enabled).map_err(|e| {
                            log::error!(
                                "Cannot enable/disable output {:?}! {:?}",
                                i,
                                e
                            )
                        })
                    {
                        handle_output_tick::spawn_after(
                            delay.convert(),
                            i.try_into().unwrap(),
                        )
                        .unwrap();
                    }
                }
            }
            if output[driver::Channel::HighPower as usize].is_enabled() {
                write_dac_spi::spawn(
                    driver::Channel::HighPower,
                    new_settings.iir_ch[1].y_offset,
                )
                .unwrap();
            }
        });
    }

    #[task(priority = 1, shared=[network, settings, telemetry, internal_adc], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = c
            .shared
            .settings
            .lock(|settings| (settings.afe, settings.telemetry_period));

        c.shared.network.lock(|net| {
            net.telemetry.publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
            ))
        });
        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
        log::info!(
            "internal adc values: {:?}",
            c.shared.internal_adc.lock(|adc| [
                adc.read_output_current(Channel::LowNoise),
                adc.read_output_current(Channel::HighPower),
                adc.read_output_voltage(Channel::LowNoise),
                adc.read_output_voltage(Channel::HighPower),
            ])
        );
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
        let iir = c
            .shared
            .settings
            .lock(|settings| settings.iir_ch[channel as usize]);
        c.shared.output_state.lock(|state| {
            state[channel as usize].handle_tick(&iir).map(|del| {
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
