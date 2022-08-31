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
            relay::{self, sm::StateMachine},
            Channel,
        },
        hal,
        signal_generator::{self, SignalGenerator},
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

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;
const SAMPLE_PERIOD: f32 =
    SAMPLE_TICKS as f32 * hardware::design_parameters::TIMER_PERIOD;

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
    /// `iir_ch/<n>/<m>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    /// * <m> specifies which cascade to configure. <m> := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// # Value
    /// See [iir::IIR#miniconf]
    iir_ch: [[iir::IIR<f32>; IIR_CASCADE_LENGTH]; 2],

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

    /// Specifies the config for signal generators to add on to DAC0/DAC1 outputs.
    ///
    /// # Path
    /// `signal_generator/<n>`
    ///
    /// * <n> specifies which channel to configure. <n> := [0, 1]
    ///
    /// # Value
    /// See [signal_generator::BasicConfig#miniconf]
    signal_generator: [signal_generator::BasicConfig; 2],

    /// Interlock settings.
    ///
    /// # Path
    /// `interlock`
    ///
    /// # Value
    /// [interlock::Interlock]
    interlock: Interlock,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 1,

            signal_generator: [signal_generator::BasicConfig::default(); 2],

            stream_target: StreamTarget::default(),

            interlock: Interlock::default(),
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
        signal_generator: [SignalGenerator; 2],
        adc_internal: driver::adc_internal::AdcInternal,
        header_adc: driver::ltc2320::Ltc2320,
        header_adc_data: [u16; 8],
        // driver_relay_state might eventually live inside driver_output_state but then the
        // relay_delay function would have to lock the whole output state..
        driver_relay_state: [StateMachine<relay::Relay<I2c1Proxy>>; 2],
        // driver_dac: [Dac; 2]
        // driver_output_state: [Output State Macheine; 2]
        interlock_handle: Option<trip_interlock::SpawnHandle>,
    }

    #[local]
    struct Local {
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        iir_state: [[iir::Vec5<f32>; IIR_CASCADE_LENGTH]; 2],
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        header_adc_conversion_scheduled: TimerInstantU64<MONOTONIC_FREQUENCY>, // auxillary local variable for exact scheduling
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
            signal_generator: [
                SignalGenerator::new(
                    settings.signal_generator[0]
                        .try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE)
                        .unwrap(),
                ),
                SignalGenerator::new(
                    settings.signal_generator[1]
                        .try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE)
                        .unwrap(),
                ),
            ],
            adc_internal: driver.adc_internal,
            header_adc: driver.ltc2320,
            header_adc_data: [0u16; 8],
            driver_relay_state: driver.relay_sm, // this might live inside driver_output_state eventually
            interlock_handle: None,
        };

        let mut local = Local {
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            iir_state: [[[0.; 5]; IIR_CASCADE_LENGTH]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            header_adc_conversion_scheduled: stabilizer.systick.now(),
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
        // let del = shared.driver_relay_state[0].enable().unwrap();
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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, dacs, iir_state, generator], shared=[settings, signal_generator, telemetry], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            signal_generator,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            iir_state,
            generator,
        } = c.local;

        (settings, telemetry, signal_generator).lock(
            |settings, telemetry, signal_generator| {
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

                    for channel in 0..adc_samples.len() {
                        adc_samples[channel]
                            .iter()
                            .zip(dac_samples[channel].iter_mut())
                            .zip(&mut signal_generator[channel])
                            .map(|((ai, di), signal)| {
                                let x = f32::from(*ai as i16);
                                let y = settings.iir_ch[channel]
                                    .iter()
                                    .zip(iir_state[channel].iter_mut())
                                    .fold(x, |yi, (ch, state)| {
                                        ch.update(state, yi, hold)
                                    });

                                // Note(unsafe): The filter limits must ensure that the value is in range.
                                // The truncation introduces 1/2 LSB distortion.
                                let y: i16 = unsafe { y.to_int_unchecked() };

                                let y = y.saturating_add(signal);

                                // Convert to DAC code
                                *di = DacCode::from(y).0;
                            })
                            .last();
                    }

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

    #[task(priority = 1, local=[afes], shared=[network, settings, signal_generator, interlock_handle])]
    fn settings_update(
        mut c: settings_update::Context,
        path: Option<String<64>>,
    ) {
        let new_settings =
            c.shared.network.lock(|net| *net.miniconf.settings());
        let old_settings = c.shared.settings.lock(|current| *current);

        c.shared.interlock_handle.lock(|handle| {
            if let Some(action) = Interlock::action(
                path.as_ref()
                    .map(|path| path.as_str().trim_start_matches("interlock/")),
                handle.is_some(),
                &new_settings.interlock,
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

        // Update the signal generators
        for (i, &config) in new_settings.signal_generator.iter().enumerate() {
            match config.try_into_config(SAMPLE_PERIOD, DacCode::FULL_SCALE) {
                Ok(config) => {
                    c.shared
                        .signal_generator
                        .lock(|generator| generator[i].update_waveform(config));
                }
                Err(err) => log::error!(
                    "Failed to update signal generation on DAC{}: {:?}",
                    i,
                    err
                ),
            }
        }

        let target = new_settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
    }

    #[task(priority = 1, shared=[network, settings, telemetry, adc_internal], local=[cpu_temp_sensor])]
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
            c.shared.adc_internal.lock(|adc| [
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

    #[task(priority = 2, shared=[header_adc], local=[header_adc_conversion_scheduled])]
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

    #[task(binds = QUADSPI, priority = 2, shared=[header_adc, header_adc_data])]
    fn header_adc_transfer_done(c: header_adc_transfer_done::Context) {
        (c.shared.header_adc, c.shared.header_adc_data).lock(|ltc, data| {
            ltc.handle_transfer_done(data);
        });
    }

    // Task for waiting the relay transition times.
    #[task(priority = 1, shared=[driver_relay_state])]
    fn handle_relay(mut c: handle_relay::Context, channel: driver::Channel) {
        let delay = (c.shared.driver_relay_state)
            .lock(|state| state[channel as usize].handle_relay());
        if let Some(del) = delay {
            handle_relay::Monotonic::spawn_after(del.convert(), channel)
                .unwrap();
        }
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

    #[task(binds = SPI2, priority = 4)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 SPI error");
    }

    #[task(binds = SPI3, priority = 4)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 SPI error");
    }

    #[task(binds = SPI4, priority = 4)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 SPI error");
    }

    #[task(binds = SPI5, priority = 4)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 SPI error");
    }
}
