//! # Fibre Noise Cancellation
//!
//! Pounder samples the error signal input and mixes it down with a DDS at
//! 2*aom_f (channel::TWO). It passes it to Stabilizer for digital filtering
//! (normally a PI application) where it is read at a fixed rate. This is used
//! to feedback back into the phase offset of the DDS channel::ONE at aom_f
//! output through the Pounder. It currently only exposes CHANNEL 0 of the
//! Pounder for maximising feedback bandwidth, but can be easily extended to
//! expose both channels in the future if required and it proves sufficient
//!
//! Currently samples at 195.3 kHz, i.e. once in 5.12 us
//!
//! ## Features
//! * up to 200 kHz rate, timed sampling
//! * Run-time filter configuration
//! * Input/Output data streaming
//! * f32 IIR math
//! * Generic biquad (second order) IIR filter
//! * Anti-windup
//! * Derivative kick avoidance
//!
//! ## Settings
//! Refer to the [Settings] structure for documentation of run-time configurable settings for this
//! application.
//!
//! ## Telemetry
//! Refer to [Telemetry] for information about telemetry reported by this application.
//!
//! ## Livestreaming
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stabilizer::net::data_stream](../stabilizer/net/data_stream/index.html) for more information.
#![deny(warnings)]
#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::sync::atomic::{fence, Ordering};

use fugit::ExtU64;
use mutex_trait::prelude::*;

use idsp::iir;

use ad9959::phase_to_pow;
use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        hal,
        pounder::{
            ChannelConfig, ClockConfig, DdsChannelConfig, PounderConfig,
        },
        setup::PounderDevices as Pounder,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, SerialTerminal, SystemTimer, Systick,
        UsbDevice, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

// The number of samples in each batch process
// The Pounder dds can only be updated once per batch, else the QSPI stalls.
// Disabling the second channel would allow for a batch size of 2.
// This does not strictly need a DMA, but the right interrupt binding is needed otherwise
const BATCH_SIZE: usize = 1;

// The number of 100MHz timer ticks between each sample. Currently set to 12.80 us
// corresponding to a 78.125 kHz sampling rate.
const SAMPLE_TICKS: u32 = (2 << 10) + (2 << 8);

const DEFAULT_AOM_FREQUENCY: f32 = 80_000_000.0;

#[derive(Clone, Copy, Debug, Tree)]
pub struct Settings {
    /// Configure the Analog Front End (AFE) gain.
    ///
    /// # Path
    /// `afe/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// Any of the variants of [Gain] enclosed in double quotes.
    #[tree]
    afe: [Gain; 2],

    /// Configure the IIR filter parameters.
    ///
    /// # Path
    /// `iir_ch/<n>/<m>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    /// * `<m>` specifies which cascade to configure. `<m>` := [0, 1], depending on [IIR_CASCADE_LENGTH]
    ///
    /// # Value
    /// See [iir::IIR#Biquad]
    #[tree(depth(2))]
    iir_ch: [[iir::Biquad<f32>; IIR_CASCADE_LENGTH]; 2],

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

    /// Specifies the config for pounder DDS clock configuration, DDS channels & attenuations
    ///
    /// # Path
    /// `pounder`
    ///
    /// # Value
    /// See [PounderConfig#miniconf]
    #[tree(depth(4))]
    pounder: PounderConfig,
}

impl Default for Settings {
    fn default() -> Self {
        let mut i = iir::Biquad::IDENTITY;
        i.set_min(-SCALE);
        i.set_max(SCALE);

        let pounder = PounderConfig {
            clock: ClockConfig::default(),
            in_channel: [ChannelConfig {
                dds: DdsChannelConfig {
                    frequency: 2. * DEFAULT_AOM_FREQUENCY,
                    phase_offset: 0.,
                    amplitude: 1.0,
                },
                attenuation: 0.5,
            }; 2],
            out_channel: [ChannelConfig {
                dds: DdsChannelConfig {
                    frequency: DEFAULT_AOM_FREQUENCY,
                    phase_offset: 0.,
                    amplitude: 1.0,
                },
                attenuation: 31.5,
            }; 2],
        };

        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [Gain::G1, Gain::G1],
            // IIR filter tap gains are an array `[b0, b1, b2, a1, a2]` such that the
            // new output is computed as `y0 = a1*y1 + a2*y2 + b0*x0 + b1*x1 + b2*x2`.
            // The array is `iir_state[channel-index][cascade-index][coeff-index]`.
            // The IIR coefficients can be mapped to other transfer function
            // representations, for example as described in https://arxiv.org/abs/1508.06319
            iir_ch: [[i; IIR_CASCADE_LENGTH]; 2],

            // Permit the DI1 digital input to suppress filter output updates.
            allow_hold: false,
            // Force suppress filter output updates.
            force_hold: false,
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),

            // Pounder config initialisation
            pounder,
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use stabilizer::hardware::pounder;

    use super::*;

    #[monotonic(binds = SysTick, default = true, priority = 2)]
    type Monotonic = Systick;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Settings, Telemetry, 5>,
        settings: Settings,
        telemetry: TelemetryBuffer,
        pounder: Pounder,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        iir_state: [[[f32; 4]; IIR_CASCADE_LENGTH]; 2],
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
        phase_offsets: [u16; 2],
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let clock = SystemTimer::new(|| monotonics::now().ticks() as u32);

        // Configure the microcontroller
        let (stabilizer, pounder) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let device_settings = stabilizer.usb_serial.settings();
        let application_settings = Settings::default();

        let mut pounder =
            pounder.expect("Fibre noise cancellation requires a Pounder");
        pounder.update_dds(application_settings.pounder, &mut None);

        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &device_settings.broker,
            &device_settings.id,
            stabilizer.metadata,
            application_settings,
        );

        let generator = network.configure_streaming(StreamFormat::AdcDacData);

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            settings: application_settings,
            telemetry: TelemetryBuffer::default(),
            pounder,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            iir_state: [[[0.; 4]; IIR_CASCADE_LENGTH]; 2],
            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
            phase_offsets: [0u16; 2],
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();
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
    #[task(binds=DMA1_STR4, local=[digital_inputs, adcs, iir_state, generator, phase_offsets], shared=[settings, telemetry, pounder], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            pounder,
        } = c.shared;

        let process::LocalResources {
            digital_inputs,
            adcs: (adc0, adc1),
            iir_state,
            generator,
            phase_offsets,
        } = c.local;

        (settings, telemetry, pounder).lock(|settings, telemetry, pounder| {
            let digital_inputs =
                [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
            telemetry.digital_inputs = digital_inputs;

            let hold = settings.force_hold
                || (digital_inputs[1] && settings.allow_hold);

            (adc0, adc1).lock(|adc0, adc1| {
                let adc_samples = [adc0, adc1];
                let mut phase_offset_codes = [[0u16; BATCH_SIZE]; 2];
                let mut dds_profile = pounder.dds_output.builder();

                // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);

                for (channel_idx, (phase_offset, dds_channel)) in phase_offsets
                    .iter_mut()
                    .zip([pounder::Channel::Out0, pounder::Channel::Out1])
                    .enumerate()
                {
                    for (dma_idx, ai) in
                        adc_samples[channel_idx].iter().enumerate()
                    {
                        let power_in = f32::from(*ai as i16);

                        let iir_out = settings.iir_ch[channel_idx]
                            .iter()
                            .zip(iir_state[channel_idx].iter_mut())
                            .fold(power_in, |iir_accumulator, (ch, state)| {
                                let filter =
                                    if hold { &iir::Biquad::HOLD } else { ch };
                                filter.update(state, iir_accumulator)
                            });

                        *phase_offset = (*phase_offset
                            + phase_to_pow(iir_out).unwrap_or(0u16))
                            & 0x3FFFu16;

                        dds_profile
                            .update_channels(
                                dds_channel.into(),
                                None,
                                Some(*phase_offset),
                                None,
                            )
                            .write();
                        // settings.pounder.out_channel[channel_idx]
                        //     .dds
                        //     .phase_offset =
                        //     pow_to_phase(*phase_offset).unwrap_or(0.0);

                        phase_offset_codes[channel_idx][dma_idx] =
                            *phase_offset;

                        // pounder.lock(|pdr| {
                        //     pdr.update_dds_waveform(settings.pounder)
                        // });
                    }
                }

                const N: usize = BATCH_SIZE * core::mem::size_of::<u16>();

                generator.add(|buf| {
                    for (data, buf) in
                        adc_samples.iter().zip(buf.chunks_exact_mut(N))
                    {
                        let data = unsafe {
                            core::slice::from_raw_parts(
                                data.as_ptr() as *const MaybeUninit<u8>,
                                N,
                            )
                        };
                        buf.copy_from_slice(data)
                    }
                    for (data, buf) in phase_offset_codes
                        .iter()
                        .zip(buf.chunks_exact_mut(N).skip(adc_samples.len()))
                    {
                        let data = unsafe {
                            core::slice::from_raw_parts(
                                data.as_ptr() as *const MaybeUninit<u8>,
                                N,
                            )
                        };
                        buf.copy_from_slice(data)
                    }

                    // N bytes each for both adc_samples and phase_offsets over two channels
                    N * 4
                });

                // Update telemetry measurements.
                telemetry.adcs =
                    [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[1][0])];

                fence(Ordering::SeqCst);
            });
        });
    }

    #[idle(shared=[network, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.shared.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged(_path) => {
                    settings_update::spawn().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => {
                    // We can't sleep if USB is not in suspend.
                    if c.shared.usb.lock(|usb| {
                        usb.state()
                            == usb_device::device::UsbDeviceState::Suspend
                    }) {
                        cortex_m::asm::wfi();
                    }
                }
            }
        }
    }

    #[task(priority = 1, local=[afes], shared=[network, settings, pounder])]
    fn settings_update(mut c: settings_update::Context) {
        let incoming_settings =
            c.shared.network.lock(|net| *net.miniconf.settings());
        let mut current_clock_settings: Option<ClockConfig> =
            Some(ClockConfig::default());

        c.shared.settings.lock(|saved_settings| {
            current_clock_settings = Some(saved_settings.pounder.clock);
            *saved_settings = incoming_settings;
        });

        c.local.afes.0.set_gain(incoming_settings.afe[0]);
        c.local.afes.1.set_gain(incoming_settings.afe[1]);

        // Update Pounder configurations
        c.shared.pounder.lock(|pounder| {
            // let mut clocking = current_clock_settings.clock;
            pounder.update_dds(
                incoming_settings.pounder,
                &mut current_clock_settings,
            );
        });

        let target = incoming_settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
    }

    #[task(priority = 1, shared=[network, settings, telemetry, pounder], local=[cpu_temp_sensor])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.shared.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period, pounder_telemetry) =
            (c.shared.settings, c.shared.pounder).lock(|settings, pounder| {
                (
                    settings.afe,
                    settings.telemetry_period,
                    pounder.get_telemetry(),
                )
            });

        c.shared.network.lock(|net| {
            net.telemetry.publish(&telemetry.finalize(
                gains[0],
                gains[1],
                c.local.cpu_temp_sensor.get_temperature().unwrap(),
                Some(pounder_telemetry),
            ))
        });

        // Schedule the telemetry task in the future.
        telemetry::Monotonic::spawn_after((telemetry_period as u64).secs())
            .unwrap();
    }

    #[task(priority = 1, shared=[usb], local=[usb_terminal])]
    fn usb(mut c: usb::Context) {
        // Handle the USB serial terminal.
        c.shared.usb.lock(|usb| {
            usb.poll(&mut [c.local.usb_terminal.interface_mut().inner_mut()]);
        });

        c.local.usb_terminal.process().unwrap();

        // Schedule to run this task every 10 milliseconds.
        usb::spawn_after(10u64.millis()).unwrap();
    }

    #[task(priority = 1, shared=[network])]
    fn ethernet_link(mut c: ethernet_link::Context) {
        c.shared.network.lock(|net| net.processor.handle_link());
        ethernet_link::Monotonic::spawn_after(1.secs()).unwrap();
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
