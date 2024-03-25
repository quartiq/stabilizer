//! # Lockin
//!
//! The `lockin` application implements a lock-in amplifier using either an external or internally
//! generated reference.
//!
//! ## Features
//! * Up to 800 kHz sampling
//! * Up to 400 kHz modulation frequency
//! * Supports internal and external reference sources:
//!     1. Internal: Generate reference internally and output on one of the channel outputs
//!     2. External: Reciprocal PLL, reference input applied to DI0.
//! * Adjustable PLL and locking time constants
//! * Adjustable phase offset and harmonic index
//! * Run-time configurable output modes (in-phase, quadrature, magnitude, log2 power, phase, frequency)
//! * Input/output data streamng via UDP
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
#![no_std]
#![no_main]

use core::{
    convert::TryFrom,
    mem::MaybeUninit,
    sync::atomic::{fence, Ordering},
};

use rtic_monotonics::Monotonic;

use fugit::ExtU32;
use mutex_trait::prelude::*;

use idsp::{Accu, Complex, ComplexExt, Filter, Lockin, Lowpass, Repeat, RPLL};

use stabilizer::{
    hardware::{
        self,
        adc::{Adc0Input, Adc1Input, AdcCode},
        afe::Gain,
        dac::{Dac0Output, Dac1Output, DacCode},
        hal,
        input_stamper::InputStamper,
        signal_generator,
        timers::SamplingTimer,
        DigitalInput0, DigitalInput1, SerialTerminal, SystemTimer, Systick,
        UsbDevice, AFE0, AFE1,
    },
    net::{
        data_stream::{FrameGenerator, StreamFormat, StreamTarget},
        miniconf::Tree,
        serde::{Deserialize, Serialize},
        telemetry::{Telemetry, TelemetryBuffer},
        NetworkState, NetworkUsers,
    },
};

// The logarithm of the number of samples in each batch process. This corresponds with 2^3 samples
// per batch = 8 samples
const BATCH_SIZE_LOG2: u32 = 3;
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// The logarithm of the number of 100MHz timer ticks between each sample. This corresponds with a
// sampling period of 2^7 = 128 ticks. At 100MHz, 10ns per tick, this corresponds to a sampling
// period of 1.28 uS or 781.25 KHz.
const SAMPLE_TICKS_LOG2: u32 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
enum Conf {
    /// Output the lockin magnitude.
    Magnitude,
    /// Output the phase of the lockin
    Phase,
    /// Output the lockin reference frequency as a sinusoid
    ReferenceFrequency,
    /// Output the logarithmic power of the lockin
    LogPower,
    /// Output the in-phase component of the lockin signal.
    InPhase,
    /// Output the quadrature component of the lockin signal.
    Quadrature,
    /// Output the lockin internal modulation frequency as a sinusoid
    Modulation,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize, PartialEq)]
enum LockinMode {
    /// Utilize an internally generated reference for demodulation
    Internal,
    /// Utilize an external modulation signal supplied to DI0
    External,
}

#[derive(Copy, Clone, Debug, Tree)]
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

    /// Specifies the operational mode of the lockin.
    ///
    /// # Path
    /// `lockin_mode`
    ///
    /// # Value
    /// One of the variants of [LockinMode] enclosed in double quotes.
    lockin_mode: LockinMode,

    /// Specifis the PLL time constant.
    ///
    /// # Path
    /// `pll_tc/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// The PLL time constant exponent (1-31).
    pll_tc: [u32; 2],

    /// Specifies the lockin lowpass gains.
    ///
    /// # Path
    /// `lockin_k`
    ///
    /// # Value
    /// The lockin low-pass coefficients. See [`idsp::Lowpass`] for determining them.
    lockin_k: <Lowpass<2> as Filter>::Config,

    /// Specifies which harmonic to use for the lockin.
    ///
    /// # Path
    /// `lockin_harmonic`
    ///
    /// # Value
    /// Harmonic index of the LO. -1 to _de_modulate the fundamental (complex conjugate)
    lockin_harmonic: i32,

    /// Specifies the LO phase offset.
    ///
    /// # Path
    /// `lockin_phase`
    ///
    /// # Value
    /// Demodulation LO phase offset. Units are in terms of i32, where [i32::MIN] is equivalent to
    /// -pi and [i32::MAX] is equivalent to +pi.
    lockin_phase: i32,

    /// Specifies DAC output mode.
    ///
    /// # Path
    /// `output_conf/<n>`
    ///
    /// * `<n>` specifies which channel to configure. `<n>` := [0, 1]
    ///
    /// # Value
    /// One of the variants of [Conf] enclosed in double quotes.
    #[tree]
    output_conf: [Conf; 2],

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
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [Gain::G1; 2],

            lockin_mode: LockinMode::External,

            pll_tc: [21, 21], // frequency and phase settling time (log2 counter cycles)

            lockin_k: [0x8_0000, -0x400_0000], // lockin lowpass gains
            lockin_harmonic: -1, // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
            lockin_phase: 0,     // Demodulation LO phase offset

            output_conf: [Conf::InPhase, Conf::Quadrature],
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream_target: StreamTarget::default(),
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Settings, Telemetry, 2>,
        settings: Settings,
        telemetry: TelemetryBuffer,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        timestamper: InputStamper,
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        pll: RPLL,
        lockin: Lockin<Repeat<2, Lowpass<2>>>,
        signal_generator: signal_generator::SignalGenerator,
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks() as u32);

        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup::setup(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let settings = stabilizer.usb_serial.settings();
        let mut network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &settings.broker,
            &settings.id,
            stabilizer.metadata,
        );

        let generator = network.configure_streaming(StreamFormat::AdcDacData);

        let shared = Shared {
            network,
            usb: stabilizer.usb,
            telemetry: TelemetryBuffer::default(),
            settings: Settings::default(),
        };

        let signal_config = signal_generator::Config {
            // Same frequency as batch size.
            phase_increment: [1 << (32 - BATCH_SIZE_LOG2); 2],
            // 1V Amplitude
            amplitude: DacCode::try_from(1.0).unwrap().into(),
            signal: signal_generator::Signal::Cosine,
            phase_offset: 0,
        };

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            timestamper: stabilizer.timestamper,

            pll: RPLL::new(SAMPLE_TICKS_LOG2 + BATCH_SIZE_LOG2),
            lockin: Lockin::default(),
            signal_generator: signal_generator::SignalGenerator::new(
                signal_config,
            ),

            generator,
            cpu_temp_sensor: stabilizer.temperature_sensor,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        // Spawn a settings and telemetry update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        start::spawn().unwrap();
        usb::spawn().unwrap();

        // Start recording digital input timestamps.
        stabilizer.timestamp_timer.start();

        // Enable the timestamper.
        local.timestamper.start();

        (shared, local)
    }

    #[task(priority = 1, local=[sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(100.millis()).await;
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of a externally (DI0) referenced PLL lockin on the ADC0 signal.
    /// It outputs either I/Q or power/phase on DAC0/DAC1. Data is normalized to full scale.
    /// PLL bandwidth, filter bandwidth, slope, and x/y or power/phase post-filters are available.
    #[task(binds=DMA1_STR4, shared=[settings, telemetry], local=[adcs, dacs, lockin, timestamper, pll, generator, signal_generator], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            settings,
            telemetry,
            ..
        } = c.shared;

        let process::LocalResources {
            timestamper,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            pll,
            lockin,
            signal_generator,
            generator,
            ..
        } = c.local;

        (settings, telemetry).lock(|settings, telemetry| {
            let (reference_phase, reference_frequency) =
                match settings.lockin_mode {
                    LockinMode::External => {
                        let timestamp =
                            timestamper.latest_timestamp().unwrap_or(None); // Ignore data from timer capture overflows.
                        let (pll_phase, pll_frequency) = pll.update(
                            timestamp.map(|t| t as i32),
                            settings.pll_tc[0],
                            settings.pll_tc[1],
                        );
                        (pll_phase, (pll_frequency >> BATCH_SIZE_LOG2) as i32)
                    }
                    LockinMode::Internal => {
                        // Reference phase and frequency are known.
                        (1i32 << 30, 1i32 << (32 - BATCH_SIZE_LOG2))
                    }
                };

            let sample_frequency =
                reference_frequency.wrapping_mul(settings.lockin_harmonic);
            let sample_phase = settings.lockin_phase.wrapping_add(
                reference_phase.wrapping_mul(settings.lockin_harmonic),
            );

            (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                let adc_samples = [adc0, adc1];
                let mut dac_samples = [dac0, dac1];

                // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);

                let output: Complex<i32> = adc_samples[0]
                    .iter()
                    // Zip in the LO phase.
                    .zip(Accu::new(sample_phase, sample_frequency))
                    // Convert to signed, MSB align the ADC sample, update the Lockin (demodulate, filter)
                    .map(|(&sample, phase)| {
                        let s = (sample as i16 as i32) << 16;
                        lockin.update(s, phase, &settings.lockin_k)
                    })
                    // Decimate
                    .last()
                    .unwrap()
                    * 2; // Full scale assuming the 2f component is gone.

                // Convert to DAC data.
                for (channel, samples) in dac_samples.iter_mut().enumerate() {
                    for sample in samples.iter_mut() {
                        let value = match settings.output_conf[channel] {
                            Conf::Magnitude => output.abs_sqr() as i32 >> 16,
                            Conf::Phase => output.arg() >> 16,
                            Conf::LogPower => output.log2() << 8,
                            Conf::ReferenceFrequency => {
                                reference_frequency >> 16
                            }
                            Conf::InPhase => output.re >> 16,
                            Conf::Quadrature => output.im >> 16,

                            Conf::Modulation => {
                                signal_generator.next().unwrap() as i32
                            }
                        };

                        *sample = DacCode::from(value as i16).0;
                    }
                }

                // Stream the data.
                const N: usize = BATCH_SIZE * core::mem::size_of::<i16>()
                    / core::mem::size_of::<MaybeUninit<u8>>();
                generator.add(|buf| {
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
                    N * 4
                });

                // Update telemetry measurements.
                telemetry.adcs =
                    [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[1][0])];

                telemetry.dacs =
                    [DacCode(dac_samples[0][0]), DacCode(dac_samples[1][0])];

                // Preserve instruction and data ordering w.r.t. DMA flag access.
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

    #[task(priority = 1, local=[afes], shared=[network, settings])]
    async fn settings_update(mut c: settings_update::Context) {
        let settings = c.shared.network.lock(|net| *net.miniconf.settings());
        c.shared.settings.lock(|current| *current = settings);

        c.local.afes.0.set_gain(settings.afe[0]);
        c.local.afes.1.set_gain(settings.afe[1]);

        let target = settings.stream_target.into();
        c.shared.network.lock(|net| net.direct_stream(target));
    }

    #[task(priority = 1, local=[digital_inputs, cpu_temp_sensor], shared=[network, settings, telemetry])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let mut telemetry: TelemetryBuffer =
                c.shared.telemetry.lock(|telemetry| *telemetry);

            telemetry.digital_inputs = [
                c.local.digital_inputs.0.is_high(),
                c.local.digital_inputs.1.is_high(),
            ];

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
            Systick::delay((telemetry_period as u32).secs()).await;
        }
    }

    #[task(priority = 1, shared=[usb], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) {
        loop {
            // Handle the USB serial terminal.
            c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()]);
            });

            c.local.usb_terminal.process().unwrap();

            // Schedule to run this task every 10 milliseconds.
            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) {
        loop {
            c.shared.network.lock(|net| net.processor.handle_link());
            Systick::delay(1.secs()).await;
        }
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
    }
}
