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
//! Refer to the [Lockin] structure for documentation of run-time configurable settings
//! for this application.
//!
//! ## Telemetry
//! Refer to [stabilizer::telemetry::Telemetry] for information about telemetry reported by this application.
//!
//! ## Stream
//! This application streams raw ADC and DAC data over UDP. Refer to
//! [stream] for more information.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::{
    iter,
    mem::MaybeUninit,
    sync::atomic::{fence, Ordering},
};

use fugit::ExtU32;
use idsp::{Accu, Complex, ComplexExt, Filter, Lowpass, Repeat, RPLL};
use miniconf::{Leaf, Tree};
use rtic_monotonics::Monotonic;
use serde::{Deserialize, Serialize};

use stabilizer::convert::{AdcCode, DacCode, Gain};

use platform::{AppSettings, NetSettings};

// The logarithm of the number of samples in each batch process. This corresponds with 2^3 samples
// per batch = 8 samples
const BATCH_SIZE_LOG2: u32 = 3;
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// The logarithm of the number of 100MHz timer ticks between each sample. This corresponds with a
// sampling period of 2^7 = 128 ticks. At 100MHz, 10ns per tick, this corresponds to a sampling
// period of 1.28 uS or 781.25 KHz.
const SAMPLE_TICKS_LOG2: u32 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    lockin: Lockin,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            lockin: Lockin::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            lockin: Lockin::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

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

#[derive(Copy, Clone, Debug, PartialEq, Serialize, Deserialize)]
enum LockinMode {
    /// Utilize an internally generated reference for demodulation
    Internal,
    /// Utilize an external modulation signal supplied to DI0
    External,
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct Lockin {
    /// Configure the Analog Front End (AFE) gain.
    afe: [Leaf<Gain>; 2],

    /// Specifies the operational mode of the lockin.
    #[tree(with=miniconf::leaf)]
    lockin_mode: LockinMode,

    /// Specifis the PLL time constant.
    ///
    /// The PLL time constant exponent (1-31).
    pll_tc: [u32; 2],

    /// Specifies the lockin lowpass gains.
    #[tree(with=miniconf::leaf)]
    lockin_k: <Lowpass<2> as Filter>::Config,

    /// Specifies which harmonic to use for the lockin.
    ///
    /// Harmonic index of the LO. -1 to _de_modulate the fundamental (complex conjugate)
    lockin_harmonic: i32,

    /// Specifies the LO phase offset.
    ///
    /// Demodulation LO phase offset. Units are in terms of i32, where [i32::MIN] is equivalent to
    /// -pi and [i32::MAX] is equivalent to +pi.
    lockin_phase: i32,

    /// Specifies DAC output mode.
    output_conf: [Leaf<Conf>; 2],

    /// Specifies the telemetry output period in seconds.
    telemetry_period: u16,

    /// Specifies the target for data streaming.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for Lockin {
    fn default() -> Self {
        Self {
            afe: [Leaf(Gain::G1); 2],

            lockin_mode: LockinMode::External,

            pll_tc: [21, 21], // frequency and phase settling time (log2 counter cycles)

            lockin_k: [0x8_0000, -0x400_0000], // lockin lowpass gains
            lockin_harmonic: -1, // Harmonic index of the LO: -1 to _de_modulate the fundamental (complex conjugate)
            lockin_phase: 0,     // Demodulation LO phase offset

            output_conf: [Leaf(Conf::InPhase), Leaf(Conf::Quadrature)],
            // The default telemetry period in seconds.
            telemetry_period: 10,

            stream: Default::default(),
        }
    }
}

#[cfg(not(target_os = "none"))]
fn main() {
    use miniconf::{json::to_json_value, json_schema::TreeJsonSchema};
    let s = Settings::default();
    println!(
        "{}",
        serde_json::to_string_pretty(&to_json_value(&s).unwrap()).unwrap()
    );
    let mut schema = TreeJsonSchema::new(Some(&s)).unwrap();
    schema
        .root
        .insert("title".to_string(), "Stabilizer lockin".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;
    use stabilizer::{
        hardware::{
            self,
            adc::{Adc0Input, Adc1Input},
            dac::{Dac0Output, Dac1Output},
            hal,
            input_stamper::InputStamper,
            net::{NetworkState, NetworkUsers},
            timers::SamplingTimer,
            DigitalInput0, DigitalInput1, Pgia, SerialTerminal, SystemTimer,
            Systick, UsbDevice,
        },
        telemetry::TelemetryBuffer,
    };
    use stream::FrameGenerator;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Lockin>,
        settings: Settings,
        active_settings: Lockin,
        telemetry: TelemetryBuffer,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        timestamper: InputStamper,
        afes: [Pgia; 2],
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        pll: RPLL,
        lockin: idsp::Lockin<Repeat<2, Lowpass<2>>>,
        source: idsp::AccuOsc<iter::Repeat<i64>>,
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup::setup::<Settings>(
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
            &stabilizer.settings.net,
            stabilizer.metadata,
        );

        let generator = network.configure_streaming(stream::Format::AdcDacData);

        let shared = Shared {
            network,
            usb: stabilizer.usb,
            telemetry: TelemetryBuffer::default(),
            active_settings: stabilizer.settings.lockin.clone(),
            settings: stabilizer.settings,
        };

        let source =
            idsp::AccuOsc::new(iter::repeat(1i64 << (64 - BATCH_SIZE_LOG2)));

        let mut local = Local {
            usb_terminal: stabilizer.usb_serial,
            sampling_timer: stabilizer.adc_dac_timer,
            digital_inputs: stabilizer.digital_inputs,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            timestamper: stabilizer.timestamper,

            pll: RPLL::new(SAMPLE_TICKS_LOG2 + BATCH_SIZE_LOG2),
            lockin: idsp::Lockin::default(),
            source,

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
    #[task(binds=DMA1_STR4, shared=[active_settings, telemetry], local=[adcs, dacs, lockin, timestamper, pll, generator, source], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(c: process::Context) {
        let process::SharedResources {
            active_settings,
            telemetry,
            ..
        } = c.shared;

        let process::LocalResources {
            timestamper,
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            pll,
            lockin,
            source,
            generator,
            ..
        } = c.local;

        (active_settings, telemetry).lock(|settings, telemetry| {
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
                        let value = match *settings.output_conf[channel] {
                            Conf::Magnitude => output.abs_sqr() as i32 >> 16,
                            Conf::Phase => output.arg() >> 16,
                            Conf::LogPower => output.log2() << 8,
                            Conf::ReferenceFrequency => {
                                reference_frequency >> 16
                            }
                            Conf::InPhase => output.re >> 16,
                            Conf::Quadrature => output.im >> 16,

                            Conf::Modulation => source.next().unwrap().re,
                        };

                        *sample = DacCode::from(value as i16).0;
                    }
                }

                // Stream the data.
                const N: usize = BATCH_SIZE * size_of::<i16>()
                    / size_of::<MaybeUninit<u8>>();
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

    #[idle(shared=[settings, network, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.lockin))
            {
                NetworkState::SettingsChanged => {
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

    #[task(priority = 1, local=[afes], shared=[network, settings, active_settings])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes[0].set_gain(*settings.lockin.afe[0]);
            c.local.afes[1].set_gain(*settings.lockin.afe[1]);

            c.shared
                .network
                .lock(|net| net.direct_stream(settings.lockin.stream));

            c.shared
                .active_settings
                .lock(|current| *current = settings.lockin.clone());
        });
    }

    #[task(priority = 1, local=[digital_inputs, cpu_temp_sensor], shared=[network, settings, telemetry])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let mut telemetry =
                c.shared.telemetry.lock(|telemetry| telemetry.clone());

            telemetry.digital_inputs = [
                c.local.digital_inputs.0.is_high(),
                c.local.digital_inputs.1.is_high(),
            ];

            let (gains, telemetry_period) =
                c.shared.settings.lock(|settings| {
                    (settings.lockin.afe, settings.lockin.telemetry_period)
                });

            c.shared.network.lock(|net| {
                net.telemetry.publish(&telemetry.finalize(
                    *gains[0],
                    *gains[1],
                    c.local.cpu_temp_sensor.get_temperature().unwrap(),
                ))
            });

            // Schedule the telemetry task in the future.
            Systick::delay((telemetry_period as u32).secs()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
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

            c.shared.settings.lock(|settings| {
                if c.local.usb_terminal.poll(settings).unwrap() {
                    settings_update::spawn().unwrap()
                }
            });

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
