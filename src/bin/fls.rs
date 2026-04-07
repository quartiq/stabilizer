#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

//! Patent pending: DE102021112017A1
//!
//! # Algorithm description
//!
//! This application can be understood as a universal phase (frequency)
//! signal processor. It determines the phase (we will drop frequency
//! from now on as in a phase-aware system frequency is merely the
//! difference between successive phases) of an RF input signal and
//! emits an RF output signal with a phase that depends on the input
//! phase. The transfer function between input and output phase is a
//! sequence of various types of filters (analog RC, digital FIR, IIR,
//! unwrapping, scaling, clipping) designed to implement either
//! high-quality phase measurements or a certain constrained and
//! somewhat exotic phase locked loop that is highly applicable
//! to the task of stabilizing the arm length of an optical Michelson
//! interferometer which in turn occurs when stabilizing the effective
//! path length of an optical frequency transmission system.
//!
//! The sequence of processing steps is as follows. Analyzing it's
//! application in the context of optical path length stabilization including
//! laser sources, optical modulators, and photodetectors optical is left as
//! an exercise for the user.
//!
//! ## PLL path
//!
//! * DDS locks its sysclk (500 MHz) to XO or external ref
//! * DDS emits SYNC signal at sysclk/4
//! * Prescaler 1/4 (in CPU)
//! * Drives CPU timer counter
//! * Counter is captured once per batch (based on CPU clock).
//!   See [stabilizer::hardware::pounder::timestamp].
//! * Digital PLL reconstructs SYNC frequency and phase (thus sysclk)
//!   w.r.t. batch and sample frequency and phase.
//!   This determines the relation of the CPU 8 MHz crystal (thus CPU
//!   clock and timers) to the DDS clock (derived from an external reference
//!   frequency or internal XCO). See [idsp::PLL].
//!
//! ## Signal path
//!
//! * RF signal enters Pounder at Pounder IN0
//! * Adjustable attenuation `demod_att`.
//! * 30 dB gain block
//! * Mixing with DDS at `demod_freq`
//! * RC lowpass and amplification to reject unwanted demodulation products and
//!   harmonics
//! * IF signal enters Stabilizer and is available at ADC0 for analog monitoring
//! * 2x PGIA and AA filter on Stabilizer
//! * ADC digitization at 1/1.28 µs interval
//! * Data processing in batches of 8 samples
//! * Digital mixing with the reconstructed sample phase (PLL path). See [idsp::Lockin].
//! * Lowpass filtering with a second order (12 dB/octave)
//!   IIR lowpass with an additional double zero at Nyquist. Adjustable corner frequency.
//!   See [idsp::Lowpass]
//! * Full rate baseband demodulated data (quadrature only) on DAC0
//! * Lowpass filtering with a batch-size boxcar FIR filter (zeros at n/4 Nyquist)
//! * Computation of signal power and phase.
//! * Fractional rescaling (`phase_scale`) and unwrapping of the phase with 32 bit turn range.
//! * Scaling and clamping.
//! * Filtering by a second order (biquad) IIR filter (supporting e.g. II, I, P
//!   action). See [idsp::iir].
//! * Clamping, output offset, and anti-windup. See [idsp::iir].
//! * Feedback onto a frequency offset of the modulation DDS at `mod_freq`
//! * Additional feedback path from the phase before unwrapping onto the
//!   modulation DDS phase offset with an adjustable gain `pow_gain`
//! * Adjustable DDS output amplitude and blanking on digital input
//! * Adjustable modulation attenuation `mod_att`
//! * Modulation output at Pounder OUT0
//!
//! # Telemetry
//! Data is regularly published via MQTT. See [CookedTelemetry].
//!
//! # Streaming
//! Full-rate ADC and DAC data is available via configurable UDP data streaming.
//! See [stream]. To view and analyze noise spectra the graphical application
//! [`stabilizer-stream`](https://github.com/quartiq/stabilizer-stream) can be used.

use core::num::Wrapping as W;

use arbitrary_int::{u10, u24};
use core::sync::atomic::{Ordering, fence};
use dsp_fixedpoint::P32;
use dsp_process::{Process, SplitProcess};
use idsp::{Complex, PLL, PLLState, Unwrapper};
use miniconf::Tree;
use serde::Serialize;

use ad9959::Acr;
use platform::{AppSettings, NetSettings};
use stabilizer::{convert::Gain, fls, statistics};

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    fls: Fls,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            fls: Default::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            fls: Default::default(),
            net: NetSettings::new(self.net.mac),
        };
    }
}

/// Settings structure for the application.
/// All fields in this structure are available through MQTT and can be configured at runtime.
#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct Fls {
    /// External 100 MHz reference
    ext_clk: bool,

    /// Channel-specific settings.
    channel: [Channel; 2],

    /// Lockin demodulation oscillator PLL crossover in Hz.
    /// This PLL reconstructs the DDS SYNC clock output on the CPU clock timescale.
    pll_bandwidth: f32,

    /// PLL lead-lag pole-zero ratio
    pll_split: f32,

    /// Telemetry output period in seconds.
    telemetry_period: f32,

    /// Specifies the target for data streaming.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,

    /// Activate settings
    ///
    /// If `true` each settings change immediately results in activation.
    /// If `false`, activation is suppressed.
    /// Use this to synchronize changes.
    activate: bool,
}

impl Default for Fls {
    fn default() -> Self {
        Self {
            ext_clk: false,
            channel: Default::default(),
            pll_bandwidth: 1e-4 / 10.24e-6,
            pll_split: 4.0,
            telemetry_period: 10.,
            stream: Default::default(),
            activate: true,
        }
    }
}

#[derive(Clone, Debug, miniconf::Tree)]
struct Dds {
    /// Frequency tuning word (i32)
    /// The DDS sample clock is nominally 500 MHz.
    frequency: W<i32>,

    /// Attenuation (dB)
    /// Range [0, 31.5]
    attenuation: f32,
}

#[derive(Clone, Debug, Tree)]
struct Channel {
    /// Input (demodulation) DDS settings
    ///
    /// The demodulation frequency must include the second LO offset
    demodulate: Dds,

    /// Demodulation LO DDS amplitude control register.
    ///
    /// # Value
    /// AD9959 amplitude control register (24 bits, see datasheet)
    ///
    /// # Default
    /// 0 for full scale amplitude and multiplier disable
    #[tree(with=miniconf::leaf)]
    demodulate_acr: u24,

    /// AFE gain
    #[tree(with=miniconf::leaf)]
    afe_gain: Gain,

    /// Output (modulation) DDS settings
    ///
    /// Feedback to stabilize the RF input phase is applied to the RF output
    /// on top of the output frequency
    modulate: Dds,

    /// Allow digital input channel to clear phase/frequency IIR and hold amplitude IIR
    hold_en: bool,

    /// Channel DSP components
    dsp: fls::Channel,
}

impl Default for Channel {
    fn default() -> Self {
        // Default modulation/demodulation frequency for characterization.
        // High CTZ has fewest DDS phase truncation spurs. Near 160 MHz.
        const F_DEMOD: W<i32> = W(0x5200_0000);

        Self {
            demodulate: Dds {
                frequency: F_DEMOD + W(0x20_0000), // LSB
                attenuation: 31.5,
            },
            modulate: Dds {
                frequency: F_DEMOD,
                attenuation: 6.0,
            },
            demodulate_acr: u24::new(0),
            hold_en: false,
            afe_gain: Gain::G5,
            dsp: Default::default(),
        }
    }
}

#[derive(Debug, Clone)]
struct ChannelConfig {
    dsp: fls::ChannelConfig,
    hold_en: bool,
    modulate_frequency: W<i32>,
}

#[derive(Debug, Clone)]
pub struct Config {
    channel: [ChannelConfig; 2],
    pll: PLL,
}

#[derive(Default, Debug, Clone)]
pub struct ChannelState {
    dsp: fls::ChannelState,
    phase: statistics::State,
    power: statistics::State,
}

impl ChannelState {
    fn update(&mut self) {
        self.phase.update(self.dsp.phase.xy.x0());
        self.power.update(self.dsp.power.inner as _);
    }

    fn stream(&self) -> StreamChannel {
        StreamChannel {
            demod: self.dsp.demod,
            phase: bytemuck::cast(self.dsp.unwrap.y),
            delta_ftw: W(self.dsp.phase.xy.y0()),
            aux_adc: self.dsp.amplitude.x0() as _,
            mod_amp: (self.dsp.amplitude.y0() >> 21) as _,
        }
    }
}

#[derive(Default, Debug, Clone)]
pub struct State {
    channel: [ChannelState; 2],
    pll: PLLState,
    pll_time: Unwrapper<i64>,
}

impl State {
    fn update(&mut self) {
        self.pll_time.process(self.pll.phase().0);
        for c in self.channel.iter_mut() {
            c.update();
        }
    }

    fn stream(&self) -> Stream {
        Stream {
            pll: self.pll.phase(),
            channel: self.channel.each_ref().map(|s| s.stream()),
        }
    }
}

impl Fls {
    fn build(&self) -> Config {
        Config {
            channel: self.channel.each_ref().map(|c| ChannelConfig {
                dsp: c.dsp.build(),
                hold_en: c.hold_en,
                modulate_frequency: c.modulate.frequency,
            }),
            pll: PLL::from_bandwidth(
                self.pll_bandwidth * 10.24e-6,
                self.pll_split,
            ),
        }
    }
}

/// Stream data format.
#[derive(Clone, Copy, Debug, Default, bytemuck::Zeroable, bytemuck::Pod)]
#[repr(C)]
struct Stream {
    pll: W<i32>,
    channel: [StreamChannel; 2],
}

#[derive(Clone, Copy, Debug, Default, bytemuck::Zeroable, bytemuck::Pod)]
#[repr(C)]
struct StreamChannel {
    /// Demodulated signal.
    demod: Complex<i32>,
    /// Current raw phase including wraps and pll correction.
    phase: [u32; 2],
    /// Current frequency tuning word added to the configured modulation
    /// offset `mod_freq`.
    delta_ftw: W<i32>,
    /// AUX ADC sample
    aux_adc: u16,
    mod_amp: u16,
}

/// Channel Telemetry, all undersampled and not AA filtered
#[derive(Default, Clone, Serialize)]
struct ChannelTelemetry {
    /// Current phase.
    phase_raw: i64,
    /// Demodulated phase statistics in turns
    phase: statistics::ScaledStatistics,
    /// Demodulated power (amplitude squared) statistics re full scale
    power: statistics::ScaledStatistics,
    /// Auxiliary front panel ADC input values, V
    aux_adc: f32,
    /// Modulation amplitude re full scale
    mod_amp: f32,
    /// Number of cycles where digital input held amplitude and
    /// cleared the frequency servo.
    holds: W<u32>,
    /// Number of potential phase slips where the absolute
    /// phase difference between successive samples is larger than π/2.
    slips: W<u32>,
    /// Counter for the number of samples with low power where the
    /// frequency servo was cleared.
    blanks: W<u32>,
    /// Total RF power in dBm as reported by the RF detector.
    rf_power: f32,
}

/// Telemetry structure.
/// This structure is published via MQTT at the `telemetry_interval` configured in
/// [Settings].
#[derive(Default, Clone, Serialize)]
pub struct CookedTelemetry {
    /// PLL time
    /// DDS PLL time as seen by CPU (sample) clock. Units of `1ns / (1 <<11)`.
    /// Settles increments of approximately `0x140_0000`.
    pll_time: i64,
    /// Channel telemetry, mostly "stateful" and undersampled
    channel: [ChannelTelemetry; 2],
    /// Pounder board temperature
    pounder_temp: f32,
    /// CPU temperature,
    cpu_temp: f32,
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
        .insert("title".to_string(), "Stabilizer fls".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[cfg_attr(target_os = "none", rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC]))]
mod app {
    use fugit::ExtU32 as _;
    use rtic_monotonics::Monotonic;

    use stabilizer::hardware::{
        self, DigitalInput0, DigitalInput1, Pgia, SerialTerminal, SystemTimer,
        Systick, UsbDevice,
        adc::{Adc0Input, Adc1Input},
        cpu_temp_sensor::CpuTempSensor,
        dac::{Dac0Output, Dac1Output},
        hal,
        net::{NetworkState, NetworkUsers},
        pounder::{
            Channel, PounderDevices, dds_output::DdsOutput,
            timestamp::Timestamper,
        },
        timers::SamplingTimer,
    };

    use stream::FrameGenerator;

    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<Fls>,
        settings: Settings,
        config: Config,
        state: State,
        dds_output: DdsOutput,
        pounder: PounderDevices,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        digital_inputs: (DigitalInput0, DigitalInput1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        generator: FrameGenerator,
        timestamper: Timestamper,
        afes: [Pgia; 2],
        cpu_temp_sensor: CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (mut carrier, mezzanine, _eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            fls::BATCH_SIZE,
            1 << fls::SAMPLE_TICKS_E,
        );

        let mut network = NetworkUsers::new(
            carrier.network_devices.stack,
            carrier.network_devices.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &carrier.settings.net,
            carrier.metadata,
        );

        let generator = network.configure_streaming(stream::Format::Fls2);

        let hardware::setup::Mezzanine::Pounder(mut pounder) = mezzanine else {
            panic!("Missing Pounder Mezzanine");
        };
        pounder.timestamper.start();

        // Enable ADC/DAC events
        carrier.adcs.0.start();
        carrier.adcs.1.start();
        carrier.dacs.0.start();
        carrier.dacs.1.start();

        let shared = Shared {
            usb: carrier.usb,
            network,
            config: carrier.settings.fls.build(),
            settings: carrier.settings,
            dds_output: pounder.dds_output,
            pounder: pounder.pounder,
            state: Default::default(),
        };

        let local = Local {
            usb_terminal: carrier.usb_serial,
            sampling_timer: carrier.sampling_timer,
            digital_inputs: carrier.digital_inputs,
            adcs: carrier.adcs,
            dacs: carrier.dacs,
            generator,
            timestamper: pounder.timestamper,
            afes: carrier.afes,
            cpu_temp_sensor: carrier.temperature_sensor,
        };

        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        aux_adc::spawn().unwrap();
        usb::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        start::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local = [sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(200.millis()).await;
        c.local.sampling_timer.start();
    }

    /// Main DSP processing routine.
    ///
    /// See `dual-iir` for general notes on processing time and timing.
    ///
    /// This is an implementation of fiber length stabilization using super-heterodyne
    /// (pounder + lockin) and digital feedback to a DDS.
    #[task(binds = DMA1_STR4, local=[timestamper, adcs, dacs, generator, digital_inputs], shared = [config, state, dds_output], priority = 3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::LocalResources {
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            digital_inputs,
            timestamper,
            generator,
            ..
        } = c.local;

        (c.shared.state, c.shared.config, c.shared.dds_output).lock(
            |state, config, dds_output| {
                (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                    fence(Ordering::SeqCst);
                    config.channel[0].dsp.demodulate::<{ fls::BATCH_SIZE }>(
                        &mut state.channel[0].dsp,
                        (**adc0).try_into().unwrap(),
                        (*dac0).try_into().unwrap(),
                    );
                    config.channel[1].dsp.demodulate::<{ fls::BATCH_SIZE }>(
                        &mut state.channel[1].dsp,
                        (**adc1).try_into().unwrap(),
                        (*dac1).try_into().unwrap(),
                    );
                    fence(Ordering::SeqCst);
                });

                // A counter running at a fourth of the DDS SYNC interval is captured by
                // the overflow of a timer synchronized to the sampling timer (locked to the
                // CPU clock and the other CPU timer clocks).
                // Captured timestamps are about 0x140 counts apart between batches
                // for default parameters.
                // They determine the phase and period of the DDS clock (driving the counter)
                // in terms of the CPU clock (driving the capture).
                // Discard double captures (overcaptures) and extrapolate.
                // Extrapolate on no capture (undercapture).
                let t = if let Ok(Some(t)) = timestamper.latest_timestamp() {
                    W((t as i32) << 16)
                } else {
                    log::warn!("timestamp under/over");
                    state.pll.clamp.x0 - state.pll.frequency()
                };
                let phase = -config.pll.process(&mut state.pll, t);

                let mut builder = dds_output.builder();
                let hold =
                    [digital_inputs.0.is_high(), digital_inputs.1.is_high()];
                for ((idx, hold), (config, state)) in
                    [Channel::Out0, Channel::Out1].into_iter().zip(hold).zip(
                        config.channel.iter().zip(state.channel.iter_mut()),
                    )
                {
                    state.dsp.hold = hold && config.hold_en;
                    config.dsp.update(&mut state.dsp, phase);
                    builder.push(
                        idx.into(),
                        Some(
                            config.modulate_frequency
                                + W(state.dsp.phase.xy.y0()),
                        ),
                        None,
                        Some(Acr::DEFAULT.with_multiplier(true).with_asf(
                            u10::new((state.dsp.amplitude.y0() >> 21) as _),
                        )),
                    );
                }
                dds_output.write(builder);
                state.update();
                generator.add(|buf| {
                    const N: usize = core::mem::size_of::<Stream>();
                    buf[..N].copy_from_slice(bytemuck::cast_slice(
                        bytemuck::bytes_of(&state.stream()),
                    ));
                    N
                });
            },
        );
    }

    #[idle(shared=[network, usb, settings])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.fls))
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

    #[task(priority = 1, shared=[network, settings, dds_output, pounder, config], local=[afes])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.local.afes[0].set_gain(settings.fls.channel[0].afe_gain);
            c.local.afes[1].set_gain(settings.fls.channel[1].afe_gain);

            c.shared.pounder.lock(|p| {
                for (ch, att) in [
                    (
                        Channel::In0,
                        settings.fls.channel[0].demodulate.attenuation,
                    ),
                    (
                        Channel::Out0,
                        settings.fls.channel[0].modulate.attenuation,
                    ),
                    (
                        Channel::In1,
                        settings.fls.channel[1].demodulate.attenuation,
                    ),
                    (
                        Channel::Out1,
                        settings.fls.channel[1].modulate.attenuation,
                    ),
                ] {
                    if p.set_attenuation(ch, att).is_err() {
                        log::warn!("invalid attenuation");
                    }
                }
                p.set_ext_clk(settings.fls.ext_clk).unwrap();
            });
            c.shared.dds_output.lock(|dds_output| {
                let mut builder = dds_output.builder();
                builder.push(
                    Channel::In0.into(),
                    Some(settings.fls.channel[0].demodulate.frequency),
                    None,
                    Some(Acr::new_with_raw_value(
                        settings.fls.channel[0].demodulate_acr,
                    )),
                );
                builder.push(
                    Channel::In1.into(),
                    Some(settings.fls.channel[1].demodulate.frequency),
                    None,
                    Some(Acr::new_with_raw_value(
                        settings.fls.channel[1].demodulate_acr,
                    )),
                );
                dds_output.write(builder);
            });
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.fls.stream));

            if settings.fls.activate {
                let cfg = settings.fls.build();
                c.shared.config.lock(|config| *config = cfg);
            }
        });
    }

    #[task(priority = 1, shared=[pounder, config, state])]
    async fn aux_adc(mut c: aux_adc::Context) -> ! {
        loop {
            let aux_adc::SharedResources {
                config,
                state,
                pounder,
                ..
            } = &mut c.shared;
            let x = pounder.lock(|p| {
                [
                    p.sample_aux_adc_raw(Channel::In0).unwrap(),
                    p.sample_aux_adc_raw(Channel::In1).unwrap(),
                ]
            });
            (config, state).lock(|c, s| {
                c.channel[0].dsp.power(&mut s.channel[0].dsp, x[0]);
                c.channel[1].dsp.power(&mut s.channel[1].dsp, x[1]);
            });
            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, local=[cpu_temp_sensor], shared=[network, settings, state, pounder])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            let (pll_time, channel) = c.shared.state.lock(|s| {
                (
                    s.pll_time.y,
                    s.channel.each_mut().map(|s| {
                        (
                            core::mem::take(&mut s.phase),
                            core::mem::take(&mut s.power),
                            ChannelTelemetry {
                                phase_raw: s.dsp.unwrap.y,
                                aux_adc: s.dsp.amplitude.x0() as f32
                                    * fls::AMPLITUDE_UNITS.x,
                                mod_amp: s.dsp.amplitude.y0() as f32
                                    * fls::AMPLITUDE_UNITS.y,
                                holds: s.dsp.holds,
                                slips: s.dsp.slips,
                                blanks: s.dsp.blanks,
                                ..Default::default()
                            },
                        )
                    }),
                )
            });
            let mut channel = channel.map(|(phase, power, mut channel)| {
                channel.phase = phase.get_scaled(fls::PHASE_UNITS.x);
                channel.power = power.get_scaled(P32::<28>::DELTA);
                channel
            });
            let cpu_temp = c.local.cpu_temp_sensor.get_temperature().unwrap();
            let pounder_temp = c.shared.pounder.lock(|p| {
                channel[0].rf_power = p.measure_power(Channel::In0).unwrap();
                channel[1].rf_power = p.measure_power(Channel::In1).unwrap();
                p.temperature().unwrap()
            });

            c.shared.network.lock(|net| {
                net.telemetry.publish_telemetry(
                    "/telemetry",
                    &CookedTelemetry {
                        pll_time,
                        channel,
                        pounder_temp,
                        cpu_temp,
                    },
                )
            });

            let telemetry_period =
                c.shared.settings.lock(|s| s.fls.telemetry_period);
            Systick::delay(((telemetry_period * 1e3) as u32).millis()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) -> ! {
        loop {
            if c.shared.usb.lock(|usb| {
                usb.poll(&mut [c
                    .local
                    .usb_terminal
                    .interface_mut()
                    .inner_mut()])
            }) && c.shared.settings.lock(|settings| {
                c.local.usb_terminal.poll(settings).unwrap_or_else(|_| {
                    log::warn!("USB error");
                    false
                })
            }) {
                settings_update::spawn().unwrap();
            }
            Systick::delay(10.millis()).await;
        }
    }

    #[task(priority = 1, shared=[network])]
    async fn ethernet_link(mut c: ethernet_link::Context) -> ! {
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
