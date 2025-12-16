#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::sync::atomic::{Ordering, fence};

use fugit::ExtU32;
use idsp::{
    iir::{SosClamp, SosState, Wdf, WdfState},
    process::{Inplace, Pair, Parallel, Process, Split},
};
use miniconf::Tree;
use rtic_monotonics::Monotonic;

use serde::Serialize;
use stabilizer::{convert::Gain, statistics};

use platform::{AppSettings, NetSettings};

const BATCH_SIZE_LOG2: u32 = 3;
const BATCH_SIZE: usize = 1 << BATCH_SIZE_LOG2;

// 100 MHz timer, 128 divider: period of 1.28 µs, ~781.25 KHz.
const SAMPLE_TICKS_LOG2: u32 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    mpll: App,
    net: NetSettings,
}

impl AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            mpll: App::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            mpll: App::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct MpllState {
    /// Lowpass state
    lp: [(((), ([WdfState<2>; 2], (WdfState<2>, WdfState<1>))), ()); 4],
    /// Phase clamp
    ///
    /// Makes the phase wrap monotonic by clamping it.
    /// Aids capture/pull-in with external modulation.
    ///
    /// TODO: Remove this for modulation drive.
    clamp: idsp::Clamp<i32>,
    /// PID state
    iir: SosState,
    /// Current output phase
    phase: i32,
    /// Current LO samples for downconverting the next batch
    lo: [[i32; BATCH_SIZE]; 2],
    // TODO: investigate matched LO delay: 20 samples
}

#[derive(Debug, Clone, Tree)]
#[tree(meta(doc, typename))]
pub struct Mpll {
    /// Lowpass
    #[tree(skip)]
    lp: Pair<[Wdf<2, 0xad>; 2], (Wdf<2, 0xad>, Wdf<1, 0xa>), i32>,
    /// Input phase offset
    phase: i32,
    /// PID IIR filter
    ///
    /// Do not use the iir offset as phase offset.
    /// Includes frequency limits.
    iir: SosClamp<30>,
    /// Output amplitude scale
    amp: [i32; 2],
}

impl Mpll {
    fn new(config: &Self) -> Self {
        config.clone()
    }
}

impl Default for Mpll {
    fn default() -> Self {
        // 7th order Cheby2 as WDF-CA, -3 dB @ 0.005*1.28, -112 dB @ 0.018*1.28
        let lp = Pair::new((
            (
                Default::default(),
                Parallel((
                    [
                        Wdf::quantize(&[
                            -0.9866183703960676,
                            0.9995042973541263,
                        ])
                        .unwrap(),
                        Wdf::quantize(&[-0.94357710177202, 0.9994723555364557])
                            .unwrap(),
                    ],
                    (
                        Wdf::quantize(&[
                            -0.9619459355859967,
                            0.9994905727027024,
                        ])
                        .unwrap(),
                        Wdf::quantize(&[0.9677764552414969]).unwrap(),
                    ),
                )),
            ),
            Default::default(),
        ));

        let mut pid = idsp::iir::PidBuilder::default();
        pid.order(idsp::iir::Order::I);
        pid.period(1.0 / BATCH_SIZE as f32);
        pid.gain(idsp::iir::Action::P, -5e-3); // fs/turn
        pid.gain(idsp::iir::Action::I, -4e-4); // fs/turn/ts = 1/turn
        pid.gain(idsp::iir::Action::D, -4e-3); // fs/turn*ts = turn
        //pid.limit(idsp::iir::Action::D, -0.2);
        let mut iir: SosClamp<_> = pid.build().into();
        iir.max = (0.3 * (1u64 << 32) as f32) as _;
        iir.min = (0.005 * (1u64 << 32) as f32) as _;

        Self {
            lp,
            phase: (0.0 * (1u64 << 32) as f32) as _,
            iir,
            amp: [(0.09 * (1u64 << 31) as f32) as _; 2], // ~0.9 V
        }
    }
}

impl Mpll {
    /// Ingest and process a batch of input samples and output new modulation
    pub fn process(
        &self,
        state: &mut MpllState,
        x: &[&[u16; BATCH_SIZE]; 2],
        y: &mut [&mut [u16; BATCH_SIZE]; 2],
    ) -> (Stream, i32) {
        // scratch
        let mut mix = [[0; BATCH_SIZE]; 4];
        let [m00, m01, m10, m11] = mix.each_mut();
        // mix
        for (x, (mix, lo)) in x[0].iter().zip(x[1].iter()).zip(
            m00.iter_mut()
                .zip(m01.iter_mut())
                .zip(m10.iter_mut().zip(m11.iter_mut()))
                .zip(state.lo[0].iter().zip(&state.lo[1])),
        ) {
            // ADC encoding, mix
            *mix.0.0 = ((*x.0 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.0.1 = ((*x.0 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
            *mix.1.0 = ((*x.1 as i16 as i64 * *lo.0 as i64) >> 16) as i32;
            *mix.1.1 = ((*x.1 as i16 as i64 * *lo.1 as i64) >> 16) as i32;
        }
        // lowpass
        for (mix, state) in mix.iter_mut().zip(state.lp.iter_mut()) {
            Split::new(&self.lp, state).inplace(mix);
        }
        // decimate
        let demod = [
            mix[0][BATCH_SIZE - 1],
            mix[1][BATCH_SIZE - 1],
            mix[2][BATCH_SIZE - 1],
            mix[3][BATCH_SIZE - 1],
        ];
        // phase
        // need full atan2 or addtl osc to support any phase offset
        let p = idsp::atan2(demod[1], demod[0]);
        // Delay correction
        let p = p
            .wrapping_add(self.phase)
            .wrapping_sub(state.iir.xy[2].wrapping_mul(10));
        let p = state.clamp.process(p);
        // pid
        let f = Split::new(&self.iir, &mut state.iir).process(p);
        // modulate
        let [y0, y1] = state.lo.each_mut();
        let [yo0, yo1] = y;
        state.phase = y0
            .iter_mut()
            .zip(y1)
            .zip((*yo0).iter_mut().zip((*yo1).iter_mut()))
            .fold(state.phase, |mut p, (y, yo)| {
                p = p.wrapping_add(f);
                (*y.0, *y.1) = idsp::cossin(p);
                *yo.0 = (((*y.0 as i64 * self.amp[0] as i64) >> (31 + 31 - 15))
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                *yo.1 = (((*y.1 as i64 * self.amp[1] as i64) >> (31 + 31 - 15))
                    as i16)
                    .wrapping_add(i16::MIN) as u16; // DAC encoding
                p
            });
        (
            Stream {
                demod,
                phase_in: p,
                phase_out: state.phase,
            },
            f,
        )
    }
}

/// Stream data format.
#[derive(Clone, Copy, Debug, Default, bytemuck::Zeroable, bytemuck::Pod)]
#[repr(C)]
pub struct Stream {
    demod: [i32; 4],
    phase_in: i32,
    phase_out: i32,
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct App {
    mpll: Mpll,

    /// Specifies the telemetry output period in seconds.
    telemetry_period: f32,

    /// Specifies the target for data streaming.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,
}

impl Default for App {
    fn default() -> Self {
        Self {
            telemetry_period: 10.,
            stream: Default::default(),
            mpll: Default::default(),
        }
    }
}

/// Channel Telemetry
#[derive(Default, Clone)]
pub struct TelemetryState {
    demod: [[statistics::State; 2]; 2],
    phase: statistics::State,
    frequency: statistics::State,
}

#[derive(Default, Clone, Serialize)]
pub struct TelemetryCooked {
    demod: [[statistics::ScaledStatistics; 2]; 2],
    phase: statistics::ScaledStatistics,
    frequency: statistics::ScaledStatistics,
}

impl From<TelemetryState> for TelemetryCooked {
    fn from(t: TelemetryState) -> Self {
        Self {
            // G10, 1 V fs
            demod: t
                .demod
                .map(|d| d.map(|p| p.get_scaled(1.0 / (1u64 << 31) as f32))),
            phase: t.phase.get_scaled(1.0 / (1u64 << 32) as f32),
            frequency: t
                .frequency
                .get_scaled(100e6 / SAMPLE_TICKS as f32 / (1u64 << 32) as f32),
        }
    }
}

#[derive(Default, Clone, Serialize)]
pub struct Telemetry {
    mpll: TelemetryCooked,
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
        .insert("title".to_string(), "Stabilizer mpll".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, SDMMC])]
mod app {
    use super::*;
    use stabilizer::hardware::{
        self, SerialTerminal, SystemTimer, Systick, UsbDevice,
        adc::{Adc0Input, Adc1Input},
        dac::{Dac0Output, Dac1Output},
        hal,
        net::{NetworkState, NetworkUsers},
        timers::SamplingTimer,
    };
    use stream::FrameGenerator;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<App>,
        settings: Settings,
        active_settings: Mpll,
        telemetry: TelemetryState,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings>,
        sampling_timer: SamplingTimer,
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        state: MpllState,
        generator: FrameGenerator,
        cpu_temp_sensor: stabilizer::hardware::cpu_temp_sensor::CpuTempSensor,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (mut carrier, _mezzanine, _eem) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        carrier.afes[0].set_gain(Gain::G10);
        carrier.afes[1].set_gain(Gain::G10);

        let mut network = NetworkUsers::new(
            carrier.network_devices.stack,
            carrier.network_devices.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &carrier.settings.net,
            carrier.metadata,
        );

        let generator = network.configure_streaming(stream::Format::Mpll);

        let shared = Shared {
            network,
            usb: carrier.usb,
            telemetry: Default::default(),
            active_settings: carrier.settings.mpll.mpll.clone(),
            settings: carrier.settings,
        };

        let mut local = Local {
            usb_terminal: carrier.usb_serial,
            sampling_timer: carrier.sampling_timer,
            adcs: carrier.adcs,
            dacs: carrier.dacs,
            cpu_temp_sensor: carrier.temperature_sensor,
            state: Default::default(),
            generator,
        };

        // Enable ADC/DAC events
        local.adcs.0.start();
        local.adcs.1.start();
        local.dacs.0.start();
        local.dacs.1.start();

        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        start::spawn().unwrap();
        usb::spawn().unwrap();

        (shared, local)
    }

    #[task(priority = 1, local=[sampling_timer])]
    async fn start(c: start::Context) {
        Systick::delay(100.millis()).await;
        // Start sampling ADCs and DACs.
        c.local.sampling_timer.start();
    }

    #[task(binds=DMA1_STR4, shared=[active_settings, telemetry], local=[adcs, dacs, state, generator], priority=3)]
    #[unsafe(link_section = ".itcm.process")]
    fn process(c: process::Context) {
        let process::SharedResources {
            active_settings,
            telemetry,
            ..
        } = c.shared;

        let process::LocalResources {
            adcs: (adc0, adc1),
            dacs: (dac0, dac1),
            state,
            generator,
            ..
        } = c.local;

        (active_settings, telemetry).lock(|settings, telemetry| {
            (adc0, adc1, dac0, dac1).lock(|adc0, adc1, dac0, dac1| {
                // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);
                let adc: [&[u16; BATCH_SIZE]; 2] = [
                    (**adc0).try_into().unwrap(),
                    (**adc1).try_into().unwrap(),
                ];
                let mut dac: [&mut [u16; BATCH_SIZE]; 2] =
                    [(*dac0).try_into().unwrap(), (*dac1).try_into().unwrap()];

                let (stream, frequency) =
                    settings.process(state, &adc, &mut dac);
                telemetry.phase.update(stream.phase_in);
                telemetry.frequency.update(frequency);

                telemetry.demod[0][0].update(stream.demod[0]);
                telemetry.demod[0][1].update(stream.demod[1]);
                telemetry.demod[1][0].update(stream.demod[2]);
                telemetry.demod[1][1].update(stream.demod[3]);

                const N: usize = core::mem::size_of::<Stream>();
                generator.add(|buf| {
                    buf[..N].copy_from_slice(bytemuck::cast_slice(
                        bytemuck::bytes_of(&stream),
                    ));
                    N
                });

                // Preserve instruction and data ordering w.r.t. DMA flag access.
                fence(Ordering::SeqCst);
            });
        });
    }

    #[idle(shared=[settings, network, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.mpll))
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

    #[task(priority = 1, shared=[network, settings, active_settings])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.shared
                .network
                .lock(|net| net.direct_stream(settings.mpll.stream));
            let new = Mpll::new(&settings.mpll.mpll);
            c.shared.active_settings.lock(|current| *current = new);
        });
    }

    #[task(priority = 1, local=[cpu_temp_sensor], shared=[network, settings, telemetry])]
    async fn telemetry(mut c: telemetry::Context) -> ! {
        loop {
            let tele = Telemetry {
                mpll: c.shared.telemetry.lock(|t| core::mem::take(t)).into(),
                cpu_temp: c
                    .local
                    .cpu_temp_sensor
                    .get_temperature()
                    .unwrap_or_default(),
            };
            c.shared.network.lock(|net| {
                net.telemetry.publish_telemetry("/telemetry", &tele)
            });

            let delay = c.shared.settings.lock(|s| s.mpll.telemetry_period);
            Systick::delay(((delay * 1000.0) as u32).millis()).await;
        }
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) -> ! {
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
