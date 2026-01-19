#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use core::sync::atomic::{Ordering, fence};

use fugit::ExtU32;
use miniconf::Tree;
use rtic_monotonics::Monotonic;

use serde::Serialize;
use stabilizer::{convert::Gain, mpll::*, statistics};

use platform::{AppSettings, NetSettings};

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

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct App {
    mpll: MpllConfig,

    /// Specifies the telemetry output period in seconds.
    telemetry_period: f32,

    /// Specifies the target for data streaming.
    #[tree(with=miniconf::leaf)]
    stream: stream::Target,

    /// Activate settings
    ///
    /// If `activate = true` each `mpll` settings change immediately results
    /// in activation.
    activate: bool,
}

impl Default for App {
    fn default() -> Self {
        Self {
            telemetry_period: 10.,
            stream: Default::default(),
            mpll: Default::default(),
            activate: true,
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
        const VOLT_PER_LSB: f32 = 10.24 /* V FS */ / Gain::G10.gain() * 2.0 /* conversion */ / (1u64 << 31) as f32;
        Self {
            demod: t.demod.map(|d| d.map(|p| p.get_scaled(VOLT_PER_LSB))),
            phase: t.phase.get_scaled(UNITS.x),
            frequency: t.frequency.get_scaled(UNITS.y),
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
            active_settings: carrier.settings.mpll.mpll.build(),
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
                let dac: [&mut [u16; BATCH_SIZE]; 2] =
                    [(*dac0).try_into().unwrap(), (*dac1).try_into().unwrap()];

                let stream = settings.process(state, adc, dac);

                telemetry.phase.update(stream.phase.0);
                telemetry.frequency.update(stream.frequency.0);
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
            if settings.mpll.activate {
                let new = settings.mpll.mpll.build();
                c.shared.active_settings.lock(|current| *current = new);
            }
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
