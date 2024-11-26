#![no_std]
#![no_main]

use arbitrary_int::{u2, u5};
use fugit::ExtU32;
use miniconf::{Leaf, Tree};
use rtic_monotonics::Monotonic;
use serde::{Deserialize, Serialize};

use stabilizer::{
    hardware::{
        self, ad9912, hal, urukul, SerialTerminal, SystemTimer, Systick,
        Urukul, UsbDevice,
    },
    net::{telemetry::TelemetryBuffer, NetworkState, NetworkUsers},
    settings::NetSettings,
};

// The number of samples in each batch process
const BATCH_SIZE: usize = 8;

// The logarithm of the number of 100MHz timer ticks between each sample. With a value of 2^7 =
// 128, there is 1.28uS per sample, corresponding to a sampling frequency of 781.25 KHz.
const SAMPLE_TICKS_LOG2: u8 = 7;
const SAMPLE_TICKS: u32 = 1 << SAMPLE_TICKS_LOG2;

#[derive(Clone, Debug, Tree)]
pub struct Settings {
    pub urukul: App,
    pub net: NetSettings,
}

impl stabilizer::settings::AppSettings for Settings {
    fn new(net: NetSettings) -> Self {
        Self {
            net,
            urukul: App::default(),
        }
    }

    fn net(&self) -> &NetSettings {
        &self.net
    }
}

impl serial_settings::Settings for Settings {
    fn reset(&mut self) {
        *self = Self {
            urukul: App::default(),
            net: NetSettings::new(self.net.mac),
        }
    }
}

#[derive(Clone, Debug, Tree, Serialize, Deserialize)]
pub struct App {
    telemetry_period: Leaf<f32>,
}

impl Default for App {
    fn default() -> Self {
        Self {
            telemetry_period: 10.0.into(),
        }
    }
}

#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<App, 3>,
        settings: Settings,
        telemetry: TelemetryBuffer,
        app: App,
        urukul: Urukul,
    }

    #[local]
    struct Local {
        usb_terminal: SerialTerminal<Settings, 4>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        // Configure the microcontroller
        let (stabilizer, _pounder) = hardware::setup::setup::<Settings, 4>(
            c.core,
            c.device,
            clock,
            BATCH_SIZE,
            SAMPLE_TICKS,
        );

        let crate::hardware::Eem::Urukul(mut urukul) = stabilizer.eem else {
            panic!("No Urukul detected.")
        };

        let ch = u2::new(0);
        urukul.io_update().unwrap();
        urukul
            .set_cfg(
                urukul
                    .cfg()
                    .with_clk_sel(urukul::ClkSel::Osc)
                    .with_div_sel(urukul::DivSel::One),
            )
            .unwrap();
        let ndiv = u5::new(10 / 2 - 2);
        urukul.dds(ch).set_ndiv(ndiv).unwrap();
        urukul.io_update().unwrap();
        let pll = ad9912::Pll::builder()
            .with_charge_pump(hardware::ad9912::ChargePump::Ua375)
            .with_vco_range_high(true)
            .with_ref_doubler(false)
            .with_vco_auto_range(false)
            .build();
        urukul.dds(ch).set_pll(pll).unwrap();
        urukul.io_update().unwrap();
        let sysclk = ad9912::sysclk(ndiv, pll, 100e6);
        let ftw = urukul.dds(ch).set_frequency(80e6, sysclk).unwrap();
        urukul.io_update().unwrap();
        assert_eq!(ftw, urukul.dds(ch).ftw().unwrap());
        urukul.set_att(ch, 0xff).unwrap();
        urukul.set_rf_sw(ch, true).unwrap();

        let network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            clock,
            env!("CARGO_BIN_NAME"),
            &stabilizer.settings.net,
            stabilizer.metadata,
        );

        let shared = Shared {
            usb: stabilizer.usb,
            network,
            app: stabilizer.settings.urukul.clone(),
            telemetry: TelemetryBuffer::default(),
            settings: stabilizer.settings,
            urukul,
        };

        let local = Local {
            usb_terminal: stabilizer.usb_serial,
        };

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        telemetry::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();

        (shared, local)
    }

    #[task(binds=DMA1_STR4, shared=[app, telemetry], priority=3)]
    #[link_section = ".itcm.process"]
    fn process(_c: process::Context) {
        // let process::SharedResources { app, telemetry, .. } = c.shared;
        // let process::LocalResources { .. } = c.local;
    }

    #[idle(shared=[network, settings, usb])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match (&mut c.shared.network, &mut c.shared.settings)
                .lock(|net, settings| net.update(&mut settings.urukul))
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

    #[task(priority = 1, shared=[network, settings, app])]
    async fn settings_update(mut c: settings_update::Context) {
        c.shared.settings.lock(|settings| {
            c.shared
                .app
                .lock(|current| *current = settings.urukul.clone());
        });
    }

    #[task(priority = 1, shared=[network, settings, telemetry])]
    async fn telemetry(mut c: telemetry::Context) {
        loop {
            let _telemetry = c.shared.telemetry.lock(|telemetry| *telemetry);
            let telemetry_period =
                c.shared.settings.lock(|s| *s.urukul.telemetry_period);

            // c.shared.network.lock(|net| {
            //     net.telemetry.publish(&telemetry.finalize(
            //         *gains[0],
            //         *gains[1],
            //         c.local.cpu_temp_sensor.get_temperature().unwrap(),
            //     ))
            // });

            Systick::delay(((telemetry_period * 1000.0) as u32).millis()).await;
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
