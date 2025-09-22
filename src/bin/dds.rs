//! Urukul as a downstream EEM on stabilizer.
//!
//! This requires the alternate direction EEM transceiver configuration.
//! It exposes the Urukul CPLD and DDS settings via miniconf (MQTT and USB).
//!
//! Note that several values are not range checked and out-of-range values
//! will lead to panics.
#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

use arbitrary_int::{u2, u5};
use fugit::ExtU32;
use miniconf::Tree;
use rtic_monotonics::Monotonic;

use platform::{AppSettings, NetSettings};

#[derive(Clone, Debug, Tree, Default)]
#[tree(meta(doc, typename))]
pub struct Settings {
    urukul: App,
    net: NetSettings,
}

impl AppSettings for Settings {
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

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct Channel {
    #[tree(with=miniconf::leaf)]
    pll_n: Option<u5>,
    pll_doubler: bool,
    frequency: f64,
    phase: f32,
    full_scale_current: f32,
    attenuation: f32,
    enable: bool,
    update: bool,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            frequency: 0.0,
            phase: 0.0,
            full_scale_current: 20e-3,
            attenuation: 31.5,
            enable: false,
            pll_n: Some(u5::new(3)),
            pll_doubler: false,
            update: true,
        }
    }
}

#[derive(Clone, Debug, Tree)]
#[tree(meta(doc, typename))]
pub struct App {
    refclk: f64,
    #[tree(with=miniconf::leaf)]
    clk_sel: urukul::ClkSel,
    #[tree(with=miniconf::leaf)]
    div_sel: urukul::DivSel,
    update: bool,
    ch: [Channel; 4],
}

impl Default for App {
    fn default() -> Self {
        Self {
            clk_sel: urukul::ClkSel::Osc,
            div_sel: urukul::DivSel::One,
            update: true,
            refclk: 100.0e6,
            ch: Default::default(),
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
        .insert("title".to_string(), "Stabilizer dds".into());
    println!("{}", serde_json::to_string_pretty(&schema.root).unwrap());
}

#[cfg(target_os = "none")]
#[rtic::app(device = stabilizer::hardware::hal::stm32, peripherals = true, dispatchers=[DCMI, JPEG, LTDC, SDMMC])]
mod app {
    use super::*;

    use stabilizer::hardware::{
        self, hal,
        net::{NetworkState, NetworkUsers},
        SerialTerminal, SystemTimer, Systick, Urukul, UsbDevice,
    };

    #[shared]
    struct Shared {
        usb: UsbDevice,
        network: NetworkUsers<App>,
        settings: Settings,
    }

    #[local]
    struct Local {
        urukul: Urukul,
        usb_terminal: SerialTerminal<Settings>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        let (stabilizer, _pounder) = hardware::setup::setup::<Settings>(
            c.core,
            c.device,
            clock,
            8,
            1 << 7,
        );

        let stabilizer::hardware::Eem::Urukul(urukul) = stabilizer.eem else {
            panic!("No Urukul detected.")
        };

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
            settings: stabilizer.settings,
        };

        let local = Local {
            urukul,
            usb_terminal: stabilizer.usb_serial,
        };

        // Spawn a settings update for default settings.
        settings_update::spawn().unwrap();
        ethernet_link::spawn().unwrap();
        usb::spawn().unwrap();

        (shared, local)
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

    #[task(priority = 1, shared=[settings], local=[urukul])]
    async fn settings_update(mut c: settings_update::Context) {
        let u = c.local.urukul;
        c.shared.settings.lock(|s| {
            let s = &mut s.urukul;
            if s.update {
                s.update = false;
                u.set_cfg(
                    u.cfg().with_clk_sel(s.clk_sel).with_div_sel(s.div_sel),
                )
                .unwrap();
            }
            let power = ad9912::Power::builder()
                .with_digital_pd(false)
                .with_full_pd(false)
                .with_pll_pd(true)
                .with_output_doubler_en(false)
                .with_cmos_en(false)
                .with_hstl_pd(true)
                .build();
            for (i, ch) in s.ch.iter_mut().enumerate() {
                if ch.update {
                    ch.update = false;
                    let refclk = s.refclk / s.div_sel.divider() as f64;
                    let i = u2::new(i as _);
                    let sysclk = if let Some(pll_n) = ch.pll_n {
                        u.dds(i).set_power(power.with_pll_pd(false)).unwrap();
                        u.dds(i).set_ndiv(pll_n).unwrap();
                        let mut pll = ad9912::Pll::default()
                            .with_charge_pump(ad9912::ChargePump::Ua375)
                            .with_ref_doubler(ch.pll_doubler);
                        let sysclk = pll.set_refclk(pll_n, refclk);
                        u.dds(i).set_pll(pll).unwrap();
                        sysclk
                    } else {
                        u.dds(i).set_power(power.with_pll_pd(true)).unwrap();
                        refclk
                    };
                    u.dds(i).set_frequency(ch.frequency, sysclk).unwrap();
                    u.dds(i).set_phase(ch.phase).unwrap();
                    u.io_update().unwrap();
                    u.dds(i)
                        .set_full_scale_current(ch.full_scale_current, 10e3)
                        .unwrap();
                    u.set_att(i, urukul::att_to_mu(ch.attenuation)).unwrap();
                    u.set_rf_sw(i, ch.enable).unwrap();
                }
            }
        });
    }

    #[task(priority = 1, shared=[usb, settings], local=[usb_terminal])]
    async fn usb(mut c: usb::Context) {
        loop {
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
