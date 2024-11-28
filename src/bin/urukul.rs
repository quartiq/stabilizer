//! Urukul as a downstream EEM on stabilizer.
//!
//! This requires the alternate direction EEM transceiver configuration.
//! It exposes the Urukul CPLD and DDS settings via miniconf (MQTT and USB).
//!
//! Note that several values are not range checked and out-of-range values
//! will lead to panics.
#![no_std]
#![no_main]

use arbitrary_int::{u2, u5};
use fugit::ExtU32;
use miniconf::{Leaf, Tree};
use rtic_monotonics::Monotonic;

use stabilizer::{
    hardware::{
        self, hal, SerialTerminal, SystemTimer, Systick, Urukul, UsbDevice,
    },
    net::{NetworkState, NetworkUsers},
    settings::NetSettings,
};

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

#[derive(Clone, Debug, Tree)]
pub struct Channel {
    pll_n: Leaf<Option<u5>>,
    pll_doubler: Leaf<bool>,
    // TODO: range check
    frequency: Leaf<f64>,
    // TODO: range check
    phase: Leaf<f32>,
    // TODO: range check
    full_scale_current: Leaf<f32>,
    // TODO: range check
    attenuation: Leaf<f32>,
    enable: Leaf<bool>,
    update: Leaf<bool>,
}

impl Default for Channel {
    fn default() -> Self {
        Self {
            frequency: 0.0.into(),
            phase: 0.0.into(),
            full_scale_current: 20e-3.into(),
            attenuation: 31.5.into(),
            enable: false.into(),
            pll_n: Some(u5::new(3)).into(),
            pll_doubler: false.into(),
            update: true.into(),
        }
    }
}

#[derive(Clone, Debug, Tree)]
pub struct App {
    refclk: Leaf<f64>,
    clk_sel: Leaf<urukul::ClkSel>,
    div_sel: Leaf<urukul::DivSel>,
    update: Leaf<bool>,
    ch: [Channel; 4],
}

impl Default for App {
    fn default() -> Self {
        let ch = Channel::default();
        Self {
            clk_sel: urukul::ClkSel::Osc.into(),
            div_sel: urukul::DivSel::One.into(),
            update: true.into(),
            refclk: 100.0e6.into(),
            ch: [ch.clone(), ch.clone(), ch.clone(), ch.clone()],
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
    }

    #[local]
    struct Local {
        urukul: Urukul,
        usb_terminal: SerialTerminal<Settings, 4>,
    }

    #[init]
    fn init(c: init::Context) -> (Shared, Local) {
        let clock = SystemTimer::new(|| Systick::now().ticks());

        let (stabilizer, _pounder) = hardware::setup::setup::<Settings, 4>(
            c.core,
            c.device,
            clock,
            8,
            1 << 7,
        );

        let crate::hardware::Eem::Urukul(urukul) = stabilizer.eem else {
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
            if *s.update {
                *s.update = false;
                u.set_cfg(
                    u.cfg().with_clk_sel(*s.clk_sel).with_div_sel(*s.div_sel),
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
                if *ch.update {
                    *ch.update = false;
                    let refclk = *s.refclk / s.div_sel.divider() as f64;
                    let i = u2::new(i as _);
                    let sysclk = if let Some(pll_n) = *ch.pll_n {
                        u.dds(i).set_power(power.with_pll_pd(false)).unwrap();
                        u.dds(i).set_ndiv(pll_n).unwrap();
                        let mut pll = ad9912::Pll::default()
                            .with_charge_pump(ad9912::ChargePump::Ua375)
                            .with_ref_doubler(*ch.pll_doubler);
                        let sysclk = pll.set_refclk(pll_n, refclk);
                        u.dds(i).set_pll(pll).unwrap();
                        sysclk
                    } else {
                        u.dds(i).set_power(power.with_pll_pd(true)).unwrap();
                        refclk
                    };
                    u.dds(i).set_frequency(*ch.frequency, sysclk).unwrap();
                    u.dds(i).set_phase(*ch.phase).unwrap();
                    u.io_update().unwrap();
                    u.dds(i)
                        .set_full_scale_current(*ch.full_scale_current, 10e3)
                        .unwrap();
                    u.set_att(i, urukul::att_to_mu(*ch.attenuation)).unwrap();
                    u.set_rf_sw(i, *ch.enable).unwrap();
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
