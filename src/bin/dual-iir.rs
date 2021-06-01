#![deny(warnings)]
#![no_std]
#![no_main]

use core::sync::atomic::{fence, Ordering};

use stabilizer::{hardware, net};

use miniconf::Miniconf;
use serde::Deserialize;

use dsp::iir;
use hardware::{
    Adc0Input, Adc1Input, AdcCode, AfeGain, Dac0Output, Dac1Output, DacCode,
    DigitalInput0, DigitalInput1, InputPin, SystemTimer, AFE0, AFE1,
};

use net::{NetworkState, NetworkUsers, Telemetry, TelemetryBuffer};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

#[derive(Clone, Copy, Debug, Deserialize, Miniconf)]
pub struct Settings {
    afe: [AfeGain; 2],
    iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
    allow_hold: bool,
    force_hold: bool,
    telemetry_period: u16,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            // Analog frontend programmable gain amplifier gains (G1, G2, G5, G10)
            afe: [AfeGain::G1, AfeGain::G1],
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
            telemetry_period: 10,
        }
    }
}

macro_rules! flatten_closures {
    ($fn:ident, $e:ident, $fun:block) => {
        $e.$fn(|$e| $fun )
    };
    ($fn:ident, $e:ident, $($es:ident),+, $fun:block) => {
        $e.$fn(|$e| flatten_closures!($fn, $($es),*, $fun))
    };
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = stabilizer::hardware::SystemTimer)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        digital_inputs: (DigitalInput0, DigitalInput1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        network: NetworkUsers<Settings, Telemetry>,

        settings: Settings,
        telemetry: TelemetryBuffer,

        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
    }

    #[init(spawn=[telemetry, settings_update])]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let network = NetworkUsers::new(
            stabilizer.net.stack,
            stabilizer.net.phy,
            stabilizer.cycle_counter,
            env!("CARGO_BIN_NAME"),
            stabilizer.net.mac_address,
        );

        // Spawn a settings update for default settings.
        c.spawn.settings_update().unwrap();
        c.spawn.telemetry().unwrap();

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            network,
            digital_inputs: stabilizer.digital_inputs,
            telemetry: net::TelemetryBuffer::default(),
            settings: Settings::default(),
        }
    }

    /// Main DSP processing routine for Stabilizer.
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
    #[task(binds=DMA1_STR4, resources=[adcs, digital_inputs, dacs, iir_state, settings, telemetry], priority=2)]
    #[inline(never)]
    #[link_section = ".itcm.process"]
    fn process(mut c: process::Context) {
        let process::Resources {
            adcs: (ref mut adc0, ref mut adc1),
            dacs: (ref mut dac0, ref mut dac1),
            ref digital_inputs,
            ref settings,
            ref mut iir_state,
            ref mut telemetry,
        } = c.resources;

        let digital_inputs = [
            digital_inputs.0.is_high().unwrap(),
            digital_inputs.1.is_high().unwrap(),
        ];
        telemetry.digital_inputs = digital_inputs;

        let hold =
            settings.force_hold || (digital_inputs[1] && settings.allow_hold);

        flatten_closures!(with_buffer, adc0, adc1, dac0, dac1, {
            let adc_samples = [adc0, adc1];
            let dac_samples = [dac0, dac1];

            // Preserve instruction and data ordering w.r.t. DMA flag access.
            fence(Ordering::SeqCst);

            for channel in 0..adc_samples.len() {
                adc_samples[channel]
                    .iter()
                    .zip(dac_samples[channel].iter_mut())
                    .map(|(ai, di)| {
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
                        // Convert to DAC code
                        *di = DacCode::from(y).0;
                    })
                    .last();
            }

            // Update telemetry measurements.
            telemetry.adcs =
                [AdcCode(adc_samples[0][0]), AdcCode(adc_samples[1][0])];

            telemetry.dacs =
                [DacCode(dac_samples[0][0]), DacCode(dac_samples[1][0])];

            // Preserve instruction and data ordering w.r.t. DMA flag access.
            fence(Ordering::SeqCst);
        });
    }

    #[idle(resources=[network], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            match c.resources.network.lock(|net| net.update()) {
                NetworkState::SettingsChanged => {
                    c.spawn.settings_update().unwrap()
                }
                NetworkState::Updated => {}
                NetworkState::NoChange => cortex_m::asm::wfi(),
            }
        }
    }

    #[task(priority = 1, resources=[network, afes, settings])]
    fn settings_update(mut c: settings_update::Context) {
        // Update the IIR channels.
        let settings = c.resources.network.miniconf.settings();
        c.resources.settings.lock(|current| *current = *settings);

        // Update AFEs
        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);
    }

    #[task(priority = 1, resources=[network, settings, telemetry], schedule=[telemetry])]
    fn telemetry(mut c: telemetry::Context) {
        let telemetry: TelemetryBuffer =
            c.resources.telemetry.lock(|telemetry| *telemetry);

        let (gains, telemetry_period) = c
            .resources
            .settings
            .lock(|settings| (settings.afe, settings.telemetry_period));

        c.resources
            .network
            .telemetry
            .publish(&telemetry.finalize(gains[0], gains[1]));

        // Schedule the telemetry task in the future.
        c.schedule
            .telemetry(
                c.scheduled
                    + SystemTimer::ticks_from_secs(telemetry_period as u32),
            )
            .unwrap();
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { stm32h7xx_hal::ethernet::interrupt_handler() }
    }

    #[task(binds = SPI2, priority = 3)]
    fn spi2(_: spi2::Context) {
        panic!("ADC0 input overrun");
    }

    #[task(binds = SPI3, priority = 3)]
    fn spi3(_: spi3::Context) {
        panic!("ADC1 input overrun");
    }

    #[task(binds = SPI4, priority = 3)]
    fn spi4(_: spi4::Context) {
        panic!("DAC0 output error");
    }

    #[task(binds = SPI5, priority = 3)]
    fn spi5(_: spi5::Context) {
        panic!("DAC1 output error");
    }

    extern "C" {
        // hw interrupt handlers for RTIC to use for scheduling tasks
        // one per priority
        fn DCMI();
        fn JPEG();
        fn SDMMC();
    }
};
