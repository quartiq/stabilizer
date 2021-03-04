#![deny(warnings)]
#![no_std]
#![no_main]

use stm32h7xx_hal as hal;

use stabilizer::hardware;

use miniconf::{minimq, Miniconf, MqttInterface};
use serde::Deserialize;

use dsp::iir;
use hardware::{
    Adc0Input, Adc1Input, AfeGain, CycleCounter, Dac0Output, Dac1Output,
    NetworkStack, AFE0, AFE1,
};

const SCALE: f32 = i16::MAX as _;

// The number of cascaded IIR biquads per channel. Select 1 or 2!
const IIR_CASCADE_LENGTH: usize = 1;

#[derive(Debug, Deserialize, Miniconf)]
pub struct Settings {
    afe: [AfeGain; 2],
    iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            afe: [AfeGain::G1, AfeGain::G1],
            iir_ch: [[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2],
        }
    }
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        afes: (AFE0, AFE1),
        adcs: (Adc0Input, Adc1Input),
        dacs: (Dac0Output, Dac1Output),
        mqtt_interface:
            MqttInterface<Settings, NetworkStack, minimq::consts::U256>,
        clock: CycleCounter,

        // Format: iir_state[ch][cascade-no][coeff]
        #[init([[[0.; 5]; IIR_CASCADE_LENGTH]; 2])]
        iir_state: [[iir::Vec5; IIR_CASCADE_LENGTH]; 2],
        #[init([[iir::IIR::new(1., -SCALE, SCALE); IIR_CASCADE_LENGTH]; 2])]
        iir_ch: [[iir::IIR; IIR_CASCADE_LENGTH]; 2],
    }

    #[init]
    fn init(c: init::Context) -> init::LateResources {
        // Configure the microcontroller
        let (mut stabilizer, _pounder) = hardware::setup(c.core, c.device);

        let mqtt_interface = {
            let mqtt_client = {
                minimq::MqttClient::new(
                    hardware::design_parameters::MQTT_BROKER.into(),
                    "",
                    stabilizer.net.stack,
                )
                .unwrap()
            };

            MqttInterface::new(
                mqtt_client,
                "dt/sinara/stabilizer",
                Settings::default(),
            )
            .unwrap()
        };

        // Enable ADC/DAC events
        stabilizer.adcs.0.start();
        stabilizer.adcs.1.start();
        stabilizer.dacs.0.start();
        stabilizer.dacs.1.start();

        // Start sampling ADCs.
        stabilizer.adc_dac_timer.start();

        init::LateResources {
            mqtt_interface,
            afes: stabilizer.afes,
            adcs: stabilizer.adcs,
            dacs: stabilizer.dacs,
            clock: stabilizer.cycle_counter,
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
    #[task(binds=DMA1_STR4, resources=[adcs, dacs, iir_state, iir_ch], priority=2)]
    fn process(c: process::Context) {
        let adc_samples = [
            c.resources.adcs.0.acquire_buffer(),
            c.resources.adcs.1.acquire_buffer(),
        ];

        let dac_samples = [
            c.resources.dacs.0.acquire_buffer(),
            c.resources.dacs.1.acquire_buffer(),
        ];

        for channel in 0..adc_samples.len() {
            for sample in 0..adc_samples[0].len() {
                let x = f32::from(adc_samples[channel][sample] as i16);
                let mut y = x;
                for i in 0..c.resources.iir_state[channel].len() {
                    y = c.resources.iir_ch[channel][i]
                        .update(&mut c.resources.iir_state[channel][i], y);
                }
                // Note(unsafe): The filter limits ensure that the value is in range.
                // The truncation introduces 1/2 LSB distortion.
                let y = unsafe { y.to_int_unchecked::<i16>() };
                // Convert to DAC code
                dac_samples[channel][sample] = y as u16 ^ 0x8000;
            }
        }
    }

    #[idle(resources=[mqtt_interface, clock], spawn=[settings_update])]
    fn idle(mut c: idle::Context) -> ! {
        let clock = c.resources.clock;

        loop {
            let sleep = c.resources.mqtt_interface.lock(|interface| {
                !interface.network_stack().poll(clock.current_ms())
            });

            match c
                .resources
                .mqtt_interface
                .lock(|interface| interface.update())
            {
                Ok(update) => {
                    if update {
                        c.spawn.settings_update().unwrap();
                    } else if sleep {
                        cortex_m::asm::wfi();
                    }
                }
                Err(miniconf::MqttError::Network(
                    smoltcp_nal::NetworkError::NoIpAddress,
                )) => {}
                Err(error) => log::info!("Unexpected error: {:?}", error),
            }
        }
    }

    #[task(priority = 1, resources=[mqtt_interface, afes, iir_ch])]
    fn settings_update(mut c: settings_update::Context) {
        let settings = &c.resources.mqtt_interface.settings;

        // Update the IIR channels.
        c.resources.iir_ch.lock(|iir| *iir = settings.iir_ch);

        // Update AFEs
        c.resources.afes.0.set_gain(settings.afe[0]);
        c.resources.afes.1.set_gain(settings.afe[1]);
    }

    #[task(binds = ETH, priority = 1)]
    fn eth(_: eth::Context) {
        unsafe { hal::ethernet::interrupt_handler() }
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
