use super::{Channel, ChannelVariant};
use crate::hardware::shared_adc::AdcChannel;

use super::super::hal::{
    gpio::{gpiof::*, Analog},
    stm32::{ADC1, ADC2, ADC3},
};

const V_REF: f32 = 2.048; // ADC reference voltage

// See schematic "diagnostics" sheet. Output voltage sense.
const V_OFFSET: f32 = 1.; // divider/shifter opamp offset
const V_SCALE: f32 = 3.; // divider/shifter opamp gain

// See schematic "output_stage" sheet. Output current sense.
const I_SCALE: f32 = 10000. / 2000.; // current sense scale

#[derive(Clone, Copy, Debug)]
pub enum DriverMonitorChannel {
    Voltage(Channel),
    Current(Channel),
}

pub struct InternalAdc {
    output_voltage: (
        AdcChannel<'static, ADC1, PF11<Analog>>,
        AdcChannel<'static, ADC3, PF3<Analog>>,
    ),
    output_current: (
        AdcChannel<'static, ADC2, PF13<Analog>>,
        AdcChannel<'static, ADC2, PF14<Analog>>,
    ),
    variant: [ChannelVariant; 2],
}

impl InternalAdc {
    pub fn new(
        output_voltage: (
            AdcChannel<'static, ADC1, PF11<Analog>>,
            AdcChannel<'static, ADC3, PF3<Analog>>,
        ),
        output_current: (
            AdcChannel<'static, ADC2, PF13<Analog>>,
            AdcChannel<'static, ADC2, PF14<Analog>>,
        ),
        ln_varinat: ChannelVariant,
        hp_varinat: ChannelVariant,
    ) -> Self {
        InternalAdc {
            output_voltage,
            output_current,
            variant: [ln_varinat, hp_varinat],
        }
    }

    pub fn read(&mut self, ch: DriverMonitorChannel) -> f32 {
        match ch {
            DriverMonitorChannel::Voltage(ch) => self.read_output_voltage(ch),
            DriverMonitorChannel::Current(ch) => self.read_output_current(ch),
        }
    }

    /// Reads the voltage of the channel output with respect to ground.
    pub fn read_output_voltage(&mut self, ch: Channel) -> f32 {
        let ratio: f32 = match ch {
            Channel::LowNoise => {
                self.output_voltage.0.read_normalized().unwrap()
            }
            Channel::HighPower => {
                self.output_voltage.1.read_normalized().unwrap()
            }
        };
        const SCALE: f32 = V_SCALE * V_REF;
        const OFFSET: f32 = V_OFFSET / V_REF; // voltage sense offset relative to V_REF
        (ratio - OFFSET) * SCALE
    }

    /// Reads the current of the channel output (negative for inflowing current).
    pub fn read_output_current(&mut self, ch: Channel) -> f32 {
        let ratio: f32 = match ch {
            Channel::LowNoise => {
                self.output_current.0.read_normalized().unwrap()
            }
            Channel::HighPower => {
                self.output_current.1.read_normalized().unwrap()
            }
        };
        let scale =
            V_REF * I_SCALE / self.variant[ch as usize].transimpedance(); // current sense scale
        ratio * scale
    }
}
