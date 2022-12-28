// This is a dummy driver for the Driver analog reads of the output Voltage and Current.
// Exact Sacales and Pinout will be filled in once we have HW.

use super::Channel;
use crate::hardware::shared_adc::AdcChannel;

use super::super::hal::{
    gpio::{gpiof::*, Analog},
    stm32::{ADC1, ADC2, ADC3},
};

const V_REF: f32 = 2.048; // ADC reference voltage
const R_SENSE: f32 = 0.1; // Driver output current sense resistor (Will maybe be something else on HW)

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
    ) -> Self {
        InternalAdc {
            output_voltage,
            output_current,
        }
    }

    pub fn read(&mut self, ch: DriverMonitorChannel) -> f32 {
        match ch {
            DriverMonitorChannel::Voltage(ch) => self.read_output_voltage(ch),
            DriverMonitorChannel::Current(ch) => self.read_output_current(ch),
        }
    }

    pub fn read_output_voltage(&mut self, ch: Channel) -> f32 {
        let ratio: f32 = match ch {
            Channel::LowNoise => {
                self.output_voltage.0.read_normalized().unwrap()
            }
            Channel::HighPower => {
                self.output_voltage.1.read_normalized().unwrap()
            }
        };
        const SCALE: f32 = V_REF; // Differential voltage sense gain      ToDo
        const OFFSET: f32 = 0.0; // Differential voltage sense offset       ToDo
        (ratio + OFFSET) * SCALE
    }

    pub fn read_output_current(&mut self, ch: Channel) -> f32 {
        let ratio: f32 = match ch {
            Channel::LowNoise => {
                self.output_current.0.read_normalized().unwrap()
            }
            Channel::HighPower => {
                self.output_current.1.read_normalized().unwrap()
            }
        };
        const SCALE: f32 = V_REF; // Current sense scale       ToDo
        const OFFSET: f32 = 0.0; // Current sense offset         ToDo
        (ratio + OFFSET) * SCALE
    }
}
