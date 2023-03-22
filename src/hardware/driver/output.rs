///! Driver output state handling.
///!
///! Driver has two independent output channels.
///! Each channel can be in various states during powerup and powerdown.
///! At powerup the device goes through the relay sequence and performs various selfchecks along the way.
///! See state machine transitions for details.
use core::{fmt::Debug, ops::Range};
use embedded_hal::blocking::i2c::{Write, WriteRead};
use idsp::iir;
use log::debug;
use mcp230xx::{Mcp23008, Mcp230xx};
use serde::{Deserialize, Serialize};
use smlang::statemachine;

use crate::hardware::I2c1Proxy;

use super::{relay::Relay, Channel};

/// Selftest struct that can be reported by telemetry.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SelfTest {
    reason: FailReason,
    valid_range: Range<f32>,
    read: f32,
    channel: Channel,
}

impl SelfTest {
    fn test(
        state: &sm::States,
        channel: &Channel,
        reads: &[f32; 2], // voltage/current reads
    ) -> Option<Self> {
        let tests = match (state, channel) {
            (sm::States::SelftestZero, Channel::LowNoise) => Some((
                [
                    Output::<I2c1Proxy>::LN_VALID_VOLTAGE_ZERO,
                    Output::<I2c1Proxy>::LN_VALID_CURRENT_ZERO,
                ],
                [FailReason::ZeroVoltage, FailReason::ZeroCurrent],
            )),
            (sm::States::SelftestShunt, Channel::LowNoise) => Some((
                [
                    Output::<I2c1Proxy>::LN_VALID_VOLTAGE_SHUNT,
                    Output::<I2c1Proxy>::LN_VALID_CURRENT_SHUNT,
                ],
                [FailReason::ShuntVoltage, FailReason::ShuntCurrent],
            )),
            (sm::States::SelftestShort, Channel::LowNoise) => Some((
                [
                    Output::<I2c1Proxy>::LN_VALID_VOLTAGE_SHOTTKY,
                    Output::<I2c1Proxy>::LN_VALID_CURRENT_SHOTTKY,
                ],
                [FailReason::ShortVoltage, FailReason::ShortCurrent],
            )),
            (sm::States::SelftestZero, Channel::HighPower) => Some((
                [
                    Output::<I2c1Proxy>::HP_VALID_VOLTAGE_ZERO,
                    Output::<I2c1Proxy>::HP_VALID_CURRENT_ZERO,
                ],
                [FailReason::ZeroVoltage, FailReason::ZeroCurrent],
            )),
            (sm::States::SelftestShunt, Channel::HighPower) => Some((
                [
                    Output::<I2c1Proxy>::HP_VALID_VOLTAGE_SHUNT,
                    Output::<I2c1Proxy>::HP_VALID_CURRENT_SHUNT,
                ],
                [FailReason::ShuntVoltage, FailReason::ShuntCurrent],
            )),
            (sm::States::SelftestShort, Channel::HighPower) => Some((
                [
                    Output::<I2c1Proxy>::HP_VALID_VOLTAGE_SHOTTKY,
                    Output::<I2c1Proxy>::HP_VALID_CURRENT_SHOTTKY,
                ],
                [FailReason::ShortVoltage, FailReason::ShortCurrent],
            )),
            _ => None,
        };

        if let Some(tests) = tests {
            for ((range, &read), &reason) in
                tests.0.iter().zip(reads.iter()).zip(tests.1.iter())
            {
                if !range.contains(&read) {
                    return Some(SelfTest {
                        reason,
                        valid_range: range.clone(),
                        read,
                        channel: *channel,
                    });
                }
            }
        }
        None
    }
}

/// Reason for why a selftest failed.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum FailReason {
    /// Current was set to zero and measured voltage was out of range. [Output::<I2C>::VALID_VOLTAGE_ZERO]
    ZeroVoltage,

    /// Current was set to zero and measured current was out of range. [Output::<I2C>::VALID_CURRENT_ZERO]
    ZeroCurrent,

    /// Driver output was connected to a 10 ohm shunt.
    /// Current was set to [Output::<I2C>::TESTCURRENT] and measured voltage was out of range.
    /// [Output::<I2C>::VALID_VOLTAGE_SHUNT]
    ShuntVoltage,

    /// Driver output was connected to a 10 ohm shunt.
    /// Current was set to [Output::<I2C>::TESTCURRENT] and measured current was out of range.
    /// [Output::<I2C>::VALID_CURRENT_SHUNT]
    ShuntCurrent,

    /// Driver output was shorted to ground.
    /// Current was set to [Output::<I2C>::TESTCURRENT] and measured voltage was out of range.
    /// [Output::<I2C>::VALID_VOLTAGE_SHORT]
    ShortVoltage,

    /// Driver output was shorted to ground.
    /// Current was set to [Output::<I2C>::TESTCURRENT] and measured current was out of range.
    /// [Output::<I2C>::VALID_CURRENT_SHORT]
    ShortCurrent,
}

/// Driver [Output].
/// An Output consits of the output [Relay]s and an IIR to implement a current ramp
/// during an enabling sequence and a current hold during powerdown.
pub struct Output<I2C: WriteRead + Write + 'static> {
    iir: iir::IIR<f32>,
    relay: Relay<I2C>,
}

impl<I2C, E> Output<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // 1 A/s current ramp-up
    const RAMP_STEP: f32 = 1e-3;
    const RAMP_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(1);

    // Delay after setting a current
    pub const SET_DELAY: fugit::MillisDuration<u64> =
        fugit::MillisDurationU64::millis(10);

    const TESTCURRENT: f32 = 0.01; // 10 mA

    // Note that the test voltages and currents are relatively loose right now due to some noise
    // on the MCU ADC measurement nodes.
    const LN_VALID_CURRENT_ZERO: Range<f32> = 0.0..0.01; // 0 mA to 10 mA
    const LN_VALID_CURRENT_SHUNT: Range<f32> = 0.009..0.011; // 9 mA to 11 mA
    const LN_VALID_CURRENT_SHOTTKY: Range<f32> = 0.009..0.011; // 9 mA to 11 mA
    const LN_VALID_VOLTAGE_ZERO: Range<f32> = -0.05..0.05; // -50 mV to 50 mV
    const LN_VALID_VOLTAGE_SHUNT: Range<f32> = 0.05..0.15; // 50 mV to 150 mV (10 ohm shunt resistor)
    const LN_VALID_VOLTAGE_SHOTTKY: Range<f32> = 0.2..0.3; // 200 mV to 300 mV (approximalte voltage drop over shottky to GND)

    // The current measurements on the HP side are very inaccurate right now.
    const HP_VALID_CURRENT_ZERO: Range<f32> = 0.0..1.;
    const HP_VALID_CURRENT_SHUNT: Range<f32> = 0.009..1.;
    const HP_VALID_CURRENT_SHOTTKY: Range<f32> = 0.009..1.;
    const HP_VALID_VOLTAGE_ZERO: Range<f32> = -0.05..0.05; // -50 mV to 50 mV
    const HP_VALID_VOLTAGE_SHUNT: Range<f32> = -0.05..0.1; // -50 mV to 100 mV (0.1 ohm shunt resistor)
    const HP_VALID_VOLTAGE_SHOTTKY: Range<f32> = 0.2..0.4; // 200 mV to 400 mV (approximalte voltage drop over shottky to GND)

    pub fn new(
        gpio: &'static spin::Mutex<Mcp230xx<I2C, Mcp23008>>,
        channel: Channel,
    ) -> Self {
        Output {
            iir: iir::IIR::new(0., 0.0, f32::MAX),
            relay: Relay::new(gpio, channel),
        }
    }
}
#[derive(PartialEq, Eq, Clone, Copy)]
pub struct Delay(pub fugit::MillisDuration<u64>);

pub mod sm {
    use super::*;

    statemachine! {
        transitions: {
            // Enable sequence
            *Disabled + Enable = SelftestZero,
            SelftestZero + Tick / set_test_current = SelftestShunt,
            SelftestShunt + Tick / engage_k0 = EnableWaitK0,
            EnableWaitK0 + Tick / set_test_current = SelftestShort,
            SelftestShort + Tick / engage_k1 = EnableWaitK1,
            EnableWaitK1 + Tick = RampCurrent,
            RampCurrent + Tick / increment_current = RampCurrent,
            RampCurrent + RampDone = Enabled,

            // Abort transitions
            EnableWaitK0 | EnableWaitK1 | RampCurrent + Disable = Abort,
            Abort + Tick / disengage_k1_and_hold_iir = DisableWaitK1,

            // Disable sequence
            Enabled + Disable / disengage_k1_and_hold_iir = DisableWaitK1,
            DisableWaitK1 + Tick / disengage_k0_and_reset_iir = DisableWaitK0,
            DisableWaitK0 + Tick = Disabled,

            Disabled + Disable = Disabled,
            Enabled + Enable = Enabled
        }
    }
}

impl<I2C, E> sm::StateMachineContext for Output<I2C>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    // Relay K0 is in the lower position, connecting the output to a 10 ohm shunt resistor.
    fn set_test_current(&mut self) {
        self.iir.y_offset = Self::TESTCURRENT;
    }

    fn engage_k0(&mut self) {
        self.iir.y_offset = 0.;
        // don't perform I2C transactions to MCP23008 on the headboard if it is intentionally not connected
        #[cfg(feature = "ai_artiq_laser_module")]
        self.relay.engage_k0();
    }

    fn increment_current(&mut self) {
        self.iir.y_offset += Output::<I2C>::RAMP_STEP;
    }

    fn disengage_k0_and_reset_iir(&mut self) {
        self.iir.y_offset = 0.;
        self.iir.ba = [0., 0., 0., 0., 0.];
        #[cfg(feature = "ai_artiq_laser_module")]
        self.relay.disengage_k0();
    }

    fn engage_k1(&mut self) {
        self.iir.y_offset = 0.;
        #[cfg(feature = "ai_artiq_laser_module")]
        self.relay.engage_k1();
    }

    fn disengage_k1_and_hold_iir(&mut self) {
        self.iir.y_offset = 0.;
        self.iir.ba = [0., 0., 0., 1., 0.];
        #[cfg(feature = "ai_artiq_laser_module")]
        self.relay.disengage_k1();
    }
}

impl<I2C, E> sm::StateMachine<Output<I2C>>
where
    I2C: WriteRead<Error = E> + Write<Error = E>,
    E: Debug,
{
    /// Start enabling sequence. Returns `Some(relay delay)` or `None` if we started an Abort
    /// or an error if SM is in a state where we can't enable.
    pub fn set_enable(
        &mut self,
        enable: bool,
    ) -> Result<Option<fugit::MillisDuration<u64>>, sm::Error> {
        let event = match enable {
            true => sm::Events::Enable,
            false => sm::Events::Disable,
        };
        if *self.process_event(event)? != sm::States::Abort {
            Ok(Some(Output::<I2C>::SET_DELAY))
        } else {
            Ok(None)
        }
    }

    /// Handle an event that happens during the enabling/disabling sequence.
    pub fn handle_tick(
        &mut self,
        target: &f32,
        channel: &Channel,
        reads: &[f32; 2], // voltage/current reads
    ) -> (Option<fugit::MillisDuration<u64>>, Option<SelfTest>) {
        let result = SelfTest::test(self.state(), channel, reads); // execute selftest if necessary in States
        match *self.state() {
            sm::States::EnableWaitK0
            | sm::States::DisableWaitK1
            | sm::States::EnableWaitK1
            | sm::States::DisableWaitK0
            | sm::States::Abort
            | sm::States::SelftestZero
            | sm::States::SelftestShunt
            | sm::States::SelftestShort => {
                self.process_event(sm::Events::Tick).unwrap();
            }
            // handle the ramp
            sm::States::RampCurrent => {
                if self.context().iir.y_offset
                    >= (*target - Output::<I2C>::RAMP_STEP)
                {
                    debug!("Current ramp done. Output enabled.");
                    self.process_event(sm::Events::RampDone).unwrap();
                } else {
                    self.process_event(sm::Events::Tick).unwrap();
                }
            }
            sm::States::Enabled | sm::States::Disabled => {
                panic!("unexpected state during output tick!")
            }
        }
        match *self.state() {
            sm::States::DisableWaitK0 => (Some(Relay::<I2C>::K0_DELAY), result),
            sm::States::DisableWaitK1 => (Some(Relay::<I2C>::K1_DELAY), result),
            sm::States::EnableWaitK0 => (Some(Relay::<I2C>::K0_DELAY), result),
            sm::States::EnableWaitK1 => (Some(Relay::<I2C>::K1_DELAY), result),
            sm::States::RampCurrent => {
                (Some(Output::<I2C>::RAMP_DELAY), result)
            }
            sm::States::SelftestShunt => {
                (Some(Output::<I2C>::SET_DELAY), result)
            }
            sm::States::SelftestShort => {
                (Some(Output::<I2C>::SET_DELAY), result)
            }
            _ => (None, result),
        }
    }

    pub fn iir(&mut self) -> &iir::IIR<f32> {
        &self.context().iir
    }

    pub fn is_enabled(&mut self) -> bool {
        *self.state() == sm::States::Enabled
    }
}
