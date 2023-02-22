use heapless::String;
use miniconf::{embedded_time, serde_json_core::from_slice};
use minimq::embedded_nal::{IpAddr, TcpClientStack};

use fugit::ExtU64;
use miniconf::Miniconf;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Miniconf, Serialize, Deserialize)]
/// Alarm functionality. Ensures safe co-operation with other devices.
///
/// The alarm is implemented via MQTT as a separate topic besides settings and telemetry.
/// Publish "false" onto the alarm topic to indicate valid operating conditions. This renews the alarm timeout.
/// Publishing "true" or failing to publish "false" for [`Self::timeout`] trips the alarm.
/// After the alarm is tripped, it has to be [rearm](Self::rearm())ed.
pub struct Settings {
    /// Set alarm to armed/disarmed.
    ///
    /// # Path
    /// `armed`
    ///
    /// # Value
    /// "true" or "false"
    pub armed: bool,

    /// Alarm timeout in milliseconds.
    ///
    /// # Path
    /// `timeout`
    ///
    /// # Value
    /// "true" or "false"
    timeout: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            armed: false,
            timeout: 1000,
        }
    }
}

pub enum Action {
    Spawn(fugit::Duration<u64, 1, 10000_u32>),
    Reschedule(fugit::Duration<u64, 1, 10000_u32>),
    Trip,
    Cancel,
}

#[derive(Clone, Copy, Debug)]
pub enum Change {
    Alarm(bool),
    Armed,
}

impl Settings {
    pub fn action(
        &self,
        change: Change,
        handle_is_some: bool,
    ) -> Option<Action> {
        match change {
            Change::Alarm(false) => {
                if handle_is_some {
                    Some(Action::Reschedule(self.timeout.millis()))
                } else {
                    None
                }
            }
            Change::Alarm(true) => {
                if handle_is_some {
                    Some(Action::Trip)
                } else {
                    None
                }
            }
            Change::Armed => {
                if !self.armed && handle_is_some {
                    Some(Action::Cancel)
                } else if self.armed && !handle_is_some {
                    Some(Action::Spawn(self.timeout.millis()))
                } else {
                    None
                }
            }
        }
    }

    /// rearm the alarm
    pub fn rearm(
        &self,
        handle_is_some: bool,
    ) -> Option<fugit::Duration<u64, 1, 10000_u32>> {
        if !handle_is_some && self.armed {
            Some(self.timeout.millis())
        } else {
            None
        }
    }
}

/// Stabilizer mqtt alarm listener.
pub struct Alarm<
    Stack,
    Clock,
    const MESSAGE_SIZE: usize,
    const MAX_TOPIC_LENGTH: usize,
> where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock,
{
    mqtt: minimq::Minimq<Stack, Clock, MESSAGE_SIZE, 1>,
    subscribed: bool,
    prefix: String<MAX_TOPIC_LENGTH>,
}

impl<
        Stack,
        Clock,
        const MESSAGE_SIZE: usize,
        const MAX_TOPIC_LENGTH: usize,
    > Alarm<Stack, Clock, MESSAGE_SIZE, MAX_TOPIC_LENGTH>
where
    Stack: TcpClientStack,
    Clock: embedded_time::Clock + Clone,
{
    pub fn new(
        stack: Stack,
        client_id: &str,
        prefix: &str,
        broker: IpAddr,
        clock: Clock,
    ) -> Result<Self, minimq::Error<Stack::Error>> {
        let mqtt = minimq::Minimq::new(broker, client_id, stack, clock)?;
        let mut prefix = String::from(prefix);
        prefix.push_str("/alarm").unwrap();
        Ok(Self {
            mqtt,
            subscribed: false,
            prefix,
        })
    }

    /// Update the alarm mqtt interface. Returns Ok(false) if the alarm was renewed.
    /// (aka if true has been published on the alarm topic)
    /// Returns Ok(true) if the alarm was triggered.
    /// Returns None if we only just subscribed or if there was an invalid message on the alarm topic.
    pub fn update(
        &mut self,
    ) -> Result<Option<bool>, minimq::Error<Stack::Error>> {
        if !self.mqtt.client().is_connected() {
            // reset subscription if client disconnects
            self.subscribed = false;
        }
        if !self.subscribed && self.mqtt.client().is_connected() {
            let result = self
                .mqtt
                .client()
                .subscribe(&[(self.prefix.as_str()).into()], &[])
                .map(|_| None); // return None if we just subscribed
            self.subscribed = true;
            log::info!("Alarm subscribed to {:?}", self.prefix);
            result
        } else {
            self.mqtt
                .poll(|_client, _topic, message, _properties| match from_slice(message) {
                    Ok((true, 4)) => Some(true),
                    Ok((false, 5)) => Some(false),
                    _ => {
                        log::error!("Invalid alarm message: {:?}", message);
                        None
                    }
                })
                .map(|res| res.flatten())
        }
    }
}
