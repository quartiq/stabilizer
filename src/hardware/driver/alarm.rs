///! Driver alarm functionality
///!
///! Driver features an alarm to ensure safe co-operation with other devices.
///! The alarm is implemented via MQTT. See [Alarm] for details about the MQTT interface.
use fugit::ExtU64;
use miniconf::Miniconf;

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Alarm {
    /// "Alarm" topic. Publish "false" onto this topic to indicate valid operating conditions. This renews the alarm timeout.
    /// Publishing "true" or failing to publish "false" for `timeout` trips the alarm.
    ///
    /// # Path
    /// `alarm`
    ///
    /// # Value
    /// "true" or "false"
    alarm: bool,

    /// Set alarm to armed/disarmed.
    ///
    /// # Path
    /// `armed`
    ///
    /// # Value
    /// "true" or "false"
    armed: bool,

    /// Alarm timeout in milliseconds.
    ///
    /// # Path
    /// `timeout`
    ///
    /// # Value
    /// "true" or "false"
    timeout: u64,
}

impl Default for Alarm {
    fn default() -> Self {
        Self {
            alarm: true,
            armed: false,
            timeout: 1000,
        }
    }
}

pub enum Action {
    Spawn(fugit::Duration<u64, 1, 10000_u32>),
    Reschedule(fugit::Duration<u64, 1, 10000_u32>),
    Trip(fugit::Duration<u64, 1, 10000_u32>),
    Cancel,
}

impl Alarm {
    pub fn action(
        &self,
        path: Option<&str>,
        handle_is_some: bool,
    ) -> Option<Action> {
        match path {
            Some("alarm") => {
                if !self.alarm && handle_is_some {
                    Some(Action::Reschedule(self.timeout.millis()))
                } else if self.alarm && handle_is_some {
                    Some(Action::Trip(100.millis()))
                } else {
                    None
                }
            }
            Some("armed") => {
                if !self.armed && handle_is_some {
                    Some(Action::Cancel)
                } else if self.armed && !handle_is_some && !self.alarm {
                    Some(Action::Spawn(self.timeout.millis()))
                } else {
                    None
                }
            }
            _ => None,
        }
    }

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
