///! Driver interlock functionality
///!
///! Driver features an interlock to ensure safe co-operation with other devices.
///! The interlock is implemented via MQTT. See [Interlock] for details about the MQTT interface.
use fugit::ExtU64;
use miniconf::Miniconf;

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Interlock {
    /// "Interlocked" topic. Publishing "true" onto this topic renews the interlock timeout.
    /// Publishing "false" or failing to publish `true` for `timeout` trips the interlock.
    ///
    /// # Path
    /// `interlocked`
    ///
    /// # Value
    /// "true" or "false"
    interlocked: bool,

    /// Set interlock to armed/disarmed.
    ///
    /// # Path
    /// `armed`
    ///
    /// # Value
    /// "true" or "false"
    armed: bool,

    /// Interlock timeout in milliseconds.
    ///
    /// # Path
    /// `timeout`
    ///
    /// # Value
    /// "true" or "false"
    timeout: u64,
}

impl Default for Interlock {
    fn default() -> Self {
        Self {
            interlocked: false,
            armed: false,
            timeout: 1000,
        }
    }
}

pub enum Action {
    Spawn(fugit::Duration<u64, 1, 10000_u32>),
    Reschedule(fugit::Duration<u64, 1, 10000_u32>),
    Cancel,
}

impl Interlock {
    pub fn action(
        &self,
        path: Option<&str>,
        handle_is_some: bool,
    ) -> Option<Action> {
        match path {
            Some("interlocked") => {
                if self.interlocked && handle_is_some {
                    Some(Action::Reschedule(self.timeout.millis()))
                } else if !self.interlocked && handle_is_some {
                    Some(Action::Reschedule(0.millis()))
                } else {
                    None
                }
            }
            Some("armed") => {
                if !self.armed && handle_is_some {
                    Some(Action::Cancel)
                } else if self.armed && !handle_is_some && self.interlocked {
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
