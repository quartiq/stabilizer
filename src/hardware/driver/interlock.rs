///! Driver interlock functionality
///!
///! Driver features an interlock to ensure safe co-operation with other devices.
///! The interlock is implemented via MQTT. See [Interlock] for details about the MQTT interface.
use fugit::ExtU64;
use miniconf::Miniconf;

#[derive(Clone, Copy, Debug, Miniconf)]
pub struct Interlock {
    /// Interlock topic. Publishing "true" onto this topic renews the interlock timeout.
    /// Publishing "false" trips the interlock.
    ///
    /// # Path
    /// `interlock`
    ///
    /// # Value
    /// "true" or "false"
    interlock: bool,

    /// Set interlock to armed/disarmed. If the interlock tripped, a false->true transition is
    /// required to re-arm the interlock.
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
            interlock: false,
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
        path: Option<&str>,
        handle_is_some: bool,
        new: &Self,
        old: &Self,
    ) -> Option<Action> {
        match path {
            Some("interlock") => {
                if new.interlock && handle_is_some {
                    Some(Action::Reschedule(new.timeout.millis()))
                } else if !new.interlock && handle_is_some {
                    Some(Action::Reschedule(0.millis()))
                } else {
                    None
                }
            }
            Some("armed") => {
                if !new.armed && handle_is_some {
                    Some(Action::Cancel)
                } else if !old.armed
                    && new.armed
                    && !handle_is_some
                    && new.interlock
                {
                    Some(Action::Spawn(new.timeout.millis()))
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}
