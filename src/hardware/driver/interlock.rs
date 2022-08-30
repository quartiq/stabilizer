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

    /// Set interlock to armed/disarmed.
    ///
    /// # Path
    /// `armed`
    ///
    /// # Value
    /// "true" or "false"
    armed: bool,

    /// A positive flank from "false" to "true" clears the interrupt.
    ///
    /// # Path
    /// `armed`
    ///
    /// # Value
    /// "true" or "false"
    clear: bool,

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
            armed: true,
            clear: false,
            timeout: 1000,
        }
    }
}

pub enum Action {
    Spawn(fugit::Duration<u64, 1, 10000_u32>),
    Reschedule(fugit::Duration<u64, 1, 10000_u32>),
    Cancel,
    None,
}

impl Interlock {
    pub fn handle(
        handle_is_some: bool,
        new: Interlock,
        old: Interlock,
    ) -> Action {
        let cleared = !old.clear && new.clear;
        match (handle_is_some, new.armed, cleared, new.interlock) {
            // Interlock is armed and got cleared, first schedule.
            // Also overwrites used handle after a tripping event.
            (_, true, true, true) => Action::Spawn(new.timeout.millis()),
            // interlock renewal, push out
            (true, true, _, true) => Action::Reschedule(new.timeout.millis()),
            // interlock got disarmed, cancel
            (true, false, _, _) => Action::Cancel,
            // `false` published onto interlock, trip immediately
            (true, true, _, false) => Action::Reschedule(0.millis()),
            // interlock not in use / in all other cases
            _ => Action::None,
        }
    }
}
