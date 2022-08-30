///! Driver Interlock functionality
///!
///! Todo docstring
use fugit::ExtU64;
use heapless::String;
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
            armed: false,
            clear: false,
            timeout: 1000,
        }
    }
}

pub enum Handle {
    Spawn(fugit::Duration<u64, 1, 10000_u32>),
    Reschedule(fugit::Duration<u64, 1, 10000_u32>),
    Cancel,
    Idle,
}

pub fn handle(
    path: String<64>,
    handle_is_some: bool,
    new: Interlock,
    old: Interlock,
) -> Option<Handle> {
    let path = path.as_str();
    if path == "interlock/interlock"
        || path == "interlock/armed"
        || path == "interlock/clear"
    {
        let cleared = !old.clear && new.clear;
        match (handle_is_some, new.armed, cleared, new.interlock) {
            // Interlock is armed and got cleared, first schedule.
            // Also overwrites used handle after a tripping event.
            (_, true, true, true) => Some(Handle::Spawn(new.timeout.millis())),
            // interlock renewal, push out
            (true, true, _, true) => {
                Some(Handle::Reschedule(new.timeout.millis()))
            }
            // interlock got disarmed, cancel
            (true, false, _, _) => Some(Handle::Cancel),
            // `false` published onto interlock, trip immediately
            (true, true, _, false) => Some(Handle::Reschedule(0.millis())),
            // interlock not in use / in all other cases
            _ => Some(Handle::Idle),
        }
    } else {
        None
    }
}
