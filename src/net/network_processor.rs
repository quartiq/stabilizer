///! Task to process network hardware.
///!
///! # Design
///! The network processir is a small taks to regularly process incoming data over ethernet, handle
///! the ethernet PHY state, and reset the network as appropriate.
use super::{NetworkReference, UpdateState};
use crate::hardware::{system_timer::SystemTimer, EthernetPhy};
use core::convert::TryFrom;
use rtic::time::{duration::Milliseconds, fraction::Fraction, Clock};

/// Processor for managing network hardware.
pub struct NetworkProcessor {
    stack: NetworkReference,
    phy: EthernetPhy,
    clock: SystemTimer,
    network_was_reset: bool,
}

impl NetworkProcessor {
    /// Construct a new network processor.
    ///
    /// # Args
    /// * `stack` - A reference to the shared network stack
    /// * `phy` - The ethernet PHY used for the network.
    /// * `clock` - The clock used for providing time to the network.
    ///
    /// # Returns
    /// The newly constructed processor.
    pub fn new(stack: NetworkReference, phy: EthernetPhy) -> Self {
        // We make assumptions about the tick rate of the timer and storage sizes below. To make
        // sure those assumptions are valid, check them once on startup.
        assert!(SystemTimer::SCALING_FACTOR <= Fraction::new(1, 1_000));

        Self {
            stack,
            phy,
            clock: SystemTimer::default(),
            network_was_reset: false,
        }
    }

    /// Handle ethernet link connection status.
    ///
    /// # Note
    /// This may take non-trivial amounts of time to communicate with the PHY. As such, this should
    /// only be called as often as necessary (e.g. once per second or so).
    pub fn handle_link(&mut self) {
        // If the PHY indicates there's no more ethernet link, reset the DHCP server in the network
        // stack.
        let link_up = self.phy.poll_link();
        match (link_up, self.network_was_reset) {
            (true, true) => {
                log::warn!("Network link UP");
                self.network_was_reset = false;
            }
            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            (false, false) => {
                log::warn!("Network link DOWN");
                self.network_was_reset = true;
                self.stack.lock(|stack| stack.handle_link_reset());
            }
            _ => {}
        };
    }

    /// Process and update the state of the network.
    ///
    /// # Note
    /// This function should be called regularly before other network tasks to update the state of
    /// all relevant network sockets.
    ///
    /// # Returns
    /// An update state corresponding with any changes in the underlying network.
    pub fn update(&mut self) -> UpdateState {
        // Service the network stack to process any inbound and outbound traffic.
        let current_ms: Milliseconds<u32> = {
            // Note(unwrap): The system timer is infallible.
            let now = self.clock.try_now().unwrap();

            // Note(unwrap): The underlying timer operates at a tick frequency greater than 1 time
            // per millisecond (see the assertion in `new()`). Therefore, at the maximum timer tick
            // count, the duration since the epoch is defined to be less than 2^32 milliseconds.
            // Thus, a 32-bit millisecond counter should always be sufficient to store this
            // duration.
            Milliseconds::try_from(now.duration_since_epoch()).unwrap()
        };

        match self.stack.lock(|stack| stack.poll(current_ms.0)) {
            Ok(true) => UpdateState::Updated,
            Ok(false) => UpdateState::NoChange,
            Err(_) => UpdateState::Updated,
        }
    }
}
