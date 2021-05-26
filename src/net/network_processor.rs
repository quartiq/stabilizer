///! Task to process network hardware.
///!
///! # Design
///! The network processir is a small taks to regularly process incoming data over ethernet, handle
///! the ethernet PHY state, and reset the network as appropriate.
use super::{NetworkReference, UpdateState};
use crate::hardware::{CycleCounter, EthernetPhy};

/// Processor for managing network hardware.
pub struct NetworkProcessor {
    stack: NetworkReference,
    phy: EthernetPhy,
    clock: CycleCounter,
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
    pub fn new(
        stack: NetworkReference,
        phy: EthernetPhy,
        clock: CycleCounter,
    ) -> Self {
        Self {
            stack,
            phy,
            clock,
            network_was_reset: false,
        }
    }

    pub fn egress(&mut self) {
        let now = self.clock.current_ms();
        self.stack.lock(|stack| stack.poll(now)).ok();
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
        let now = self.clock.current_ms();

        let result = match self.stack.lock(|stack| stack.poll(now)) {
            Ok(true) => UpdateState::Updated,
            Ok(false) => UpdateState::NoChange,
            Err(err) => {
                log::info!("Network error: {:?}", err);
                UpdateState::Updated
            }
        };

        // If the PHY indicates there's no more ethernet link, reset the DHCP server in the network
        // stack.
        // TODO: Poll the link state in a task and handle resets. Polling this often is slow and
        // uses necessary CPU time.
        //match self.phy.poll_link() {
        //    true => self.network_was_reset = false,

        //    // Only reset the network stack once per link reconnection. This prevents us from
        //    // sending an excessive number of DHCP requests.
        //    false if !self.network_was_reset => {
        //        self.network_was_reset = true;
        //        self.stack.lock(|stack| stack.handle_link_reset());
        //    }
        //    _ => {}
        //};

        result
    }
}
