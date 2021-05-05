use super::{NetworkReference, UpdateState};

use crate::hardware::{CycleCounter, EthernetPhy};

pub struct NetworkProcessor {
    stack: NetworkReference,
    phy: EthernetPhy,
    clock: CycleCounter,
    network_was_reset: bool,
}

impl NetworkProcessor {
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
        match self.phy.poll_link() {
            true => self.network_was_reset = false,

            // Only reset the network stack once per link reconnection. This prevents us from
            // sending an excessive number of DHCP requests.
            false if !self.network_was_reset => {
                self.network_was_reset = true;
                self.stack.lock(|stack| stack.handle_link_reset());
            }
            _ => {}
        };

        result
    }
}
