#[repr(align(4))]
struct WatchVariable {
    pub id: u8,
}

static mut ENTRY_WATCH: WatchVariable = WatchVariable { id: 0 };
static mut EXIT_WATCH: WatchVariable = WatchVariable { id: 0 };

#[repr(u8)]
pub enum TraceEvent {
    Unknown = 0,
}

pub fn enter_block(event: TraceEvent) {
    unsafe {
        core::ptr::write_volatile(&mut ENTRY_WATCH.id, event as u8);
    }
}

pub fn exit_block(event: TraceEvent) {
    unsafe {
        core::ptr::write_volatile(&mut EXIT_WATCH.id, event as u8);
    }
}

pub fn configure_dwt(_dwt: &mut cortex_m::peripheral::DWT) {
    assert!(cortex_m::peripheral::DWT::num_comp() > 2);

    let dwt = unsafe { &*cortex_m::peripheral::DWT::ptr() };

    unsafe {
        // Enable exception tracing
        dwt.ctrl.modify(|r| r | (1 << 16));
    }

    // Configure comparators
    for (comparator, address) in [
        (&dwt.c[1], unsafe { &ENTRY_WATCH as *const _ }),
        (&dwt.c[2], unsafe { &EXIT_WATCH as *const _ }),
    ] {
        unsafe {
            comparator.function.write(0b1101);
            comparator.mask.write(0);
            comparator.comp.write(address as u32);
        }
    }
}

pub fn configure_tpiu(_tpiu: &mut cortex_m::peripheral::TPIU, refclk_hz: u32) {
    let tpiu = unsafe { &*cortex_m::peripheral::TPIU::ptr() };

    // TODO: Check ref clock frequency.
    unsafe {
        tpiu.acpr.write((refclk_hz / 115_200) - 1);
        // Set the trace output  to AsyncSWO(NRZ)
        tpiu.sppr.modify(|mut r| {
            r &= !(0b11);
            r |= 0b10;
            r
        });

        // Disable continuous formatting
        tpiu.ffcr.modify(|mut r| {
            r &= !(1 << 1); //ENFCONT
            r
        });
    }
}

pub fn configure_itm(_itm: &mut cortex_m::peripheral::ITM) {
    let itm = unsafe { &*cortex_m::peripheral::ITM::ptr() };

    unsafe {
        itm.tcr.modify(|mut r| {
            r |= 1 << 0; // ITMENA

            // Enable local timestamps
            r |= 1 << 1; // TSENA

            // Enable DWT event forwarding.
            r |= 1 << 3; // TXENA

            // Configure the bus ID to 1, not sure why?
            r |= 1 << 16; // TraceBusID

            // Clear the prescalers for global and local timestamps.
            r &= !(0b11 << 8); //TSPrescale
            r &= !(0b11 << 10); //GTSFREQ

            r
        });
    }
}
