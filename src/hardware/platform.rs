#![allow(static_mut_refs)]

/// Flag used to indicate that a reboot to DFU is requested.
const DFU_REBOOT_FLAG: u32 = 0xDEAD_BEEF;

/// Indicate a reboot to DFU is requested.
pub fn start_dfu_reboot() {
    extern "C" {
        static mut _bootflag: u8;
    }

    unsafe {
        let start_ptr = &mut _bootflag as *mut u8;
        core::ptr::write_unaligned(start_ptr.cast::<u32>(), DFU_REBOOT_FLAG);
    }

    cortex_m::peripheral::SCB::sys_reset();
}

/// Check if the DFU reboot flag is set, indicating a reboot to DFU is requested.
pub fn dfu_bootflag() -> bool {
    // Obtain panic region start and end from linker symbol _panic_dump_start and _panic_dump_end
    extern "C" {
        static mut _bootflag: u8;
    }

    unsafe {
        let start_ptr = &mut _bootflag as *mut u8;
        let set = DFU_REBOOT_FLAG
            == core::ptr::read_unaligned(start_ptr.cast::<u32>());

        // Clear the boot flag after checking it to ensure it doesn't stick between reboots.
        core::ptr::write_unaligned(start_ptr.cast::<u32>(), 0);
        set
    }
}

/// Execute the DFU bootloader stored in system memory.
///
/// # Note
/// This function must be called before any system configuration is performed, as the DFU
/// bootloader expects the system in an uninitialized state.
pub fn execute_system_bootloader() {
    // This process is largely adapted from
    // https://community.st.com/t5/stm32-mcus/jump-to-bootloader-from-application-on-stm32h7-devices/ta-p/49510
    cortex_m::interrupt::disable();

    // Disable the SysTick peripheral.
    let systick = unsafe { &*cortex_m::peripheral::SYST::PTR };
    unsafe {
        systick.csr.write(0);
        systick.rvr.write(0);
        systick.cvr.write(0);
    }

    // Clear NVIC interrupt flags and enables.
    let nvic = unsafe { &*cortex_m::peripheral::NVIC::PTR };
    for reg in nvic.icer.iter() {
        unsafe {
            reg.write(u32::MAX);
        }
    }

    for reg in nvic.icpr.iter() {
        unsafe {
            reg.write(u32::MAX);
        }
    }

    unsafe { cortex_m::interrupt::enable() };

    // The chip does not provide a means to modify the BOOT pins during
    // run-time. Jump to the bootloader in system memory instead.
    unsafe {
        let system_memory_address: *const u32 = 0x1FF0_9800 as *const u32;
        log::info!("Jumping to DFU");
        cortex_m::asm::bootload(system_memory_address);
    }
}
