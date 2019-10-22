target remote :3333
set print asm-demangle on
# https://github.com/rust-embedded/cortex-m-rt/issues/139
set backtrace limit 32
monitor arm semihosting enable
# if using ITM with itmdump
# monitor tpiu config internal itm.fifo uart off 168000000
# or uart
# monitor tpiu config external uart off 168000000 2000000
# monitor itm port 0 on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind

load
# tbreak cortex_m_rt::reset_handler
# monitor reset halt

# cycle counter delta tool, place two bkpts around the section
set var $cc=0xe0001004
define qq
print *$cc-$t0
set var $t0=*$cc
continue
end
#set var $t0=*$cc
continue
