target remote :3333
set print asm-demangle on
monitor arm semihosting enable
# if using ITM with itmdump
# monitor tpiu config internal itm.fifo uart off 168000000
# or uart
# monitor tpiu config external uart off 168000000 2000000
# monitor itm port 0 on
load
# tbreak cortex_m_rt::reset_handler
monitor reset halt

# cycle counter delta tool, place two bkpts around the section
define qq
print *0xe0001004-$t0
set var $t0=*0xe0001004
continue
end
set var $t0=0xe0001004
continue
