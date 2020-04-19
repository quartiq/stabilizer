use super::pac;

pub fn dma1_setup(
    dma1: &pac::DMA1,
    dmamux1: &pac::DMAMUX1,
    ma: usize,
    pa0: usize,
    pa1: usize,
) {
    dma1.st[0].cr.modify(|_, w| w.en().clear_bit());
    while dma1.st[0].cr.read().en().bit_is_set() {}

    dma1.st[0].par.write(|w| unsafe { w.bits(pa0 as u32) });
    dma1.st[0].m0ar.write(|w| unsafe { w.bits(ma as u32) });
    dma1.st[0].ndtr.write(|w| w.ndt().bits(1));
    dmamux1.ccr[0].modify(|_, w| w.dmareq_id().tim2_up());
    dma1.st[0].cr.modify(|_, w| {
        w.pl()
            .medium()
            .circ()
            .enabled()
            .msize()
            .bits32()
            .minc()
            .fixed()
            .mburst()
            .single()
            .psize()
            .bits32()
            .pinc()
            .fixed()
            .pburst()
            .single()
            .dbm()
            .disabled()
            .dir()
            .memory_to_peripheral()
            .pfctrl()
            .dma()
    });
    dma1.st[0].fcr.modify(|_, w| w.dmdis().clear_bit());
    dma1.st[0].cr.modify(|_, w| w.en().set_bit());

    dma1.st[1].cr.modify(|_, w| w.en().clear_bit());
    while dma1.st[1].cr.read().en().bit_is_set() {}

    dma1.st[1].par.write(|w| unsafe { w.bits(pa1 as u32) });
    dma1.st[1].m0ar.write(|w| unsafe { w.bits(ma as u32) });
    dma1.st[1].ndtr.write(|w| w.ndt().bits(1));
    dmamux1.ccr[1].modify(|_, w| w.dmareq_id().tim2_up());
    dma1.st[1].cr.modify(|_, w| {
        w.pl()
            .medium()
            .circ()
            .enabled()
            .msize()
            .bits32()
            .minc()
            .fixed()
            .mburst()
            .single()
            .psize()
            .bits32()
            .pinc()
            .fixed()
            .pburst()
            .single()
            .dbm()
            .disabled()
            .dir()
            .memory_to_peripheral()
            .pfctrl()
            .dma()
    });
    dma1.st[1].fcr.modify(|_, w| w.dmdis().clear_bit());
    dma1.st[1].cr.modify(|_, w| w.en().set_bit());
}
