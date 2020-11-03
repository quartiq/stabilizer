use super::{hal, DmaConfig, PeripheralToMemory, MemoryToPeripheral, TargetAddress, Transfer, DMAReq,
Stream};

const INPUT_BUFFER_SIZE: usize = 1;

#[link_section = ".axisram.buffers"]
static mut SPI_START: [u16; 1] = [0x00];

#[link_section = ".axisram.buffers"]
static mut ADC0_BUF0: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC0_BUF1: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC1_BUF0: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut ADC1_BUF1: [u16; INPUT_BUFFER_SIZE] = [0; INPUT_BUFFER_SIZE];

struct SPI2 {}

impl SPI2 {
    pub fn new() -> Self {
        Self {}
    }
}

unsafe impl TargetAddress<MemoryToPeripheral> for SPI2 {
    type MemSize = u16;

    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_UP as u8);

    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::SPI2::ptr() };
        &regs.txdr as *const _ as u32
    }
}

struct SPI3 {}

impl SPI3 {
    pub fn new() -> Self {
        Self {}
    }
}

unsafe impl TargetAddress<MemoryToPeripheral> for SPI3 {
    type MemSize = u16;

    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_UP as u8);

    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::SPI3::ptr() };
        &regs.txdr as *const _ as u32
    }
}

pub struct Adc0Input {
    next_buffer: Option<&'static mut [u16; INPUT_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream1<hal::stm32::DMA1>,
        hal::spi::Spi<hal::stm32::SPI2, hal::spi::Disabled, u16>,
        PeripheralToMemory,
        &'static mut [u16; INPUT_BUFFER_SIZE]>,
}

impl Adc0Input {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI2, hal::spi::Enabled, u16>,
        trigger_stream: hal::dma::dma::Stream0<hal::stm32::DMA1>,
        data_stream: hal::dma::dma::Stream1<hal::stm32::DMA1>,
    ) -> Self {
        let trigger_config = DmaConfig::default()
            .memory_increment(false)
            .peripheral_increment(false)
            .circular_buffer(true);

        let mut trigger_transfer: Transfer<_, _, MemoryToPeripheral, _ > = Transfer::init(
            trigger_stream,
            &SPI2::new(),
            unsafe { &mut SPI_START },
            None,
            trigger_config);

        let data_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .peripheral_increment(false);

        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _ > = Transfer::init(
            data_stream,
            &spi,
            unsafe { &mut ADC0_BUF0 },
            None,
            data_config);

        spi.enable_dma_rx();
        spi.enable_dma_tx();

        let spi = spi.enable();
        spi.inner().cr1.modify(|_, w| w.cstart().started());

        data_transfer.start();
        trigger_transfer.start();

        Self {
            next_buffer: unsafe { Some(&mut ADC0_BUF1) },
            transfer: data_transfer,
        }
    }

    pub fn transfer_complete_handler(&mut self) -> &[u16; INPUT_BUFFER_SIZE] {
        let next_buffer = self.next_buffer.take().unwrap();
        let (prev_buffer, _) = self.transfer.next_transfer(next_buffer).unwrap();
        self.next_buffer.replace(prev_buffer);
        self.next_buffer.as_ref().unwrap()
    }
}

pub struct Adc1Input {
    next_buffer: Option<&'static mut [u16; INPUT_BUFFER_SIZE]>,
    transfer: Transfer<
        hal::dma::dma::Stream3<hal::stm32::DMA1>,
        hal::spi::Spi<hal::stm32::SPI3, hal::spi::Disabled, u16>,
        PeripheralToMemory,
        &'static mut [u16; INPUT_BUFFER_SIZE]>,
}

impl Adc1Input {
    pub fn new(
        spi: hal::spi::Spi<hal::stm32::SPI3, hal::spi::Enabled, u16>,
        trigger_stream: hal::dma::dma::Stream2<hal::stm32::DMA1>,
        data_stream: hal::dma::dma::Stream3<hal::stm32::DMA1>,
    ) -> Self {
        let trigger_config = DmaConfig::default()
            .memory_increment(false)
            .peripheral_increment(false)
            .circular_buffer(true);

        let mut trigger_transfer: Transfer<_, _, MemoryToPeripheral, _ > = Transfer::init(
            trigger_stream,
            &SPI3::new(),
            unsafe { &mut SPI_START },
            None,
            trigger_config);

        let data_config = DmaConfig::default()
            .memory_increment(true)
            .transfer_complete_interrupt(true)
            .peripheral_increment(false);

        let mut spi = spi.disable();
        spi.listen(hal::spi::Event::Error);

        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _ > = Transfer::init(
            data_stream,
            &spi,
            unsafe { &mut ADC1_BUF0 },
            None,
            data_config);

        spi.enable_dma_rx();
        spi.enable_dma_tx();

        let spi = spi.enable();
        spi.inner().cr1.modify(|_, w| w.cstart().started());

        data_transfer.start();
        trigger_transfer.start();

        Self {
            next_buffer: unsafe { Some(&mut ADC1_BUF1) },
            transfer: data_transfer,
        }
    }

    pub fn transfer_complete_handler(&mut self) -> &[u16; INPUT_BUFFER_SIZE] {
        let next_buffer = self.next_buffer.take().unwrap();
        let (prev_buffer, _) = self.transfer.next_transfer(next_buffer).unwrap();
        self.next_buffer.replace(prev_buffer);
        self.next_buffer.as_ref().unwrap()
    }
}
