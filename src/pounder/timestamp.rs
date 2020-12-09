///! ADC sample timestamper using external Pounder reference clock.
use stm32h7xx_hal as hal;

use hal::{
    dma::{
        dma::{DMAReq, DmaConfig},
        traits::TargetAddress,
        PeripheralToMemory, Transfer,
    },
    rcc::ResetEnable,
};

use crate::{timers, SAMPLE_BUFFER_SIZE};

struct TimestampDma {
    _dma_request: timers::tim2::UpdateEvent,
}

pub struct Timer {
    timer: hal::stm32::TIM8,
    dma: Option<TimestampDma>,
}

// Note(unsafe): This is safe to implement because we take ownership of the DMA request.
// Additionally, we only read the registers in PeripheralToMemory mode, so it is always safe to
// access them.
unsafe impl TargetAddress<PeripheralToMemory> for TimestampDma {
    // TIM8 is a 16-bit timer.
    type MemSize = u16;

    // Note: It is safe for us to us the TIM2_UPDATE DMA request because the Timer
    // maintains ownership of the UpdateEvent object for this timer.
    const REQUEST_LINE: Option<u8> = Some(DMAReq::TIM2_UP as u8);

    fn address(&self) -> u32 {
        let regs = unsafe { &*hal::stm32::TIM8::ptr() };
        &regs.cnt as *const _ as u32
    }
}

pub enum Prescaler {
    Div4,
    Div8,
}

impl Timer {
    pub fn new(
        timer: hal::stm32::TIM8,
        prec: hal::rcc::rec::Tim8,
        _external_source: hal::gpio::gpioa::PA0<
            hal::gpio::Alternate<hal::gpio::AF3>,
        >,
        prescaler: Prescaler,
        update_event: timers::tim2::UpdateEvent,
        period: u16,
    ) -> Self {
        prec.reset().enable();

        let divisor = match prescaler {
            Prescaler::Div4 => hal::stm32::tim1::smcr::ETPS_A::DIV4,
            Prescaler::Div8 => hal::stm32::tim1::smcr::ETPS_A::DIV8,
        };

        // Configure the timer to utilize an external clock source with the provided divider on the
        // ETR.
        timer
            .smcr
            .modify(|_, w| w.etps().variant(divisor).ece().set_bit());

        // Set the timer period and generate an update of the timer registers so that ARR takes
        // effect.
        timer.arr.write(|w| w.arr().bits(period));

        timer.egr.write(|w| w.ug().set_bit());

        // Allow TIM2 updates to generate DMA requests.
        update_event.listen_dma();

        // Enable the timer.
        timer.cr1.modify(|_, w| w.cen().set_bit());

        let dma = TimestampDma {
            _dma_request: update_event,
        };

        Self {
            timer,
            dma: Some(dma),
        }
    }

    /// Update the timer period.
    ///
    /// # Note
    /// Timer period updates will take effect after the current period elapses.
    fn set_period(&mut self, period: u16) {
        // Modify the period register.
        self.timer.arr.write(|w| w.arr().bits(period));
    }

    fn dma_transfer(&mut self) -> TimestampDma {
        self.dma.take().unwrap()
    }
}

#[link_section = ".axisram.buffers"]
static mut BUF: [[u16; SAMPLE_BUFFER_SIZE]; 3] = [[0; SAMPLE_BUFFER_SIZE]; 3];

pub struct Timestamper {
    next_buffer: Option<&'static mut [u16; SAMPLE_BUFFER_SIZE]>,
    timer: Timer,
    transfer: Transfer<
        hal::dma::dma::Stream7<hal::stm32::DMA1>,
        TimestampDma,
        PeripheralToMemory,
        &'static mut [u16; SAMPLE_BUFFER_SIZE],
    >,
}

impl Timestamper {
    pub fn new(
        mut timer: Timer,
        stream: hal::dma::dma::Stream7<hal::stm32::DMA1>,
    ) -> Self {
        let config = DmaConfig::default()
            .memory_increment(true)
            .circular_buffer(true)
            .double_buffer(true);

        // The data transfer is always a transfer of data from the peripheral to a RAM buffer.
        let mut data_transfer: Transfer<_, _, PeripheralToMemory, _> =
            Transfer::init(
                stream,
                timer.dma_transfer(),
                // Note(unsafe): The BUF[0] and BUF[1] is "owned" by this peripheral.
                // It shall not be used anywhere else in the module.
                unsafe { &mut BUF[0] },
                unsafe { Some(&mut BUF[1]) },
                config,
            );

        data_transfer.start(|_| {});

        Self {
            timer,
            transfer: data_transfer,
            next_buffer: unsafe { Some(&mut BUF[2]) },
        }
    }

    pub fn update_period(&mut self, period: u16) {
        self.timer.set_period(period);
    }

    /// Obtain a buffer filled with timestamps.
    ///
    /// # Returns
    /// A reference to the underlying buffer that has been filled with timestamps.
    pub fn acquire_buffer(&mut self) -> &[u16; SAMPLE_BUFFER_SIZE] {
        // Wait for the transfer to fully complete before continuing.
        // Note: If a device hangs up, check that this conditional is passing correctly, as there is
        // no time-out checks here in the interest of execution speed.
        while !self.transfer.get_transfer_complete_flag() {}

        let next_buffer = self.next_buffer.take().unwrap();

        // Start the next transfer.
        self.transfer.clear_interrupts();
        let (prev_buffer, _, _) =
            self.transfer.next_transfer(next_buffer).unwrap();

        self.next_buffer.replace(prev_buffer); // .unwrap_none() https://github.com/rust-lang/rust/issues/62633

        self.next_buffer.as_ref().unwrap()
    }
}
