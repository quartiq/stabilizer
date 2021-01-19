use super::{divide_round, pll::PLL};

/// Processes external timestamps to produce the frequency and initial phase of the demodulation
/// signal.
pub struct TimestampHandler {
    pll: PLL,
    pll_shift_frequency: u8,
    pll_shift_phase: u8,
    // Index of the current ADC batch.
    batch_index: u32,
    // Most recent phase and frequency values of the external reference.
    reference_phase: i64,
    reference_frequency: i64,
    adc_sample_ticks_log2: usize,
    sample_buffer_size_log2: usize,
}

impl TimestampHandler {
    /// Construct a new `TimestampHandler` instance.
    ///
    /// # Args
    /// * `pll_shift_frequency` - See `PLL::update()`.
    /// * `pll_shift_phase` - See `PLL::update()`.
    /// * `adc_sample_ticks_log2` - Number of ticks in one ADC sampling timer period.
    /// * `sample_buffer_size_log2` - Number of ADC samples in one processing batch.
    ///
    /// # Returns
    /// New `TimestampHandler` instance.
    pub fn new(
        pll_shift_frequency: u8,
        pll_shift_phase: u8,
        adc_sample_ticks_log2: usize,
        sample_buffer_size_log2: usize,
    ) -> Self {
        TimestampHandler {
            pll: PLL::default(),
            pll_shift_frequency,
            pll_shift_phase,
            batch_index: 0,
            reference_phase: 0,
            reference_frequency: 0,
            adc_sample_ticks_log2,
            sample_buffer_size_log2,
        }
    }

    /// Compute the initial phase and frequency of the demodulation signal.
    ///
    /// # Args
    /// * `timestamp` - Counter value corresponding to an external reference edge.
    ///
    /// # Returns
    /// Tuple consisting of the initial phase value and frequency of the demodulation signal.
    pub fn update(&mut self, timestamp: Option<u32>) -> (u32, u32) {
        if let Some(t) = timestamp {
            let (phase, frequency) = self.pll.update(
                t as i32,
                self.pll_shift_frequency,
                self.pll_shift_phase,
            );
            self.reference_phase = phase as u32 as i64;
            self.reference_frequency = frequency as u32 as i64;
        }

        let demodulation_frequency: u32;
        let demodulation_initial_phase: u32;

        if self.reference_frequency == 0 {
            demodulation_frequency = u32::MAX;
            demodulation_initial_phase = u32::MAX;
        } else {
            demodulation_frequency = divide_round(
                1 << (32 + self.adc_sample_ticks_log2),
                self.reference_frequency,
            ) as u32;
            demodulation_initial_phase = divide_round(
                (((self.batch_index as i64)
                    << (self.adc_sample_ticks_log2
                        + self.sample_buffer_size_log2))
                    - self.reference_phase)
                    << 32,
                self.reference_frequency,
            ) as u32;
        }

        if self.batch_index
            < (1 << (32
                - self.adc_sample_ticks_log2
                - self.sample_buffer_size_log2))
                - 1
        {
            self.batch_index += 1;
        } else {
            self.batch_index = 0;
            self.reference_phase -= 1 << 32;
        }

        (demodulation_initial_phase, demodulation_frequency)
    }
}
