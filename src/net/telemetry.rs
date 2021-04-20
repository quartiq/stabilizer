use serde::Serialize;

#[derive(Serialize, Clone)]
pub struct Telemetry {
    pub latest_samples: [i16; 2],
    pub latest_outputs: [i16; 2],
    pub digital_inputs: [bool; 2],
}

impl Default for Telemetry {
    fn default() -> Self {
        Self {
            latest_samples: [0, 0],
            latest_outputs: [0, 0],
            digital_inputs: [false, false],
        }
    }
}
