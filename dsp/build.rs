use std::f64::consts::PI;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

const TABLE_DEPTH: usize = 8;
const TABLE_SIZE: usize = 1 << TABLE_DEPTH;
// Treat sin and cos as unsigned values since the sign will always be
// positive in the range [0, pi/4).
const SINCOS_MAX: f64 = u16::MAX as f64;

fn main() {
    let path = Path::new("src").join("cossin_table.txt");
    let display = path.display();

    let mut file = match File::create(&path) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(file) => file,
    };

    match file.write_all("[\n".as_bytes()) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(_) => (),
    }

    let phase_delta = PI / 4. / TABLE_SIZE as f64;
    let phase_offset = phase_delta / 2.;
    for i in 0..TABLE_SIZE {
        let phase = phase_offset + phase_delta * (i as f64);
        let cos = ((phase.cos() - 0.5) * 2. * SINCOS_MAX).round() as u16;
        let sin = (phase.sin() * SINCOS_MAX).round() as u16;
        let s = format!("    ({}, {}),\n", cos, sin);
        match file.write_all(s.as_bytes()) {
            Err(why) => panic!("failed to write to {}: {}", display, why),
            Ok(_) => (),
        }
    }

    match file.write_all("]\n".as_bytes()) {
        Err(why) => panic!("failed to write to {}: {}", display, why),
        Ok(_) => (),
    }
}
