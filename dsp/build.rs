use std::f64::consts::PI;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

fn write_cossin_table() {
    const DEPTH: usize = 7;

    let mut file =
        File::create(Path::new("src").join("cossin_table.rs")).unwrap();
    writeln!(file, "pub(crate) const COSSIN_DEPTH: usize = {};", DEPTH)
        .unwrap();
    write!(
        file,
        "pub(crate) const COSSIN: [(u16, u16); 1 << COSSIN_DEPTH] = ["
    )
    .unwrap();

    // Treat sin and cos as unsigned values since the sign will always be
    // positive in the range [0, pi/4).
    // No headroom for interpolation rounding error (this is needed for
    // DEPTH = 6 for example).
    const AMPLITUDE: f64 = u16::MAX as f64;

    for i in 0..(1 << DEPTH) {
        // use midpoint samples to save one entry in the LUT
        let phase = (PI / 4. / (1 << DEPTH) as f64) * (i as f64 + 0.5);
        // add one bit accuracy to cos due to 0.5 < cos(z) <= 1 for |z| < pi/4
        let cos = ((phase.cos() - 0.5) * 2. * AMPLITUDE).round() as u16;
        let sin = (phase.sin() * AMPLITUDE).round() as u16;
        if i % 4 == 0 {
            write!(file, "\n   ").unwrap();
        }
        write!(file, " ({}, {}),", cos, sin).unwrap();
    }
    writeln!(file, "\n];").unwrap();
}

fn main() {
    write_cossin_table();
}
