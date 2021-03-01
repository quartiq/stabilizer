use std::env;
use std::f64::consts::PI;
use std::fs::File;
use std::io::prelude::*;
use std::path::Path;

fn write_cossin_table() {
    const DEPTH: usize = 7;

    let out_dir = env::var_os("OUT_DIR").unwrap();
    let dest_path = Path::new(&out_dir).join("cossin_table.rs");
    let mut file = File::create(dest_path).unwrap();

    writeln!(file, "pub(crate) const COSSIN_DEPTH: usize = {};", DEPTH)
        .unwrap();
    write!(
        file,
        "pub(crate) const COSSIN: [u32; 1 << COSSIN_DEPTH] = ["
    )
    .unwrap();

    // Treat sin and cos as unsigned values since the sign will always be
    // positive in the range [0, pi/4).
    // No headroom for interpolation rounding error (this is needed for
    // DEPTH = 6 for example).
    const AMPLITUDE: f64 = u16::MAX as f64;

    for i in 0..(1 << DEPTH) {
        if i % 4 == 0 {
            write!(file, "\n   ").unwrap();
        }
        // Use midpoint samples to save one entry in the LUT
        let (sin, cos) =
            (PI / 4. * ((i as f64 + 0.5) / (1 << DEPTH) as f64)).sin_cos();
        // Add one bit accuracy to cos due to 0.5 < cos(z) <= 1 for |z| < pi/4
        // The -1 LSB is cancelled when unscaling with the biased half amplitude
        let cos = ((cos * 2. - 1.) * AMPLITUDE - 1.).round() as u32;
        let sin = (sin * AMPLITUDE).round() as u32;
        write!(file, " {},", cos + (sin << 16)).unwrap();
    }
    writeln!(file, "\n];").unwrap();

    println!("cargo:rerun-if-changed=build.rs");
}

fn main() {
    write_cossin_table();
}
