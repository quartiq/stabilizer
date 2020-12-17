use core::f32::consts::PI;
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dsp::cossin::cossin;

fn cossin_bench(c: &mut Criterion) {
    let zi = -0x7304_2531_i32;
    let zf = zi as f32 / i32::MAX as f32 * PI;
    c.bench_function("cossin(zi)", |b| b.iter(|| cossin(black_box(zi))));
    c.bench_function("zf.sin_cos()", |b| b.iter(|| black_box(zf).sin_cos()));
}

criterion_group!(benches, cossin_bench);
criterion_main!(benches);
