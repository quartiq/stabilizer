use core::f32::consts::PI;
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dsp::{atan2, cossin};

fn atan2_bench(c: &mut Criterion) {
    let xi = (10 << 16) as i32;
    let xf = xi as f32 / i32::MAX as f32;

    let yi = (-26_328 << 16) as i32;
    let yf = yi as f32 / i32::MAX as f32;

    c.bench_function("atan2(y, x)", |b| {
        b.iter(|| atan2(black_box(yi), black_box(xi)))
    });
    c.bench_function("y.atan2(x)", |b| {
        b.iter(|| black_box(yf).atan2(black_box(xf)))
    });
}

fn cossin_bench(c: &mut Criterion) {
    let zi = -0x7304_2531_i32;
    let zf = zi as f32 / i32::MAX as f32 * PI;
    c.bench_function("cossin(zi)", |b| b.iter(|| cossin(black_box(zi))));
    c.bench_function("zf.sin_cos()", |b| b.iter(|| black_box(zf).sin_cos()));
}

criterion_group!(benches, atan2_bench, cossin_bench);
criterion_main!(benches);
