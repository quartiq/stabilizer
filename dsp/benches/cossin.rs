use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dsp::trig::cossin;

fn cossin_bench(c: &mut Criterion) {
    let z = -0x7304_2531_i32;
    c.bench_function("cossin(z)", |b| b.iter(|| cossin(black_box(z))));
    c.bench_function("(z as f32).sin_cos()", |b| {
        b.iter(|| (black_box(z) as f32).sin_cos())
    });
}

criterion_group!(benches, cossin_bench);
criterion_main!(benches);
