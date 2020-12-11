use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dsp::trig::cossin;

fn cossin_bench(c: &mut Criterion) {
    c.bench_function("cossin(0)", |b| {
        b.iter(|| cossin(black_box(0)))
    });
}

criterion_group!(benches, cossin_bench);
criterion_main!(benches);
