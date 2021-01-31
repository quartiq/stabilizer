use core::f32::consts::PI;
use criterion::{black_box, criterion_group, criterion_main, Criterion};
use dsp::{atan2, cossin, pll::PLL, rpll::RPLL};

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

fn rpll_bench(c: &mut Criterion) {
    let mut dut = RPLL::new(8);
    c.bench_function("RPLL::update(Some(t), 21, 20)", |b| {
        b.iter(|| dut.update(black_box(Some(0x241)), 21, 20))
    });
    c.bench_function("RPLL::update(Some(t), sf, sp)", |b| {
        b.iter(|| {
            dut.update(black_box(Some(0x241)), black_box(21), black_box(20))
        })
    });
}

fn pll_bench(c: &mut Criterion) {
    let mut dut = PLL::default();
    c.bench_function("PLL::update(t, 12, 11)", |b| {
        b.iter(|| dut.update(black_box(0x1234), 12, 1))
    });
    c.bench_function("PLL::update(t, sf, sp)", |b| {
        b.iter(|| dut.update(black_box(0x241), black_box(21), black_box(20)))
    });
}

criterion_group!(trig, atan2_bench, cossin_bench);
criterion_group!(pll, rpll_bench, pll_bench);
criterion_main!(trig, pll);
