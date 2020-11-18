/* origin: FreeBSD /usr/src/lib/msun/src/k_sinf.c */
/*
 * This has been adapted from libm to remove all use of
 * double-precision floating point.
 *
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 * Optimized by Bruce D. Evans.
 */
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

/* |sin(x)/x - s(x)| < 2**-37.5 (~[-4.89e-12, 4.824e-12]). */
const S1: f32 = -0.166666671633720397949; /* -0x1.555556p-3 */
const S2: f32 = 0.00833332911133766174316; /*  0x1.111108p-7 */
const S3: f32 = -0.000198393347091041505337; /* -0x1.a00f9ep-13 */
const S4: f32 = 0.0000027183114070794545114; /*  0x1.6cd878p-19 */

#[cfg_attr(all(test, assert_no_panic), no_panic::no_panic)]
pub(crate) fn k_sinf(x: f32) -> f32 {
    let z = x * x;
    let w = z * z;
    let r = S3 + z * S4;
    let s = z * x;
    (x + s * (S1 + z * S2)) + s * w * r
}
