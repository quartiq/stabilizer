/* origin: FreeBSD /usr/src/lib/msun/src/k_cosf.c */
/*
 * This has been adapted from libm to remove all use of
 * double-precision floating point.
 *
 * Conversion to float by Ian Lance Taylor, Cygnus Support, ian@cygnus.com.
 * Debugged and optimized by Bruce D. Evans.
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

/* |cos(x) - c(x)| < 2**-34.1 (~[-5.37e-11, 5.295e-11]). */
const C0: f32 = -0.5; /* -0x1p-1 */
const C1: f32 = 0.0416666232049465179443; /*  0x1.55553ep-5 */
const C2: f32 = -0.00138867634814232587814; /* -0x1.6c087ep-10 */
const C3: f32 = 0.0000243904487433610484004; /*  0x1.99342ep-16 */

#[cfg_attr(all(test, assert_no_panic), no_panic::no_panic)]
pub(crate) fn k_cosf(x: f32) -> f32 {
    let z = x * x;
    let w = z * z;
    let r = C2 + z * C3;
    ((1.0 + z * C0) + w * C1) + (w * z) * r
}
