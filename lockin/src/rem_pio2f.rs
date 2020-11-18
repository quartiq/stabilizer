/* origin: FreeBSD /usr/src/lib/msun/src/e_rem_pio2f.c */
/*
 * This has been adapted from libm to remove all use of
 * double-precision floating point. Additionally, it supports our
 * particular case where the argument is guaranteed to be in the range
 * 0 to 2pi. The implementation has been completely changed, but the
 * name and interface have been retained.
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

use core::f32::consts::{PI, FRAC_PI_2};

/// Remainder of `x` with respect to pi/2. `x` must be in the range
/// [0, 2pi).
pub fn rem_pio2f(x: f32) -> (i32, f32) {
    debug_assert!(x >= 0. && x < 2. * PI);

    let mut y = x;
    let mut i: i32 = 0;
    while y > FRAC_PI_2 {
        y -= FRAC_PI_2;
        i += 1;
    }
    (i, y)
}
