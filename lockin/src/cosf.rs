/* origin: FreeBSD /usr/src/lib/msun/src/s_cosf.c */
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

use super::k_sinf::k_sinf;
use super::k_cosf::k_cosf;
use super::rem_pio2f::rem_pio2f;

use core::f32::consts::FRAC_PI_2;

/* Small multiples of pi/2 rounded to double precision. */
const C1_PIO2: f32 = 1. * FRAC_PI_2;
const C2_PIO2: f32 = 2. * FRAC_PI_2;
const C3_PIO2: f32 = 3. * FRAC_PI_2;
const C4_PIO2: f32 = 4. * FRAC_PI_2;

#[cfg_attr(all(test, assert_no_panic), no_panic::no_panic)]
pub fn cosf(x: f32) -> f32 {
    let x1p120 = f32::from_bits(0x7b800000); // 0x1p120f === 2 ^ 120

    let mut ix = x.to_bits();
    let sign = (ix >> 31) != 0;
    ix &= 0x7fffffff;

    if ix <= 0x3f490fda {
        /* |x| ~<= pi/4 */
        if ix < 0x39800000 {
            /* |x| < 2**-12 */
            /* raise inexact if x != 0 */
            force_eval!(x + x1p120);
            return 1.;
        }
        return k_cosf(x);
    }
    if ix <= 0x407b53d1 {
        /* |x| ~<= 5*pi/4 */
        if ix > 0x4016cbe3 {
            /* |x|  ~> 3*pi/4 */
            return -k_cosf(if sign { x + C2_PIO2 } else { x - C2_PIO2 });
        } else if sign {
            return k_sinf(x + C1_PIO2);
        } else {
            return k_sinf(C1_PIO2 - x);
        }
    }
    if ix <= 0x40e231d5 {
        /* |x| ~<= 9*pi/4 */
        if ix > 0x40afeddf {
            /* |x| ~> 7*pi/4 */
            return k_cosf(if sign { x + C4_PIO2 } else { x - C4_PIO2 });
        } else if sign {
            return k_sinf(-x - C3_PIO2);
        } else {
            return k_sinf(x - C3_PIO2);
        }
    }

    /* cos(Inf or NaN) is NaN */
    if ix >= 0x7f800000 {
        return x - x;
    }

    /* general argument reduction needed */
    let (n, y) = rem_pio2f(x);
    match n & 3 {
        0 => k_cosf(y),
        1 => k_sinf(-y),
        2 => -k_cosf(y),
        _ => k_sinf(y),
    }
}
