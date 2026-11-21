#![forbid(unsafe_code)]

use core::f64::consts::{FRAC_PI_2, FRAC_PI_4, LN_2, LOG2_E, PI, SQRT_2};

const PIO2_A: f64 = f64::from_bits(0x3FF9_21FB_5440_0000);
const PIO2_B: f64 = f64::from_bits(0x3DD0_B461_1A60_0000);
const PIO2_C: f64 = f64::from_bits(0x3BA3_198A_2E03_7073);

const LN2_HI: f64 = f64::from_bits(0x3FE6_2E42_FEE0_0000);
const LN2_LO: f64 = f64::from_bits(0x3DEA_39EF_3579_3C76);

const SQRT3: f64 = 1.732_050_807_568_877_2;
const TAN_PI_12: f64 = 0.267_949_192_431_122_7;
const FRAC_PI_6: f64 = PI / 6.0;

#[inline]
pub fn abs_f32(x: f32) -> f32 {
    f32::from_bits(x.to_bits() & 0x7FFF_FFFF)
}

#[inline]
pub fn abs_f64(x: f64) -> f64 {
    f64::from_bits(x.to_bits() & 0x7FFF_FFFF_FFFF_FFFF)
}

#[inline]
pub fn copysign_f32(x: f32, y: f32) -> f32 {
    f32::from_bits((x.to_bits() & 0x7FFF_FFFF) | (y.to_bits() & 0x8000_0000))
}

#[inline]
pub fn copysign_f64(x: f64, y: f64) -> f64 {
    f64::from_bits((x.to_bits() & 0x7FFF_FFFF_FFFF_FFFF) | (y.to_bits() & 0x8000_0000_0000_0000))
}

#[inline]
pub fn is_finite_pair(x: f32, y: f32) -> bool {
    x.is_finite() && y.is_finite()
}

#[inline]
pub fn clamp_finite_f32(x: f32, min: f32, max: f32) -> f32 {
    if !x.is_finite() {
        return if x.is_nan() {
            0.0
        } else if x.is_sign_negative() {
            min
        } else {
            max
        };
    }
    if x < min {
        min
    } else if x > max {
        max
    } else {
        x
    }
}

#[inline]
pub fn trunc_f32(x: f32) -> f32 {
    let bits = x.to_bits();
    let exp = ((bits >> 23) & 0xFF) as i32 - 127;
    if exp < 0 {
        return f32::from_bits(bits & 0x8000_0000);
    }
    if exp >= 23 {
        return x;
    }
    let mask = !((1u32 << (23 - exp)) - 1);
    f32::from_bits(bits & mask)
}

#[inline]
pub fn trunc_f64(x: f64) -> f64 {
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7FF) as i32 - 1023;
    if exp < 0 {
        return f64::from_bits(bits & 0x8000_0000_0000_0000);
    }
    if exp >= 52 {
        return x;
    }
    let mask = !((1u64 << (52 - exp)) - 1);
    f64::from_bits(bits & mask)
}

#[inline]
pub fn floor_f32(x: f32) -> f32 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    let t = trunc_f32(x);
    if x < t { t - 1.0 } else { t }
}

#[inline]
pub fn floor_f64(x: f64) -> f64 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    let t = trunc_f64(x);
    if x < t { t - 1.0 } else { t }
}

#[inline]
pub fn ceil_f32(x: f32) -> f32 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    let t = trunc_f32(x);
    if x > t { t + 1.0 } else { t }
}

#[inline]
pub fn ceil_f64(x: f64) -> f64 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    let t = trunc_f64(x);
    if x > t { t + 1.0 } else { t }
}

#[inline]
pub fn round_ties_even_f32(x: f32) -> f32 {
    let bits = x.to_bits();
    let exp = ((bits >> 23) & 0xFF) as i32 - 127;
    if exp >= 23 {
        return x;
    }
    if exp < -1 {
        return f32::from_bits(bits & 0x8000_0000);
    }
    if exp == -1 {
        return if (bits & 0x007F_FFFF) == 0 {
            f32::from_bits(bits & 0x8000_0000)
        } else {
            copysign_f32(1.0, x)
        };
    }
    let frac_bits = (23 - exp) as u32;
    let half = 1u32 << (frac_bits - 1);
    let mask = (1u32 << frac_bits) - 1;
    let frac = bits & mask;
    let lsb = (bits >> frac_bits) & 1;
    let mut rounded = bits & !mask;
    if frac > half || (frac == half && lsb == 1) {
        rounded += 1u32 << frac_bits;
    }
    f32::from_bits(rounded)
}

#[inline]
pub fn round_ties_even_f64(x: f64) -> f64 {
    let bits = x.to_bits();
    let exp = ((bits >> 52) & 0x7FF) as i32 - 1023;
    if exp >= 52 {
        return x;
    }
    if exp < -1 {
        return f64::from_bits(bits & 0x8000_0000_0000_0000);
    }
    if exp == -1 {
        return if (bits & 0x000F_FFFF_FFFF_FFFF) == 0 {
            f64::from_bits(bits & 0x8000_0000_0000_0000)
        } else {
            copysign_f64(1.0, x)
        };
    }
    let frac_bits = (52 - exp) as u32;
    let half = 1u64 << (frac_bits - 1);
    let mask = (1u64 << frac_bits) - 1;
    let frac = bits & mask;
    let lsb = (bits >> frac_bits) & 1;
    let mut rounded = bits & !mask;
    if frac > half || (frac == half && lsb == 1) {
        rounded += 1u64 << frac_bits;
    }
    f64::from_bits(rounded)
}

#[inline]
pub fn fract_f32(x: f32) -> f32 {
    x - floor_f32(x)
}

// Exact bit-by-bit restoring sqrt (0 ULP IEEE-754 correctly rounded).
pub fn sqrt_f32(x: f32) -> f32 {
    if x.is_nan() || x == 0.0 {
        return x;
    }
    if x < 0.0 {
        return f32::NAN;
    }
    if x.is_infinite() {
        return x;
    }

    let bits = x.to_bits();
    let mut exp = ((bits >> 23) & 0xFF) as i32;
    let mut mant = bits & 0x007F_FFFF;

    if exp == 0 {
        let shift = mant.leading_zeros() as i32 - 8;
        mant <<= shift;
        exp = 1 - shift;
    } else {
        mant |= 1u32 << 23;
    }
    exp -= 127;

    if (exp & 1) != 0 {
        mant <<= 1;
        exp -= 1;
    }
    let res_exp = ((exp >> 1) + 127) as u32;

    let mut m = mant << 1;
    let mut q = 0u32;
    let mut s = 0u32;
    let mut r = 1u32 << 24;
    while r != 0 {
        let t = s + r;
        if t <= m {
            s = t + r;
            m -= t;
            q += r;
        }
        m <<= 1;
        r >>= 1;
    }

    let mut out_mant = q >> 1;
    if (q & 1) != 0 && (m != 0 || (out_mant & 1) != 0) {
        out_mant += 1;
    }
    f32::from_bits(((res_exp - 1) << 23) + out_mant)
}

// Exact bit-by-bit restoring sqrt (0 ULP IEEE-754 correctly rounded).
pub fn sqrt_f64(x: f64) -> f64 {
    if x.is_nan() || x == 0.0 {
        return x;
    }
    if x < 0.0 {
        return f64::NAN;
    }
    if x.is_infinite() {
        return x;
    }

    let bits = x.to_bits();
    let mut exp = ((bits >> 52) & 0x7FF) as i32;
    let mut mant = bits & 0x000F_FFFF_FFFF_FFFF;

    if exp == 0 {
        let shift = mant.leading_zeros() as i32 - 11;
        mant <<= shift;
        exp = 1 - shift;
    } else {
        mant |= 1u64 << 52;
    }
    exp -= 1023;

    if (exp & 1) != 0 {
        mant <<= 1;
        exp -= 1;
    }
    let res_exp = ((exp >> 1) + 1023) as u64;

    let mut m = mant << 1;
    let mut q = 0u64;
    let mut s = 0u64;
    let mut r = 1u64 << 53;
    while r != 0 {
        let t = s + r;
        if t <= m {
            s = t + r;
            m -= t;
            q += r;
        }
        m <<= 1;
        r >>= 1;
    }

    let mut out_mant = q >> 1;
    if (q & 1) != 0 && (m != 0 || (out_mant & 1) != 0) {
        out_mant += 1;
    }
    f64::from_bits(((res_exp - 1) << 52) + out_mant)
}

pub fn cbrt_f64(x: f64) -> f64 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    let neg = x < 0.0;
    let mut a = abs_f64(x);
    let mut bits = a.to_bits();
    let mut exp = ((bits >> 52) & 0x7FF) as i32;

    if exp == 0 {
        a *= f64::from_bits((1023u64 + 54) << 52);
        bits = a.to_bits();
        exp = ((bits >> 52) & 0x7FF) as i32 - 54;
    }
    let e = exp - 1023;
    let q = e.div_euclid(3);
    let rem = e.rem_euclid(3);

    // m in [1, 8)
    let m = f64::from_bits((bits & 0x000F_FFFF_FFFF_FFFF) | (((1023 + rem) as u64) << 52));

    // Cubic-converging Halley iterations from piecewise linear seed on [1, 8).
    let mut y = if m < 2.0 {
        0.74 + 0.26 * m
    } else if m < 4.0 {
        0.92 + 0.165 * m
    } else {
        1.18 + 0.103 * m
    };
    for _ in 0..4 {
        let y3 = y * y * y;
        y += y * (m - y3) / (2.0 * y3 + m);
    }

    let scale = f64::from_bits(((q + 1023) as u64) << 52);
    let res = y * scale;
    if neg { -res } else { res }
}

#[inline]
pub fn cbrt_f32(x: f32) -> f32 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    cbrt_f64(x as f64) as f32
}

#[inline]
fn sin_poly_f64(r: f64) -> f64 {
    let r2 = r * r;
    const S1: f64 = -1.666_666_666_666_663_2e-1;
    const S2: f64 = 8.333_333_333_322_49e-3;
    const S3: f64 = -1.984_126_982_985_795e-4;
    const S4: f64 = 2.755_731_370_707_006_8e-6;
    const S5: f64 = -2.505_076_025_340_686_3e-8;
    const S6: f64 = 1.589_690_995_211_55e-10;
    let p = S1 + r2 * (S2 + r2 * (S3 + r2 * (S4 + r2 * (S5 + r2 * S6))));
    r + r * r2 * p
}

#[inline]
fn cos_poly_f64(r: f64) -> f64 {
    let r2 = r * r;
    const C1: f64 = 4.166_666_666_666_66e-2;
    const C2: f64 = -1.388_888_888_887_411e-3;
    const C3: f64 = 2.480_158_728_947_673e-5;
    const C4: f64 = -2.755_731_435_139_066_3e-7;
    const C5: f64 = 2.087_572_321_298_175e-9;
    const C6: f64 = -1.135_964_755_778_819_5e-11;
    let p = C1 + r2 * (C2 + r2 * (C3 + r2 * (C4 + r2 * (C5 + r2 * C6))));
    1.0 - 0.5 * r2 + r2 * r2 * p
}

pub fn sincos_f32(x: f32) -> (f32, f32) {
    if !x.is_finite() {
        return (f32::NAN, f32::NAN);
    }
    if x == 0.0 {
        return (x, 1.0);
    }

    let mut xd = x as f64;
    if abs_f64(xd) > 1_500_000.0 {
        // Bring extreme angles safely into [-2*PI, 2*PI] so f64 quadrant reduction never overflows.
        const TAU: f64 = 2.0 * PI;
        let sign = if xd < 0.0 { -1.0 } else { 1.0 };
        let mut a = abs_f64(xd);
        let mut step = TAU;
        while step * 2.0 <= a {
            step *= 2.0;
        }
        while a >= TAU {
            if a >= step {
                a -= step;
            }
            step *= 0.5;
        }
        xd = sign * a;
    }

    let k_f = round_ties_even_f64(xd * (2.0 / PI));
    let k = k_f as i64;
    let r = ((xd - k_f * PIO2_A) - k_f * PIO2_B) - k_f * PIO2_C;

    let s = sin_poly_f64(r);
    let c = cos_poly_f64(r);

    let (sn, cs) = match k.rem_euclid(4) {
        0 => (s, c),
        1 => (c, -s),
        2 => (-s, -c),
        _ => (-c, s),
    };
    (sn as f32, cs as f32)
}

#[inline]
pub fn sin_f32(x: f32) -> f32 {
    sincos_f32(x).0
}

#[inline]
pub fn cos_f32(x: f32) -> f32 {
    sincos_f32(x).1
}

// Evaluates atan(u) for u in [0, 1] to ~48 bits in f64.
fn atan_unit_f64(u: f64) -> f64 {
    let (z, offset) = if u > TAN_PI_12 {
        ((SQRT3 * u - 1.0) / (SQRT3 + u), FRAC_PI_6)
    } else {
        (u, 0.0)
    };
    let z2 = z * z;
    let p = 1.0
        + z2 * (-1.0 / 3.0
            + z2 * (1.0 / 5.0
                + z2 * (-1.0 / 7.0
                    + z2 * (1.0 / 9.0
                        + z2 * (-1.0 / 11.0
                            + z2 * (1.0 / 13.0
                                + z2 * (-1.0 / 15.0
                                    + z2 * (1.0 / 17.0
                                        + z2 * (-1.0 / 19.0
                                            + z2 * (1.0 / 21.0 + z2 * (-1.0 / 23.0)))))))))));
    offset + z * p
}

pub fn atan2_f32(y: f32, x: f32) -> f32 {
    if y.is_nan() || x.is_nan() {
        return f32::NAN;
    }
    let y_neg = y.is_sign_negative();
    let x_neg = x.is_sign_negative();

    if y == 0.0 {
        return if x_neg {
            copysign_f32(PI as f32, y)
        } else {
            y
        };
    }
    if x == 0.0 {
        return copysign_f32(FRAC_PI_2 as f32, y);
    }
    if y.is_infinite() {
        let angle = if x.is_infinite() {
            if x_neg { 3.0 * FRAC_PI_4 } else { FRAC_PI_4 }
        } else {
            FRAC_PI_2
        };
        return copysign_f32(angle as f32, y);
    }
    if x.is_infinite() {
        let angle = if x_neg { PI } else { 0.0 };
        return copysign_f32(angle as f32, y);
    }

    let ay = abs_f64(y as f64);
    let ax = abs_f64(x as f64);
    let mut a = if ay <= ax {
        atan_unit_f64(ay / ax)
    } else {
        FRAC_PI_2 - atan_unit_f64(ax / ay)
    };
    if x_neg {
        a = PI - a;
    }
    let res = a as f32;
    if y_neg { -res } else { res }
}

pub fn acos_f32(x: f32) -> f32 {
    if !(-1.0..=1.0).contains(&x) {
        return f32::NAN;
    }
    if x == 1.0 {
        return 0.0;
    }
    if x == -1.0 {
        return PI as f32;
    }
    let xd = x as f64;
    let a = sqrt_f64(1.0 - xd);
    let b = sqrt_f64(1.0 + xd);
    let ang = if a <= b {
        atan_unit_f64(a / b)
    } else {
        FRAC_PI_2 - atan_unit_f64(b / a)
    };
    (2.0 * ang) as f32
}

#[inline]
fn exp_poly_f64(u: f64) -> f64 {
    1.0 + u
        * (1.0
            + u * (1.0 / 2.0
                + u * (1.0 / 6.0
                    + u * (1.0 / 24.0
                        + u * (1.0 / 120.0
                            + u * (1.0 / 720.0
                                + u * (1.0 / 5_040.0
                                    + u * (1.0 / 40_320.0
                                        + u * (1.0 / 362_880.0
                                            + u * (1.0 / 3_628_800.0
                                                + u * (1.0 / 39_916_800.0
                                                    + u * (1.0 / 479_001_600.0))))))))))))
}

fn exp2_f64_core(x: f64) -> f64 {
    if x >= 128.0 {
        return f64::INFINITY;
    }
    if x <= -150.0 {
        return 0.0;
    }
    let k_f = round_ties_even_f64(x);
    let k = k_f as i32;
    let u = (x - k_f) * LN_2;
    let scale = f64::from_bits(((k + 1023) as u64) << 52);
    exp_poly_f64(u) * scale
}

pub fn exp2_f32(x: f32) -> f32 {
    if x.is_nan() {
        return f32::NAN;
    }
    exp2_f64_core(x as f64) as f32
}

pub fn exp_f32(x: f32) -> f32 {
    if x.is_nan() {
        return f32::NAN;
    }
    if x >= 88.722_84 {
        return f32::INFINITY;
    }
    if x <= -104.0 {
        return 0.0;
    }
    let xd = x as f64;
    let k_f = round_ties_even_f64(xd * LOG2_E);
    let k = k_f as i32;
    let u = (xd - k_f * LN2_HI) - k_f * LN2_LO;
    let scale = f64::from_bits(((k + 1023) as u64) << 52);
    (exp_poly_f64(u) * scale) as f32
}

// Returns (e, ln(m)) where x = m * 2^e and m in [sqrt(0.5), sqrt(2)].
fn ln_decompose_f64(x: f64) -> (i32, f64) {
    let mut a = x;
    let mut bits = a.to_bits();
    let mut exp = ((bits >> 52) & 0x7FF) as i32;
    if exp == 0 {
        a *= f64::from_bits((1023u64 + 54) << 52);
        bits = a.to_bits();
        exp = ((bits >> 52) & 0x7FF) as i32 - 54;
    }
    let mut e = exp - 1023;
    let mut m = f64::from_bits((bits & 0x000F_FFFF_FFFF_FFFF) | (1023u64 << 52));
    if m > SQRT_2 {
        m *= 0.5;
        e += 1;
    }
    let t = (m - 1.0) / (m + 1.0);
    let t2 = t * t;
    let p = 1.0
        + t2 * (1.0 / 3.0
            + t2 * (1.0 / 5.0
                + t2 * (1.0 / 7.0
                    + t2 * (1.0 / 9.0
                        + t2 * (1.0 / 11.0
                            + t2 * (1.0 / 13.0
                                + t2 * (1.0 / 15.0
                                    + t2 * (1.0 / 17.0 + t2 * (1.0 / 19.0)))))))));
    (e, 2.0 * t * p)
}

pub fn ln_f32(x: f32) -> f32 {
    if x.is_nan() || x < 0.0 {
        return f32::NAN;
    }
    if x == 0.0 {
        return f32::NEG_INFINITY;
    }
    if x.is_infinite() {
        return f32::INFINITY;
    }
    let (e, ln_m) = ln_decompose_f64(x as f64);
    ((e as f64) * LN_2 + ln_m) as f32
}

pub fn log2_f32(x: f32) -> f32 {
    if x.is_nan() || x < 0.0 {
        return f32::NAN;
    }
    if x == 0.0 {
        return f32::NEG_INFINITY;
    }
    if x.is_infinite() {
        return f32::INFINITY;
    }
    let (e, ln_m) = ln_decompose_f64(x as f64);
    ((e as f64) + ln_m * LOG2_E) as f32
}

fn is_odd_integer_f32(y: f32) -> bool {
    if !y.is_finite() || y != trunc_f32(y) {
        return false;
    }
    let ay = abs_f32(y);
    if ay >= 16_777_216.0 {
        return false;
    }
    ((ay as u32) & 1) == 1
}

pub fn powf_f32(x: f32, y: f32) -> f32 {
    if y == 0.0 || x == 1.0 {
        return 1.0;
    }
    if x.is_nan() || y.is_nan() {
        return f32::NAN;
    }
    if y.is_infinite() {
        let ax = abs_f32(x);
        if ax == 1.0 {
            return 1.0;
        }
        return if (ax > 1.0) == y.is_sign_positive() {
            f32::INFINITY
        } else {
            0.0
        };
    }
    if x == 0.0 {
        let odd = is_odd_integer_f32(y);
        let mag = if y < 0.0 { f32::INFINITY } else { 0.0 };
        return if odd && x.is_sign_negative() { -mag } else { mag };
    }
    if x.is_infinite() {
        let odd = is_odd_integer_f32(y);
        let mag = if y > 0.0 { f32::INFINITY } else { 0.0 };
        return if odd && x.is_sign_negative() { -mag } else { mag };
    }

    let neg_res = if x < 0.0 {
        if y != trunc_f32(y) {
            return f32::NAN;
        }
        is_odd_integer_f32(y)
    } else {
        false
    };

    let (e, ln_m) = ln_decompose_f64(abs_f64(x as f64));
    let log2_x = (e as f64) + ln_m * LOG2_E;
    let mag = exp2_f64_core((y as f64) * log2_x) as f32;
    if neg_res { -mag } else { mag }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "std")]
    fn ulp_diff_f32(a: f32, b: f32) -> u32 {
        if a.is_nan() && b.is_nan() {
            return 0;
        }
        if a == 0.0 && b == 0.0 {
            return 0;
        }
        if a.is_infinite() && b.is_infinite() && a.is_sign_positive() == b.is_sign_positive() {
            return 0;
        }
        let ai = a.to_bits() as i32;
        let bi = b.to_bits() as i32;
        let am = if ai < 0 { i32::MIN - ai } else { ai };
        let bm = if bi < 0 { i32::MIN - bi } else { bi };
        am.abs_diff(bm)
    }

    #[cfg(feature = "std")]
    fn ulp_diff_f64(a: f64, b: f64) -> u64 {
        if a.is_nan() && b.is_nan() {
            return 0;
        }
        if a == 0.0 && b == 0.0 {
            return 0;
        }
        if a.is_infinite() && b.is_infinite() && a.is_sign_positive() == b.is_sign_positive() {
            return 0;
        }
        let ai = a.to_bits() as i64;
        let bi = b.to_bits() as i64;
        let am = if ai < 0 { i64::MIN - ai } else { ai };
        let bm = if bi < 0 { i64::MIN - bi } else { bi };
        am.abs_diff(bm)
    }

    #[cfg(feature = "std")]
    fn lcg(state: &mut u64) -> u64 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *state
    }

    #[test]
    fn rounding_and_bit_ops() {
        assert_eq!(abs_f32(-0.0).to_bits(), 0.0f32.to_bits());
        assert_eq!(copysign_f32(2.5, -0.0), -2.5);
        assert_eq!(trunc_f32(-0.5).to_bits(), (-0.0f32).to_bits());
        assert_eq!(floor_f32(-0.5), -1.0);
        assert_eq!(floor_f32(-0.0).to_bits(), (-0.0f32).to_bits());
        assert_eq!(ceil_f32(-0.5).to_bits(), (-0.0f32).to_bits());
        assert_eq!(ceil_f32(0.1), 1.0);

        assert_eq!(round_ties_even_f32(0.5), 0.0);
        assert_eq!(round_ties_even_f32(1.5), 2.0);
        assert_eq!(round_ties_even_f32(2.5), 2.0);
        assert_eq!(round_ties_even_f32(3.5), 4.0);
        assert_eq!(round_ties_even_f32(-0.5).to_bits(), (-0.0f32).to_bits());
        assert_eq!(round_ties_even_f32(-1.5), -2.0);
        assert_eq!(round_ties_even_f32(-2.5), -2.0);

        assert_eq!(round_ties_even_f64(0.5), 0.0);
        assert_eq!(round_ties_even_f64(1.5), 2.0);
        assert_eq!(round_ties_even_f64(2.5), 2.0);
        assert_eq!(round_ties_even_f64(-2.5), -2.0);

        assert_eq!(clamp_finite_f32(f32::NAN, -1.0, 1.0), 0.0);
        assert_eq!(clamp_finite_f32(f32::INFINITY, -1.0, 1.0), 1.0);
        assert_eq!(clamp_finite_f32(f32::NEG_INFINITY, -1.0, 1.0), -1.0);
        assert!(is_finite_pair(1.0, -2.0));
        assert!(!is_finite_pair(1.0, f32::NAN));
    }

    #[cfg(feature = "std")]
    #[test]
    fn sqrt_zero_ulp_exact() {
        let specials = [
            0.0f32,
            -0.0,
            f32::MIN_POSITIVE,
            f32::from_bits(1),
            f32::from_bits(0x007F_FFFF),
            0.25,
            0.5,
            1.0,
            2.0,
            3.0,
            4.0,
            9.0,
            1e-30,
            1e30,
            f32::MAX,
            f32::INFINITY,
            -1.0,
            f32::NAN,
        ];
        for &x in &specials {
            let got = sqrt_f32(x);
            let exp = x.sqrt();
            if x == 0.0 {
                assert_eq!(got.to_bits(), exp.to_bits());
            } else {
                assert_eq!(ulp_diff_f32(got, exp), 0, "sqrt_f32({x:?})");
            }
        }

        let mut rng = 0x1234_5678_9ABC_DEF0u64;
        for _ in 0..50_000 {
            let bits = (lcg(&mut rng) >> 32) as u32 & 0x7FFF_FFFF;
            let x = f32::from_bits(bits);
            assert_eq!(ulp_diff_f32(sqrt_f32(x), x.sqrt()), 0, "bits={bits:#x}");
        }

        let specials64 = [
            0.0f64,
            -0.0,
            f64::MIN_POSITIVE,
            f64::from_bits(1),
            f64::from_bits(0x000F_FFFF_FFFF_FFFF),
            0.5,
            1.0,
            2.0,
            3.0,
            1e-200,
            1e200,
            f64::MAX,
            f64::INFINITY,
            -2.0,
            f64::NAN,
        ];
        for &x in &specials64 {
            let got = sqrt_f64(x);
            let exp = x.sqrt();
            if x == 0.0 {
                assert_eq!(got.to_bits(), exp.to_bits());
            } else {
                assert_eq!(ulp_diff_f64(got, exp), 0, "sqrt_f64({x:?})");
            }
        }
        for _ in 0..50_000 {
            let bits = lcg(&mut rng) & 0x7FFF_FFFF_FFFF_FFFF;
            let x = f64::from_bits(bits);
            assert_eq!(ulp_diff_f64(sqrt_f64(x), x.sqrt()), 0, "bits64={bits:#x}");
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn cbrt_within_one_ulp() {
        let mut rng = 0xCAFE_BABE_1111_2222u64;
        for &x in &[
            0.0f32,
            -0.0,
            1.0,
            -1.0,
            8.0,
            -27.0,
            f32::from_bits(1),
            f32::MAX,
            -f32::MAX,
            f32::INFINITY,
            f32::NEG_INFINITY,
            f32::NAN,
        ] {
            assert!(ulp_diff_f32(cbrt_f32(x), x.cbrt()) <= 1, "cbrt_f32({x:?})");
        }
        for _ in 0..20_000 {
            let bits = (lcg(&mut rng) >> 32) as u32;
            let x = f32::from_bits(bits);
            assert!(
                ulp_diff_f32(cbrt_f32(x), x.cbrt()) <= 1,
                "cbrt_f32({x:?})"
            );
        }
        for _ in 0..20_000 {
            let bits = lcg(&mut rng);
            let x = f64::from_bits(bits);
            if x.is_finite() {
                assert!(
                    ulp_diff_f64(cbrt_f64(x), x.cbrt()) <= 1,
                    "cbrt_f64({x:?}) got={} exp={}",
                    cbrt_f64(x),
                    x.cbrt()
                );
            }
        }
    }

    #[cfg(feature = "std")]
    #[test]
    fn trig_and_inverse_within_one_ulp() {
        let mut rng = 0xDEAD_BEEF_4242_1337u64;
        for &x in &[
            0.0f32,
            -0.0,
            1e-7,
            -1e-7,
            core::f32::consts::FRAC_PI_4,
            core::f32::consts::FRAC_PI_2,
            core::f32::consts::PI,
            -core::f32::consts::PI,
            1000.0,
            -1_000_000.0,
            1_000_000.0,
            f32::INFINITY,
            f32::NAN,
        ] {
            let (s, c) = sincos_f32(x);
            assert!(ulp_diff_f32(s, x.sin()) <= 1, "sin({x})");
            assert!(ulp_diff_f32(c, x.cos()) <= 1, "cos({x})");
        }
        for i in -10_000..=10_000 {
            let x = (i as f32) * 0.0137;
            let (s, c) = sincos_f32(x);
            assert!(ulp_diff_f32(s, x.sin()) <= 1, "sin({x})");
            assert!(ulp_diff_f32(c, x.cos()) <= 1, "cos({x})");
        }
        // Extreme angles stay finite and unit-length (s^2 + c^2 ~= 1).
        for &x in &[1e10f32, -1e10, 1e25, -1e25, f32::MAX, -f32::MAX] {
            let (s, c) = sincos_f32(x);
            assert!(s.is_finite() && c.is_finite(), "sincos({x})");
            assert!(abs_f32(s * s + c * c - 1.0) < 1e-5, "unit({x})");
        }

        // atan2 across all quadrants and special values
        let vals = [0.0f32, -0.0, 1.0, -1.0, 0.25, -3.5, f32::INFINITY, f32::NEG_INFINITY, f32::NAN];
        for &y in &vals {
            for &x in &vals {
                let got = atan2_f32(y, x);
                let exp = y.atan2(x);
                assert!(
                    ulp_diff_f32(got, exp) <= 1,
                    "atan2({y:?}, {x:?}) got={got:?} exp={exp:?}"
                );
            }
        }
        for _ in 0..20_000 {
            let y = (((lcg(&mut rng) >> 40) as f32) / 8_388_608.0 - 1.0) * 100.0;
            let x = (((lcg(&mut rng) >> 40) as f32) / 8_388_608.0 - 1.0) * 100.0;
            assert!(ulp_diff_f32(atan2_f32(y, x), y.atan2(x)) <= 1);
        }

        // acos across [-1, 1] and out of range
        for i in -2000..=2000 {
            let x = (i as f32) / 2000.0;
            assert!(ulp_diff_f32(acos_f32(x), x.acos()) <= 1, "acos({x})");
        }
        assert!(acos_f32(1.0001).is_nan());
        assert!(acos_f32(-1.0001).is_nan());
    }

    #[cfg(feature = "std")]
    #[test]
    fn exp_log_pow_within_one_ulp() {
        for i in -1500..=1500 {
            let x = (i as f32) * 0.1;
            assert!(ulp_diff_f32(exp2_f32(x), x.exp2()) <= 1, "exp2({x})");
            assert!(ulp_diff_f32(exp_f32(x), x.exp()) <= 1, "exp({x})");
        }

        let mut rng = 0x5555_AAAA_3333_CCCCu64;
        for _ in 0..20_000 {
            let bits = (lcg(&mut rng) >> 32) as u32 & 0x7FFF_FFFF;
            let x = f32::from_bits(bits);
            assert!(ulp_diff_f32(ln_f32(x), x.ln()) <= 1, "ln({x:?})");
            assert!(ulp_diff_f32(log2_f32(x), x.log2()) <= 1, "log2({x:?})");
        }

        // Exact powers of 2 and subnormal edge
        for e in -149..=127 {
            let x = exp2_f32(e as f32);
            assert_eq!(ulp_diff_f32(x, (e as f32).exp2()), 0, "exp2({e})");
            if e >= -126 {
                assert_eq!(log2_f32(x), e as f32);
            }
        }

        // powf including sRGB gamma exponents and negative integer powers
        let bases = [0.0f32, -0.0, 0.0031308, 0.5, 1.0, 2.0, -2.0, 10.0, f32::INFINITY, -f32::INFINITY];
        let exps = [-3.0f32, -2.0, -0.5, 0.0, 1.0 / 2.4, 0.5, 1.0, 2.0, 2.4, 3.0, f32::INFINITY, f32::NEG_INFINITY];
        for &b in &bases {
            for &e in &exps {
                let got = powf_f32(b, e);
                let exp = b.powf(e);
                assert!(
                    ulp_diff_f32(got, exp) <= 1,
                    "powf({b:?}, {e:?}) got={got:?} exp={exp:?}"
                );
            }
        }
    }

    #[test]
    fn deterministic_bit_checksum() {
        let mut hash = 0xCBF2_9CE4_8422_2325u64;
        let mix = |h: &mut u64, bits: u32| {
            *h ^= bits as u64;
            *h = h.wrapping_mul(0x0000_0100_0000_01B3);
        };
        for i in 1..=2_000i32 {
            let x = (i as f32) * 0.0173 - 17.3;
            let ax = abs_f32(x) + 1e-4;
            let (s, c) = sincos_f32(x);
            mix(&mut hash, sqrt_f32(ax).to_bits());
            mix(&mut hash, cbrt_f32(x).to_bits());
            mix(&mut hash, s.to_bits());
            mix(&mut hash, c.to_bits());
            mix(&mut hash, atan2_f32(x, ax).to_bits());
            mix(&mut hash, acos_f32(clamp_finite_f32(x * 0.05, -1.0, 1.0)).to_bits());
            mix(&mut hash, exp_f32(x * 0.25).to_bits());
            mix(&mut hash, ln_f32(ax).to_bits());
            mix(&mut hash, powf_f32(ax, 2.4).to_bits());
        }
        // Pinned IEEE-754 bit checksum across 18,000 outputs.
        assert_eq!(hash, 0xfb48_4840_5611_6aef);
    }
}
