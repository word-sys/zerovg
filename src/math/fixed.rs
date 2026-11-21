#![forbid(unsafe_code)]

use super::float::{clamp_finite_f32, round_ties_even_f32, round_ties_even_f64};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct Fixed24_8(pub i32);

impl Fixed24_8 {
    pub const FRAC_BITS: u32 = 8;
    pub const ONE_RAW: i32 = 1 << Self::FRAC_BITS;
    pub const HALF_RAW: i32 = 1 << (Self::FRAC_BITS - 1);
    pub const FRAC_MASK: i32 = Self::ONE_RAW - 1;

    pub const ZERO: Self = Self(0);
    pub const ONE: Self = Self(Self::ONE_RAW);
    pub const HALF: Self = Self(Self::HALF_RAW);
    pub const MIN: Self = Self(i32::MIN);
    pub const MAX: Self = Self(i32::MAX);

    const MIN_INT: i32 = -(1 << 23);
    const MAX_INT: i32 = (1 << 23) - 1;
    const CLAMP_F32: f32 = 8_388_607.0;
    const CLAMP_F64: f64 = 8_388_607.0;

    #[inline]
    pub const fn from_raw(raw: i32) -> Self {
        Self(raw)
    }

    #[inline]
    pub const fn raw(self) -> i32 {
        self.0
    }

    #[inline]
    pub const fn from_int(v: i32) -> Self {
        if v <= Self::MIN_INT {
            Self::MIN
        } else if v >= Self::MAX_INT {
            Self(Self::MAX_INT << Self::FRAC_BITS)
        } else {
            Self(v << Self::FRAC_BITS)
        }
    }

    #[inline]
    pub fn from_f32_clamped(v: f32) -> Self {
        let c = clamp_finite_f32(v, -Self::CLAMP_F32, Self::CLAMP_F32);
        Self(round_ties_even_f32(c * (Self::ONE_RAW as f32)) as i32)
    }

    #[inline]
    pub fn from_f64_clamped(v: f64) -> Self {
        let c = if !v.is_finite() {
            if v.is_nan() {
                0.0
            } else if v.is_sign_negative() {
                -Self::CLAMP_F64
            } else {
                Self::CLAMP_F64
            }
        } else {
            v.clamp(-Self::CLAMP_F64, Self::CLAMP_F64)
        };
        Self(round_ties_even_f64(c * (Self::ONE_RAW as f64)) as i32)
    }

    #[inline]
    pub const fn to_f32(self) -> f32 {
        (self.0 as f32) * (1.0 / 256.0)
    }

    #[inline]
    pub const fn floor_int(self) -> i32 {
        self.0 >> Self::FRAC_BITS
    }

    #[inline]
    pub const fn ceil_int(self) -> i32 {
        (((self.0 as i64) + (Self::FRAC_MASK as i64)) >> Self::FRAC_BITS) as i32
    }

    // Always in [0, 255]; satisfies floor_int() * 256 + frac() == raw().
    #[inline]
    pub const fn frac(self) -> i32 {
        self.0 & Self::FRAC_MASK
    }

    #[inline]
    pub const fn saturating_add(self, rhs: Self) -> Self {
        Self(self.0.saturating_add(rhs.0))
    }

    #[inline]
    pub const fn saturating_sub(self, rhs: Self) -> Self {
        Self(self.0.saturating_sub(rhs.0))
    }

    #[inline]
    pub const fn wrapping_add(self, rhs: Self) -> Self {
        Self(self.0.wrapping_add(rhs.0))
    }

    #[inline]
    pub const fn wrapping_sub(self, rhs: Self) -> Self {
        Self(self.0.wrapping_sub(rhs.0))
    }

    #[inline]
    pub const fn saturating_neg(self) -> Self {
        Self(self.0.saturating_neg())
    }

    #[inline]
    pub const fn abs_saturated(self) -> Self {
        Self(self.0.saturating_abs())
    }
}

// Computes exact round-to-nearest-ties-to-even (a * b) / denom in i64, saturated to i32.
#[inline]
pub fn mul_div_i64(a: i32, b: i32, denom: i32) -> i32 {
    debug_assert!(denom != 0);
    if denom == 0 {
        return 0;
    }
    let mut num = (a as i64) * (b as i64);
    let mut den = denom as i64;
    if den < 0 {
        num = -num;
        den = -den;
    }
    let mut q = num / den;
    let r = num % den;
    let twice_r = (r.unsigned_abs() as u128) << 1;
    let den_u = den as u128;
    if twice_r > den_u || (twice_r == den_u && (q & 1) != 0) {
        q += if num >= 0 { 1 } else { -1 };
    }
    q.clamp(i32::MIN as i64, i32::MAX as i64) as i32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fixed_negative_floor_ceil_frac() {
        let v = Fixed24_8::from_f32_clamped(-0.25);
        assert_eq!(v.raw(), -64);
        assert_eq!(v.floor_int(), -1);
        assert_eq!(v.ceil_int(), 0);
        assert_eq!(v.frac(), 192);
        assert_eq!(v.floor_int() * 256 + v.frac(), v.raw());

        for raw in [-513, -512, -257, -256, -1, 0, 1, 255, 256, 257, i32::MIN, i32::MAX] {
            let f = Fixed24_8::from_raw(raw);
            assert_eq!(f.floor_int() as i64 * 256 + f.frac() as i64, raw as i64);
            assert!(f.frac() >= 0 && f.frac() < 256);
        }
        assert_eq!(Fixed24_8::MAX.ceil_int(), 8_388_608);
        assert_eq!(Fixed24_8::MIN.floor_int(), -8_388_608);
    }

    #[test]
    fn fixed_clamping_and_specials() {
        assert_eq!(Fixed24_8::from_f32_clamped(f32::NAN), Fixed24_8::ZERO);
        assert_eq!(
            Fixed24_8::from_f32_clamped(f32::INFINITY),
            Fixed24_8::from_int(8_388_607)
        );
        assert_eq!(
            Fixed24_8::from_f32_clamped(f32::NEG_INFINITY),
            Fixed24_8::from_int(-8_388_607)
        );
        assert_eq!(Fixed24_8::from_f32_clamped(1e30), Fixed24_8::from_int(8_388_607));
        assert_eq!(Fixed24_8::from_f32_clamped(-1e30), Fixed24_8::from_int(-8_388_607));
        assert_eq!(Fixed24_8::from_f32_clamped(12.5).to_f32(), 12.5);
    }

    #[test]
    fn mul_div_exact_ties_even() {
        // 1/2 -> 0 (even), 3/2 -> 2 (even), 5/2 -> 2 (even), -3/2 -> -2
        assert_eq!(mul_div_i64(1, 1, 2), 0);
        assert_eq!(mul_div_i64(3, 1, 2), 2);
        assert_eq!(mul_div_i64(5, 1, 2), 2);
        assert_eq!(mul_div_i64(-1, 1, 2), 0);
        assert_eq!(mul_div_i64(-3, 1, 2), -2);
        assert_eq!(mul_div_i64(3, 1, -2), -2);
        // Large intermediate product without overflow
        assert_eq!(mul_div_i64(1_000_000, 8192, 1_000_000), 8192);
        assert_eq!(mul_div_i64(i32::MAX, 2, 1), i32::MAX);
        assert_eq!(mul_div_i64(i32::MIN, 2, 1), i32::MIN);
    }
}
