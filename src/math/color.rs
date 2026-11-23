#![forbid(unsafe_code)]

use core::f32::consts::TAU;

use super::float::{
    abs_f32, atan2_f32, cbrt_f32, clamp_finite_f32, powf_f32, round_ties_even_f32, sincos_f32,
    sqrt_f32,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum ColorSpace {
    #[default]
    Srgb,
    LinearSrgb,
    DisplayP3,
    LinearDisplayP3,
}

// Exact normalized u16 multiply: round(a * b / 65535) for all 65536^2 pairs.
#[inline]
pub const fn mul_u16(a: u16, b: u16) -> u16 {
    let t = (a as u32) * (b as u32) + 0x8000;
    ((t + (t >> 16)) >> 16) as u16
}

#[inline]
pub const fn u8_to_u16_linear(v: u8) -> u16 {
    (v as u16) * 257
}

#[inline]
pub const fn u16_to_u8_linear(v: u16) -> u8 {
    (((v as u32) * 255 + 32768) / 65535) as u8
}

#[inline]
pub const fn unpremultiply_u16(c: u16, a: u16) -> u16 {
    if a == 0 {
        return 0;
    }
    if c >= a {
        return 65535;
    }
    (((c as u32) * 65535 + ((a as u32) >> 1)) / (a as u32)) as u16
}

// Compile-time f64 exp/ln helpers used solely to populate exact .rodata sRGB LUTs.
const fn const_exp_f64(x: f64) -> f64 {
    const LN_2: f64 = core::f64::consts::LN_2;
    const LOG2_E: f64 = core::f64::consts::LOG2_E;
    let k = (x * LOG2_E + if x >= 0.0 { 0.5 } else { -0.5 }) as i32;
    let u = x - (k as f64) * LN_2;
    let mut term = 1.0;
    let mut sum = 1.0;
    let mut i = 1;
    while i <= 14 {
        term = term * u / (i as f64);
        sum += term;
        i += 1;
    }
    let scale = f64::from_bits(((k + 1023) as u64) << 52);
    sum * scale
}

const fn const_ln_f64(x: f64) -> f64 {
    const LN_2: f64 = core::f64::consts::LN_2;
    const SQRT_2: f64 = core::f64::consts::SQRT_2;
    let bits = x.to_bits();
    let mut e = (((bits >> 52) & 0x7FF) as i32) - 1023;
    let mut m = f64::from_bits((bits & 0x000F_FFFF_FFFF_FFFF) | (1023u64 << 52));
    if m > SQRT_2 {
        m *= 0.5;
        e += 1;
    }
    let t = (m - 1.0) / (m + 1.0);
    let t2 = t * t;
    let mut pow = t;
    let mut sum = t;
    let mut k = 3;
    while k <= 21 {
        pow *= t2;
        sum += pow / (k as f64);
        k += 2;
    }
    (e as f64) * LN_2 + 2.0 * sum
}

const fn const_pow_f64(x: f64, y: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    const_exp_f64(y * const_ln_f64(x))
}

const fn build_srgb_to_linear_u16() -> [u16; 256] {
    let mut lut = [0u16; 256];
    let mut i = 0usize;
    while i < 256 {
        let s = (i as f64) / 255.0;
        let lin = if s <= 0.04045 {
            s / 12.92
        } else {
            const_pow_f64((s + 0.055) / 1.055, 2.4)
        };
        let v = (lin * 65535.0 + 0.5) as i32;
        lut[i] = if v < 0 {
            0
        } else if v > 65535 {
            65535
        } else {
            v as u16
        };
        i += 1;
    }
    lut
}

// 4096-interval LUT (+ sentinel) storing 8.8 fixed-point sRGB [0..=65280].
const fn build_linear_to_srgb_8_8() -> [u16; 4097] {
    let mut lut = [0u16; 4097];
    let mut i = 0usize;
    while i <= 4096 {
        let lin = if i == 4096 {
            1.0
        } else {
            ((i * 16) as f64) / 65535.0
        };
        let s = if lin <= 0.0031308 {
            lin * 12.92
        } else {
            1.055 * const_pow_f64(lin, 1.0 / 2.4) - 0.055
        };
        let v = (s * 255.0 * 256.0 + 0.5) as i32;
        lut[i] = if v < 0 {
            0
        } else if v > 65280 {
            65280
        } else {
            v as u16
        };
        i += 1;
    }
    lut
}

pub static SRGB_TO_LINEAR_U16: [u16; 256] = build_srgb_to_linear_u16();
pub static LINEAR_TO_SRGB_8_8: [u16; 4097] = build_linear_to_srgb_8_8();

#[inline]
pub fn srgb_u8_to_linear_u16(v: u8) -> u16 {
    SRGB_TO_LINEAR_U16[v as usize]
}

// Piecewise-linear LUT lookup returning 8.8 fixed-point sRGB in [0..=65280].
#[inline]
pub fn linear_u16_to_srgb_8_8(v: u16) -> u16 {
    let idx = (v >> 4) as usize;
    let frac = (v & 0xF) as u32;
    let lo = LINEAR_TO_SRGB_8_8[idx] as u32;
    let hi = LINEAR_TO_SRGB_8_8[idx + 1] as u32;
    (lo + (((hi - lo) * frac + 8) >> 4)) as u16
}

#[inline]
pub fn linear_u16_to_srgb_u8(v: u16) -> u8 {
    let s88 = linear_u16_to_srgb_8_8(v) as u32;
    ((s88 + 128) >> 8).min(255) as u8
}

// Sign-preserving extended sRGB transfer for HDR / wide-gamut workflows.
#[inline]
pub fn srgb_to_linear_f32(c: f32) -> f32 {
    let a = abs_f32(c);
    let lin = if a <= 0.04045 {
        a / 12.92
    } else {
        powf_f32((a + 0.055) / 1.055, 2.4)
    };
    if c < 0.0 { -lin } else { lin }
}

#[inline]
pub fn linear_to_srgb_f32(c: f32) -> f32 {
    let a = abs_f32(c);
    let s = if a <= 0.0031308 {
        a * 12.92
    } else {
        1.055 * powf_f32(a, 1.0 / 2.4) - 0.055
    };
    if c < 0.0 { -s } else { s }
}

#[inline]
pub fn linear_srgb_to_display_p3(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    (
        0.822_461_96 * r + 0.177_538_04 * g,
        0.033_194_2 * r + 0.966_805_8 * g,
        0.017_082_63 * r + 0.072_397_44 * g + 0.910_519_9 * b,
    )
}

#[inline]
pub fn linear_display_p3_to_srgb(r: f32, g: f32, b: f32) -> (f32, f32, f32) {
    (
        1.224_940_2 * r - 0.224_940_18 * g,
        -0.042_056_955 * r + 1.042_057 * g,
        -0.019_637_555 * r - 0.078_636_05 * g + 1.098_273_6 * b,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Color {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Color {
    pub const TRANSPARENT: Self = Self::new(0.0, 0.0, 0.0, 0.0);
    pub const BLACK: Self = Self::new(0.0, 0.0, 0.0, 1.0);
    pub const WHITE: Self = Self::new(1.0, 1.0, 1.0, 1.0);
    pub const RED: Self = Self::new(1.0, 0.0, 0.0, 1.0);
    pub const GREEN: Self = Self::new(0.0, 1.0, 0.0, 1.0);
    pub const BLUE: Self = Self::new(0.0, 0.0, 1.0, 1.0);

    #[inline]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    #[inline]
    pub const fn from_rgb(r: f32, g: f32, b: f32) -> Self {
        Self { r, g, b, a: 1.0 }
    }

    #[inline]
    pub fn from_rgba8(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self {
            r: (r as f32) * (1.0 / 255.0),
            g: (g as f32) * (1.0 / 255.0),
            b: (b as f32) * (1.0 / 255.0),
            a: (a as f32) * (1.0 / 255.0),
        }
    }

    #[inline]
    pub fn from_rgb8(r: u8, g: u8, b: u8) -> Self {
        Self::from_rgba8(r, g, b, 255)
    }

    #[inline]
    pub fn from_hex(rgb: u32) -> Self {
        Self::from_rgb8(((rgb >> 16) & 0xFF) as u8, ((rgb >> 8) & 0xFF) as u8, (rgb & 0xFF) as u8)
    }

    #[inline]
    pub fn from_hex_rgba(rgba: u32) -> Self {
        Self::from_rgba8(
            ((rgba >> 24) & 0xFF) as u8,
            ((rgba >> 16) & 0xFF) as u8,
            ((rgba >> 8) & 0xFF) as u8,
            (rgba & 0xFF) as u8,
        )
    }

    #[inline]
    pub const fn with_alpha(self, a: f32) -> Self {
        Self { a, ..self }
    }

    #[inline]
    pub fn from_linear_rgba(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self {
            r: linear_to_srgb_f32(r),
            g: linear_to_srgb_f32(g),
            b: linear_to_srgb_f32(b),
            a,
        }
    }

    #[inline]
    pub fn from_display_p3(r: f32, g: f32, b: f32, a: f32) -> Self {
        let lr = srgb_to_linear_f32(r);
        let lg = srgb_to_linear_f32(g);
        let lb = srgb_to_linear_f32(b);
        let (sr, sg, sb) = linear_display_p3_to_srgb(lr, lg, lb);
        Self::from_linear_rgba(sr, sg, sb, a)
    }

    #[inline]
    pub fn to_linear_f32(self) -> LinearRgbaF32 {
        let a = clamp_finite_f32(self.a, 0.0, 1.0);
        let lr = srgb_to_linear_f32(clamp_finite_f32(self.r, -8.0, 8.0));
        let lg = srgb_to_linear_f32(clamp_finite_f32(self.g, -8.0, 8.0));
        let lb = srgb_to_linear_f32(clamp_finite_f32(self.b, -8.0, 8.0));
        LinearRgbaF32::new(lr * a, lg * a, lb * a, a)
    }

    #[inline]
    pub fn to_linear_rgba16(self) -> LinearRgba16 {
        self.to_linear_f32().to_linear_rgba16()
    }

    #[inline]
    pub fn to_oklab(self) -> Oklab {
        let lr = srgb_to_linear_f32(clamp_finite_f32(self.r, 0.0, 1.0));
        let lg = srgb_to_linear_f32(clamp_finite_f32(self.g, 0.0, 1.0));
        let lb = srgb_to_linear_f32(clamp_finite_f32(self.b, 0.0, 1.0));
        Oklab::from_linear_srgb(lr, lg, lb, clamp_finite_f32(self.a, 0.0, 1.0))
    }

    #[inline]
    pub fn from_oklab(ok: Oklab) -> Self {
        let (lr, lg, lb) = ok.to_linear_srgb();
        Self {
            r: linear_to_srgb_f32(clamp_finite_f32(lr, 0.0, 1.0)),
            g: linear_to_srgb_f32(clamp_finite_f32(lg, 0.0, 1.0)),
            b: linear_to_srgb_f32(clamp_finite_f32(lb, 0.0, 1.0)),
            a: clamp_finite_f32(ok.alpha, 0.0, 1.0),
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct LinearRgba16 {
    pub r: u16,
    pub g: u16,
    pub b: u16,
    pub a: u16,
}

impl LinearRgba16 {
    pub const TRANSPARENT: Self = Self::new(0, 0, 0, 0);
    pub const BLACK: Self = Self::new(0, 0, 0, 65535);
    pub const WHITE: Self = Self::new(65535, 65535, 65535, 65535);

    #[inline]
    pub const fn new(r: u16, g: u16, b: u16, a: u16) -> Self {
        Self { r, g, b, a }
    }

    #[inline]
    pub fn from_srgb_u8(r: u8, g: u8, b: u8, a: u8) -> Self {
        let la = u8_to_u16_linear(a);
        if la == 65535 {
            Self::new(
                srgb_u8_to_linear_u16(r),
                srgb_u8_to_linear_u16(g),
                srgb_u8_to_linear_u16(b),
                65535,
            )
        } else if la == 0 {
            Self::TRANSPARENT
        } else {
            Self::new(
                mul_u16(srgb_u8_to_linear_u16(r), la),
                mul_u16(srgb_u8_to_linear_u16(g), la),
                mul_u16(srgb_u8_to_linear_u16(b), la),
                la,
            )
        }
    }

    #[inline]
    pub fn to_srgb_u8(self) -> [u8; 4] {
        if self.a == 0 {
            return [0, 0, 0, 0];
        }
        if self.a == 65535 {
            return [
                linear_u16_to_srgb_u8(self.r),
                linear_u16_to_srgb_u8(self.g),
                linear_u16_to_srgb_u8(self.b),
                255,
            ];
        }
        let ur = unpremultiply_u16(self.r, self.a);
        let ug = unpremultiply_u16(self.g, self.a);
        let ub = unpremultiply_u16(self.b, self.a);
        [
            linear_u16_to_srgb_u8(ur),
            linear_u16_to_srgb_u8(ug),
            linear_u16_to_srgb_u8(ub),
            u16_to_u8_linear(self.a),
        ]
    }

    #[inline]
    pub const fn scale_coverage(self, cov: u16) -> Self {
        if cov == 65535 {
            self
        } else if cov == 0 {
            Self::TRANSPARENT
        } else {
            Self::new(
                mul_u16(self.r, cov),
                mul_u16(self.g, cov),
                mul_u16(self.b, cov),
                mul_u16(self.a, cov),
            )
        }
    }

    #[inline]
    pub const fn src_over(self, dst: Self) -> Self {
        if self.a == 65535 {
            return self;
        }
        if self.a == 0 {
            return dst;
        }
        let inv_a = 65535 - self.a;
        Self {
            r: self.r.saturating_add(mul_u16(dst.r, inv_a)),
            g: self.g.saturating_add(mul_u16(dst.g, inv_a)),
            b: self.b.saturating_add(mul_u16(dst.b, inv_a)),
            a: self.a.saturating_add(mul_u16(dst.a, inv_a)),
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct LinearRgbaF32 {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl LinearRgbaF32 {
    pub const TRANSPARENT: Self = Self::new(0.0, 0.0, 0.0, 0.0);

    #[inline]
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Self {
        Self { r, g, b, a }
    }

    #[inline]
    pub fn from_linear_rgba16(c: LinearRgba16) -> Self {
        const INV: f32 = 1.0 / 65535.0;
        Self::new(
            (c.r as f32) * INV,
            (c.g as f32) * INV,
            (c.b as f32) * INV,
            (c.a as f32) * INV,
        )
    }

    #[inline]
    pub fn to_linear_rgba16(self) -> LinearRgba16 {
        let a = round_ties_even_f32(clamp_finite_f32(self.a, 0.0, 1.0) * 65535.0) as u16;
        let max_c = a as f32;
        let r = round_ties_even_f32(clamp_finite_f32(self.r * 65535.0, 0.0, max_c)) as u16;
        let g = round_ties_even_f32(clamp_finite_f32(self.g * 65535.0, 0.0, max_c)) as u16;
        let b = round_ties_even_f32(clamp_finite_f32(self.b * 65535.0, 0.0, max_c)) as u16;
        LinearRgba16::new(r, g, b, a)
    }

    #[inline]
    pub fn src_over(self, dst: Self) -> Self {
        let inv_a = 1.0 - self.a;
        Self {
            r: self.r + dst.r * inv_a,
            g: self.g + dst.g * inv_a,
            b: self.b + dst.b * inv_a,
            a: self.a + dst.a * inv_a,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Oklab {
    pub l: f32,
    pub a: f32,
    pub b: f32,
    pub alpha: f32,
}

impl Oklab {
    #[inline]
    pub const fn new(l: f32, a: f32, b: f32, alpha: f32) -> Self {
        Self { l, a, b, alpha }
    }

    pub fn from_linear_srgb(r: f32, g: f32, b: f32, alpha: f32) -> Self {
        let l = 0.412_221_46 * r + 0.536_332_55 * g + 0.051_445_995 * b;
        let m = 0.211_903_5 * r + 0.680_699_5 * g + 0.107_396_96 * b;
        let s = 0.088_302_46 * r + 0.281_718_84 * g + 0.629_978_7 * b;

        let l_ = cbrt_f32(l);
        let m_ = cbrt_f32(m);
        let s_ = cbrt_f32(s);

        Self {
            l: 0.210_454_26 * l_ + 0.793_617_8 * m_ - 0.004_072_047 * s_,
            a: 1.977_998_5 * l_ - 2.428_592_2 * m_ + 0.450_593_7 * s_,
            b: 0.025_904_037 * l_ + 0.782_771_77 * m_ - 0.808_675_77 * s_,
            alpha,
        }
    }

    pub fn to_linear_srgb(self) -> (f32, f32, f32) {
        let l_ = self.l + 0.396_337_78 * self.a + 0.215_803_76 * self.b;
        let m_ = self.l - 0.105_561_346 * self.a - 0.063_854_17 * self.b;
        let s_ = self.l - 0.089_484_18 * self.a - 1.291_485_5 * self.b;

        let l = l_ * l_ * l_;
        let m = m_ * m_ * m_;
        let s = s_ * s_ * s_;

        (
            4.076_741_7 * l - 3.307_711_6 * m + 0.230_969_94 * s,
            -1.268_438 * l + 2.609_757_4 * m - 0.341_319_38 * s,
            -0.004_196_086_3 * l - 0.703_418_6 * m + 1.707_614_7 * s,
        )
    }

    #[inline]
    pub fn to_oklch(self) -> Oklch {
        let c = sqrt_f32(self.a * self.a + self.b * self.b);
        let h = if c < 1e-6 {
            0.0
        } else {
            let ang = atan2_f32(self.b, self.a);
            if ang < 0.0 { ang + TAU } else { ang }
        };
        Oklch {
            l: self.l,
            c,
            h,
            alpha: self.alpha,
        }
    }

    #[inline]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self {
            l: self.l + (other.l - self.l) * t,
            a: self.a + (other.a - self.a) * t,
            b: self.b + (other.b - self.b) * t,
            alpha: self.alpha + (other.alpha - self.alpha) * t,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Oklch {
    pub l: f32,
    pub c: f32,
    pub h: f32,
    pub alpha: f32,
}

impl Oklch {
    #[inline]
    pub const fn new(l: f32, c: f32, h: f32, alpha: f32) -> Self {
        Self { l, c, h, alpha }
    }

    #[inline]
    pub fn to_oklab(self) -> Oklab {
        let (s, c) = sincos_f32(self.h);
        Oklab {
            l: self.l,
            a: self.c * c,
            b: self.c * s,
            alpha: self.alpha,
        }
    }

    pub fn lerp_hue(self, other: Self, t: f32, longer: bool) -> Self {
        let l = self.l + (other.l - self.l) * t;
        let c = self.c + (other.c - self.c) * t;
        let alpha = self.alpha + (other.alpha - self.alpha) * t;

        // Powerless hue handling when either endpoint is achromatic.
        let h = if self.c < 1e-6 {
            other.h
        } else if other.c < 1e-6 {
            self.h
        } else {
            let mut dh = other.h - self.h;
            if dh > core::f32::consts::PI {
                dh -= TAU;
            } else if dh < -core::f32::consts::PI {
                dh += TAU;
            }
            if longer {
                if dh > 0.0 {
                    dh -= TAU;
                } else {
                    dh += TAU;
                }
            }
            let mut res = self.h + dh * t;
            if res < 0.0 {
                res += TAU;
            } else if res >= TAU {
                res -= TAU;
            }
            res
        };
        Self { l, c, h, alpha }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn srgb_u8_lut_exhaustive_roundtrip() {
        assert_eq!(SRGB_TO_LINEAR_U16[0], 0);
        assert_eq!(SRGB_TO_LINEAR_U16[255], 65535);
        assert_eq!(LINEAR_TO_SRGB_8_8[0], 0);
        assert_eq!(LINEAR_TO_SRGB_8_8[4096], 65280);

        for x in 0..=255u8 {
            let lin = srgb_u8_to_linear_u16(x);
            let back = linear_u16_to_srgb_u8(lin);
            assert_eq!(back, x, "sRGB round-trip failed for {x}");
        }

        // Verify reverse LUT piecewise interpolation accuracy over all 65536 u16 inputs.
        let mut max_err = 0.0f32;
        for v in 0..=65535u16 {
            let got = (linear_u16_to_srgb_8_8(v) as f32) / 256.0;
            let exact = linear_to_srgb_f32((v as f32) / 65535.0) * 255.0;
            let err = abs_f32(got - exact);
            if err > max_err {
                max_err = err;
            }
        }
        assert!(max_err < 0.05, "reverse LUT max sRGB level error = {max_err}");
    }

    #[test]
    fn mul_u16_exact_identities_and_grid() {
        for x in 0..=65535u16 {
            assert_eq!(mul_u16(0, x), 0);
            assert_eq!(mul_u16(65535, x), x);
            assert_eq!(mul_u16(x, 65535), x);
        }

        // Sample dense grid against exact integer division rounding
        for a in (0..=65535u32).step_by(257) {
            for b in (0..=65535u32).step_by(131) {
                let exact = ((a * b + 32767) / 65535) as u16;
                assert_eq!(mul_u16(a as u16, b as u16), exact, "a={a}, b={b}");
            }
        }
        assert_eq!(mul_u16(1, 32767), 0);
        assert_eq!(mul_u16(1, 32768), 1);
    }

    #[test]
    fn oklab_and_display_p3_reference_vectors() {
        // Ottosson reference vectors for sRGB primaries
        let red = Color::RED.to_oklab();
        assert!(abs_f32(red.l - 0.627_955) < 1e-4);
        assert!(abs_f32(red.a - 0.224_863) < 1e-4);
        assert!(abs_f32(red.b - 0.125_846) < 1e-4);

        let green = Color::GREEN.to_oklab();
        assert!(abs_f32(green.l - 0.866_44) < 1e-4);
        assert!(abs_f32(green.a - (-0.233_887)) < 1e-4);
        assert!(abs_f32(green.b - 0.179_498) < 1e-4);

        let blue = Color::BLUE.to_oklab();
        assert!(abs_f32(blue.l - 0.452_014) < 1e-4);
        assert!(abs_f32(blue.a - (-0.032_457)) < 1e-4);
        assert!(abs_f32(blue.b - (-0.311_528)) < 1e-4);

        let white = Color::WHITE.to_oklab();
        assert!(abs_f32(white.l - 1.0) < 1e-5);
        assert!(abs_f32(white.a) < 1e-5 && abs_f32(white.b) < 1e-5);

        // Oklab <-> Oklch <-> Color round-trip
        let c = Color::from_hex(0x0066FF);
        let back = Color::from_oklab(c.to_oklab().to_oklch().to_oklab());
        assert!(abs_f32(c.r - back.r) < 1e-4);
        assert!(abs_f32(c.g - back.g) < 1e-4);
        assert!(abs_f32(c.b - back.b) < 1e-4);

        // Display P3 round-trip and white preservation
        let (pw_r, pw_g, pw_b) = linear_srgb_to_display_p3(1.0, 1.0, 1.0);
        assert!(abs_f32(pw_r - 1.0) < 1e-6 && abs_f32(pw_g - 1.0) < 1e-6 && abs_f32(pw_b - 1.0) < 1e-6);

        let (p3_r, p3_g, p3_b) = linear_srgb_to_display_p3(0.8, 0.3, 0.1);
        let (sr, sg, sb) = linear_display_p3_to_srgb(p3_r, p3_g, p3_b);
        assert!(abs_f32(sr - 0.8) < 1e-5 && abs_f32(sg - 0.3) < 1e-5 && abs_f32(sb - 0.1) < 1e-5);
    }

    #[test]
    fn premultiplied_src_over_invariants() {
        let src = LinearRgba16::from_srgb_u8(255, 0, 0, 128);
        let dst = LinearRgba16::from_srgb_u8(0, 255, 0, 255);
        let out = src.src_over(dst);
        assert_eq!(out.a, 65535);
        // Linear 50% red over 100% green gives bright yellow (~0.5 linear R, ~0.5 linear G -> ~188 sRGB!)
        let srgb = out.to_srgb_u8();
        assert!(srgb[0] >= 184 && srgb[0] <= 190, "r={}", srgb[0]);
        assert!(srgb[1] >= 184 && srgb[1] <= 190, "g={}", srgb[1]);
        assert_eq!(srgb[2], 0);
        assert_eq!(srgb[3], 255);
    }
}
