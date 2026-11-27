#![forbid(unsafe_code)]

use alloc::boxed::Box;
use alloc::sync::Arc;
use alloc::vec::Vec;
use core::f32::consts::TAU;

use crate::math::affine::{Affine2D, Point};
use crate::math::color::{Color, LinearRgba16, LinearRgbaF32, Oklab, srgb_to_linear_f32};
use crate::math::float::{
    abs_f32, abs_f64, atan2_f32, clamp_finite_f32, floor_f32, round_ties_even_f32, sqrt_f64,
};

pub const GRADIENT_RAMP_LEN: usize = 1024;
const RAMP_MAX_F32: f32 = (GRADIENT_RAMP_LEN - 1) as f32;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum SpreadMode {
    #[default]
    Pad,
    Repeat,
    Reflect,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum InterpolationSpace {
    #[default]
    LinearSrgb,
    Srgb,
    Oklab,
    OklchShorter,
    OklchLonger,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ColorStop {
    pub offset: f32,
    pub color: Color,
}

impl ColorStop {
    #[inline]
    pub const fn new(offset: f32, color: Color) -> Self {
        Self { offset, color }
    }
}

#[inline]
pub fn apply_spread(t: f32, spread: SpreadMode) -> f32 {
    if !t.is_finite() {
        return if t.is_nan() || t.is_sign_negative() {
            0.0
        } else {
            1.0
        };
    }
    match spread {
        SpreadMode::Pad => t.clamp(0.0, 1.0),
        SpreadMode::Repeat => {
            let f = t - floor_f32(t);
            if !(0.0..1.0).contains(&f) { 0.0 } else { f }
        }
        SpreadMode::Reflect => {
            let t2 = t * 0.5;
            let period = t2 - floor_f32(t2);
            let doubled = period * 2.0;
            let r = if doubled <= 1.0 {
                doubled
            } else {
                2.0 - doubled
            };
            clamp_finite_f32(r, 0.0, 1.0)
        }
    }
}

#[inline]
fn ramp_index(t: f32, spread: SpreadMode) -> usize {
    let s = apply_spread(t, spread);
    let idx = round_ties_even_f32(s * RAMP_MAX_F32) as usize;
    if idx >= GRADIENT_RAMP_LEN {
        GRADIENT_RAMP_LEN - 1
    } else {
        idx
    }
}

fn interpolate_pair(c0: Color, c1: Color, u: f32, space: InterpolationSpace) -> LinearRgbaF32 {
    let u = clamp_finite_f32(u, 0.0, 1.0);
    match space {
        InterpolationSpace::LinearSrgb => {
            let l0 = c0.to_linear_f32();
            let l1 = c1.to_linear_f32();
            LinearRgbaF32::new(
                l0.r + (l1.r - l0.r) * u,
                l0.g + (l1.g - l0.g) * u,
                l0.b + (l1.b - l0.b) * u,
                l0.a + (l1.a - l0.a) * u,
            )
        }
        InterpolationSpace::Srgb => {
            let a0 = clamp_finite_f32(c0.a, 0.0, 1.0);
            let a1 = clamp_finite_f32(c1.a, 0.0, 1.0);
            let a = a0 + (a1 - a0) * u;
            if a <= 1e-7 {
                return LinearRgbaF32::TRANSPARENT;
            }
            let r0 = clamp_finite_f32(c0.r, -8.0, 8.0) * a0;
            let g0 = clamp_finite_f32(c0.g, -8.0, 8.0) * a0;
            let b0 = clamp_finite_f32(c0.b, -8.0, 8.0) * a0;
            let r1 = clamp_finite_f32(c1.r, -8.0, 8.0) * a1;
            let g1 = clamp_finite_f32(c1.g, -8.0, 8.0) * a1;
            let b1 = clamp_finite_f32(c1.b, -8.0, 8.0) * a1;
            let inv_a = 1.0 / a;
            let sr = (r0 + (r1 - r0) * u) * inv_a;
            let sg = (g0 + (g1 - g0) * u) * inv_a;
            let sb = (b0 + (b1 - b0) * u) * inv_a;
            LinearRgbaF32::new(
                srgb_to_linear_f32(sr) * a,
                srgb_to_linear_f32(sg) * a,
                srgb_to_linear_f32(sb) * a,
                a,
            )
        }
        InterpolationSpace::Oklab => {
            let ok0 = c0.to_oklab();
            let ok1 = c1.to_oklab();
            let a0 = ok0.alpha;
            let a1 = ok1.alpha;
            let alpha = a0 + (a1 - a0) * u;
            if alpha <= 1e-7 {
                return LinearRgbaF32::TRANSPARENT;
            }
            let inv_a = 1.0 / alpha;
            let l = (ok0.l * a0 + (ok1.l * a1 - ok0.l * a0) * u) * inv_a;
            let oa = (ok0.a * a0 + (ok1.a * a1 - ok0.a * a0) * u) * inv_a;
            let ob = (ok0.b * a0 + (ok1.b * a1 - ok0.b * a0) * u) * inv_a;
            let (lr, lg, lb) = Oklab::new(l, oa, ob, alpha).to_linear_srgb();
            LinearRgbaF32::new(
                lr.max(0.0) * alpha,
                lg.max(0.0) * alpha,
                lb.max(0.0) * alpha,
                alpha,
            )
        }
        InterpolationSpace::OklchShorter | InterpolationSpace::OklchLonger => {
            let longer = matches!(space, InterpolationSpace::OklchLonger);
            let ok0 = c0.to_oklab().to_oklch();
            let ok1 = c1.to_oklab().to_oklch();
            let a0 = ok0.alpha;
            let a1 = ok1.alpha;
            let alpha = a0 + (a1 - a0) * u;
            if alpha <= 1e-7 {
                return LinearRgbaF32::TRANSPARENT;
            }
            let interp = if a0 <= 1e-6 {
                crate::math::color::Oklch::new(ok1.l, ok1.c, ok1.h, alpha)
            } else if a1 <= 1e-6 {
                crate::math::color::Oklch::new(ok0.l, ok0.c, ok0.h, alpha)
            } else {
                let mut res = ok0.lerp_hue(ok1, u, longer);
                let inv_a = 1.0 / alpha;
                res.l = (ok0.l * a0 + (ok1.l * a1 - ok0.l * a0) * u) * inv_a;
                res.c = (ok0.c * a0 + (ok1.c * a1 - ok0.c * a0) * u) * inv_a;
                res.alpha = alpha;
                res
            };
            let (lr, lg, lb) = interp.to_oklab().to_linear_srgb();
            LinearRgbaF32::new(
                lr.max(0.0) * alpha,
                lg.max(0.0) * alpha,
                lb.max(0.0) * alpha,
                alpha,
            )
        }
    }
}

fn eval_normalized_stops(
    stops: &[ColorStop],
    t: f32,
    space: InterpolationSpace,
    alpha_scale: f32,
) -> LinearRgbaF32 {
    let raw = if stops.is_empty() {
        LinearRgbaF32::TRANSPARENT
    } else if stops.len() == 1 || t <= stops[0].offset {
        stops[0].color.to_linear_f32()
    } else if t >= stops[stops.len() - 1].offset {
        stops[stops.len() - 1].color.to_linear_f32()
    } else {
        // Find the last stop with offset <= t so duplicate offsets produce exact hard stops.
        let mut idx = 0usize;
        for i in 0..stops.len() - 1 {
            if t < stops[i + 1].offset {
                idx = i;
                break;
            }
            idx = i;
        }
        let s0 = stops[idx];
        let s1 = stops[idx + 1];
        let span = s1.offset - s0.offset;
        if span <= 1e-7 {
            s1.color.to_linear_f32()
        } else {
            let u = (t - s0.offset) / span;
            interpolate_pair(s0.color, s1.color, u, space)
        }
    };
    if alpha_scale >= 1.0 {
        raw
    } else {
        LinearRgbaF32::new(
            raw.r * alpha_scale,
            raw.g * alpha_scale,
            raw.b * alpha_scale,
            raw.a * alpha_scale,
        )
    }
}

fn normalize_stops(stops: &[ColorStop]) -> Vec<ColorStop> {
    let mut out = Vec::with_capacity(stops.len());
    for &s in stops {
        out.push(ColorStop::new(clamp_finite_f32(s.offset, 0.0, 1.0), s.color));
    }
    // Stable sort preserves left-to-right order of duplicate-offset hard stops.
    out.sort_by(|a, b| a.offset.total_cmp(&b.offset));
    out
}

#[derive(Debug, Clone)]
pub struct GradientRamp {
    lut16: [LinearRgba16; GRADIENT_RAMP_LEN],
    lut_f32: Option<Box<[LinearRgbaF32; GRADIENT_RAMP_LEN]>>,
    is_opaque: bool,
}

impl GradientRamp {
    pub fn build(
        stops: &[ColorStop],
        space: InterpolationSpace,
        alpha_scale: f32,
        include_f32: bool,
    ) -> Self {
        let alpha_scale = clamp_finite_f32(alpha_scale, 0.0, 1.0);
        let mut lut16 = [LinearRgba16::TRANSPARENT; GRADIENT_RAMP_LEN];
        let mut lut_f32 = if include_f32 {
            Some(Box::new([LinearRgbaF32::TRANSPARENT; GRADIENT_RAMP_LEN]))
        } else {
            None
        };

        if stops.is_empty() || alpha_scale <= 0.0 {
            return Self {
                lut16,
                lut_f32,
                is_opaque: false,
            };
        }

        let norm = normalize_stops(stops);
        let mut is_opaque = true;
        let inv_max = 1.0 / RAMP_MAX_F32;

        for (i, slot16) in lut16.iter_mut().enumerate() {
            let t = (i as f32) * inv_max;
            let c_f32 = eval_normalized_stops(&norm, t, space, alpha_scale);
            let c16 = c_f32.to_linear_rgba16();
            if c16.a < 65535 || c_f32.a < 1.0 {
                is_opaque = false;
            }
            *slot16 = c16;
            if let Some(ref mut f32_arr) = lut_f32 {
                f32_arr[i] = c_f32;
            }
        }

        Self {
            lut16,
            lut_f32,
            is_opaque,
        }
    }

    #[inline]
    pub fn is_opaque(&self) -> bool {
        self.is_opaque
    }

    #[inline]
    pub fn has_f32(&self) -> bool {
        self.lut_f32.is_some()
    }

    #[inline]
    pub fn sample_u16(&self, t: f32, spread: SpreadMode) -> LinearRgba16 {
        self.lut16[ramp_index(t, spread)]
    }

    #[inline]
    pub fn sample_f32(&self, t: f32, spread: SpreadMode) -> LinearRgbaF32 {
        let idx = ramp_index(t, spread);
        if let Some(ref arr) = self.lut_f32 {
            arr[idx]
        } else {
            LinearRgbaF32::from_linear_rgba16(self.lut16[idx])
        }
    }

    #[inline]
    pub fn entries_u16(&self) -> &[LinearRgba16; GRADIENT_RAMP_LEN] {
        &self.lut16
    }
}

#[derive(Debug, Clone)]
pub struct LinearGradient {
    pub p0: Point,
    pub p1: Point,
    pub stops: Vec<ColorStop>,
    pub spread: SpreadMode,
    pub interpolation: InterpolationSpace,
    pub transform: Affine2D,
    ramp: Option<Arc<GradientRamp>>,
}

impl PartialEq for LinearGradient {
    fn eq(&self, other: &Self) -> bool {
        self.p0 == other.p0
            && self.p1 == other.p1
            && self.stops == other.stops
            && self.spread == other.spread
            && self.interpolation == other.interpolation
            && self.transform == other.transform
    }
}

impl LinearGradient {
    pub fn new(p0: Point, p1: Point) -> Self {
        Self {
            p0,
            p1,
            stops: Vec::new(),
            spread: SpreadMode::Pad,
            interpolation: InterpolationSpace::LinearSrgb,
            transform: Affine2D::IDENTITY,
            ramp: None,
        }
    }

    pub fn with_stops(mut self, stops: &[ColorStop]) -> Self {
        self.stops.clear();
        self.stops.extend_from_slice(stops);
        self.ramp = None;
        self
    }

    pub fn add_stop(mut self, offset: f32, color: Color) -> Self {
        self.stops.push(ColorStop::new(offset, color));
        self.ramp = None;
        self
    }

    pub fn with_spread(mut self, spread: SpreadMode) -> Self {
        self.spread = spread;
        self
    }

    pub fn with_interpolation(mut self, interpolation: InterpolationSpace) -> Self {
        self.interpolation = interpolation;
        self.ramp = None;
        self
    }

    pub fn with_transform(mut self, transform: Affine2D) -> Self {
        self.transform = transform;
        self
    }

    pub fn bake(&mut self) {
        if self.ramp.is_none() {
            self.ramp = Some(Arc::new(GradientRamp::build(
                &self.stops,
                self.interpolation,
                1.0,
                false,
            )));
        }
    }

    pub(crate) fn get_or_build_ramp(
        &self,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Arc<GradientRamp> {
        if alpha_scale >= 1.0
            && let Some(ref r) = self.ramp
            && (!use_f32 || r.has_f32())
        {
            return Arc::clone(r);
        }
        Arc::new(GradientRamp::build(
            &self.stops,
            self.interpolation,
            alpha_scale,
            use_f32,
        ))
    }
}

#[derive(Debug, Clone)]
pub struct RadialGradient {
    pub c0: Point,
    pub r0: f32,
    pub c1: Point,
    pub r1: f32,
    pub stops: Vec<ColorStop>,
    pub spread: SpreadMode,
    pub interpolation: InterpolationSpace,
    pub transform: Affine2D,
    ramp: Option<Arc<GradientRamp>>,
}

impl PartialEq for RadialGradient {
    fn eq(&self, other: &Self) -> bool {
        self.c0 == other.c0
            && self.r0 == other.r0
            && self.c1 == other.c1
            && self.r1 == other.r1
            && self.stops == other.stops
            && self.spread == other.spread
            && self.interpolation == other.interpolation
            && self.transform == other.transform
    }
}

impl RadialGradient {
    pub fn new(center: Point, radius: f32) -> Self {
        Self::new_two_point(center, 0.0, center, radius)
    }

    pub fn new_two_point(c0: Point, r0: f32, c1: Point, r1: f32) -> Self {
        Self {
            c0,
            r0: r0.max(0.0),
            c1,
            r1: r1.max(0.0),
            stops: Vec::new(),
            spread: SpreadMode::Pad,
            interpolation: InterpolationSpace::LinearSrgb,
            transform: Affine2D::IDENTITY,
            ramp: None,
        }
    }

    pub fn with_stops(mut self, stops: &[ColorStop]) -> Self {
        self.stops.clear();
        self.stops.extend_from_slice(stops);
        self.ramp = None;
        self
    }

    pub fn add_stop(mut self, offset: f32, color: Color) -> Self {
        self.stops.push(ColorStop::new(offset, color));
        self.ramp = None;
        self
    }

    pub fn with_spread(mut self, spread: SpreadMode) -> Self {
        self.spread = spread;
        self
    }

    pub fn with_interpolation(mut self, interpolation: InterpolationSpace) -> Self {
        self.interpolation = interpolation;
        self.ramp = None;
        self
    }

    pub fn with_transform(mut self, transform: Affine2D) -> Self {
        self.transform = transform;
        self
    }

    pub fn bake(&mut self) {
        if self.ramp.is_none() {
            self.ramp = Some(Arc::new(GradientRamp::build(
                &self.stops,
                self.interpolation,
                1.0,
                false,
            )));
        }
    }

    pub(crate) fn get_or_build_ramp(
        &self,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Arc<GradientRamp> {
        if alpha_scale >= 1.0
            && let Some(ref r) = self.ramp
            && (!use_f32 || r.has_f32())
        {
            return Arc::clone(r);
        }
        Arc::new(GradientRamp::build(
            &self.stops,
            self.interpolation,
            alpha_scale,
            use_f32,
        ))
    }
}

#[derive(Debug, Clone)]
pub struct ConicGradient {
    pub center: Point,
    pub start_angle: f32,
    pub end_angle: f32,
    pub stops: Vec<ColorStop>,
    pub spread: SpreadMode,
    pub interpolation: InterpolationSpace,
    pub transform: Affine2D,
    ramp: Option<Arc<GradientRamp>>,
}

impl PartialEq for ConicGradient {
    fn eq(&self, other: &Self) -> bool {
        self.center == other.center
            && self.start_angle == other.start_angle
            && self.end_angle == other.end_angle
            && self.stops == other.stops
            && self.spread == other.spread
            && self.interpolation == other.interpolation
            && self.transform == other.transform
    }
}

impl ConicGradient {
    pub fn new(center: Point, start_angle: f32) -> Self {
        Self::new_sweep(center, start_angle, start_angle + TAU)
    }

    pub fn new_sweep(center: Point, start_angle: f32, end_angle: f32) -> Self {
        Self {
            center,
            start_angle,
            end_angle,
            stops: Vec::new(),
            spread: SpreadMode::Pad,
            interpolation: InterpolationSpace::LinearSrgb,
            transform: Affine2D::IDENTITY,
            ramp: None,
        }
    }

    pub fn with_stops(mut self, stops: &[ColorStop]) -> Self {
        self.stops.clear();
        self.stops.extend_from_slice(stops);
        self.ramp = None;
        self
    }

    pub fn add_stop(mut self, offset: f32, color: Color) -> Self {
        self.stops.push(ColorStop::new(offset, color));
        self.ramp = None;
        self
    }

    pub fn with_spread(mut self, spread: SpreadMode) -> Self {
        self.spread = spread;
        self
    }

    pub fn with_interpolation(mut self, interpolation: InterpolationSpace) -> Self {
        self.interpolation = interpolation;
        self.ramp = None;
        self
    }

    pub fn with_transform(mut self, transform: Affine2D) -> Self {
        self.transform = transform;
        self
    }

    pub fn bake(&mut self) {
        if self.ramp.is_none() {
            self.ramp = Some(Arc::new(GradientRamp::build(
                &self.stops,
                self.interpolation,
                1.0,
                false,
            )));
        }
    }

    pub(crate) fn get_or_build_ramp(
        &self,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Arc<GradientRamp> {
        if alpha_scale >= 1.0
            && let Some(ref r) = self.ramp
            && (!use_f32 || r.has_f32())
        {
            return Arc::clone(r);
        }
        Arc::new(GradientRamp::build(
            &self.stops,
            self.interpolation,
            alpha_scale,
            use_f32,
        ))
    }
}

#[derive(Debug, Clone)]
enum PreparedKind {
    Constant(f32),
    Linear {
        inv: Affine2D,
        p0x: f64,
        p0y: f64,
        ux: f64,
        uy: f64,
        dt_dx: f64,
    },
    RadialConcentric {
        inv: Affine2D,
        cx: f64,
        cy: f64,
        r0: f64,
        dr: f64,
        inv_dr: f64,
    },
    RadialTwoPoint {
        inv: Affine2D,
        c0x: f64,
        c0y: f64,
        r0: f64,
        cdx: f64,
        cdy: f64,
        dr: f64,
        a: f64,
    },
    Conic {
        inv: Affine2D,
        cx: f32,
        cy: f32,
        start_angle: f32,
        sweep: f32,
        inv_sweep: f32,
    },
}

#[derive(Debug, Clone)]
pub(crate) struct PreparedGradient {
    ramp: Arc<GradientRamp>,
    spread: SpreadMode,
    kind: PreparedKind,
    is_opaque: bool,
}

impl PreparedGradient {
    pub fn prepare_linear(
        grad: &LinearGradient,
        ctm: Affine2D,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Option<Self> {
        let total = ctm.pre_concat(grad.transform);
        let inv = total.invert()?;
        let ramp = grad.get_or_build_ramp(alpha_scale, use_f32);
        let is_opaque = ramp.is_opaque();
        let dx = (grad.p1.x as f64) - (grad.p0.x as f64);
        let dy = (grad.p1.y as f64) - (grad.p0.y as f64);
        let len_sq = dx * dx + dy * dy;
        let kind = if !len_sq.is_finite() || len_sq <= 1e-12 {
            PreparedKind::Constant(1.0)
        } else {
            let ux = dx / len_sq;
            let uy = dy / len_sq;
            let dt_dx = (inv.0[0] as f64) * ux + (inv.0[1] as f64) * uy;
            PreparedKind::Linear {
                inv,
                p0x: grad.p0.x as f64,
                p0y: grad.p0.y as f64,
                ux,
                uy,
                dt_dx,
            }
        };
        Some(Self {
            ramp,
            spread: grad.spread,
            kind,
            is_opaque,
        })
    }

    pub fn prepare_radial(
        grad: &RadialGradient,
        ctm: Affine2D,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Option<Self> {
        let total = ctm.pre_concat(grad.transform);
        let inv = total.invert()?;
        let ramp = grad.get_or_build_ramp(alpha_scale, use_f32);
        let r0 = (grad.r0 as f64).max(0.0);
        let r1 = (grad.r1 as f64).max(0.0);
        let cdx = (grad.c1.x as f64) - (grad.c0.x as f64);
        let cdy = (grad.c1.y as f64) - (grad.c0.y as f64);
        let dr = r1 - r0;
        let dist_sq = cdx * cdx + cdy * cdy;

        if dist_sq <= 1e-18 {
            let is_opaque = ramp.is_opaque() && abs_f64(dr) > 1e-12;
            let kind = PreparedKind::RadialConcentric {
                inv,
                cx: grad.c0.x as f64,
                cy: grad.c0.y as f64,
                r0,
                dr,
                inv_dr: if abs_f64(dr) > 1e-12 { 1.0 / dr } else { 0.0 },
            };
            Some(Self {
                ramp,
                spread: grad.spread,
                kind,
                is_opaque,
            })
        } else {
            let a = dist_sq - dr * dr;
            // Only guaranteed to cover the full plane when one circle is strictly inside the other (A < 0).
            let is_opaque = ramp.is_opaque() && a < -1e-12;
            let kind = PreparedKind::RadialTwoPoint {
                inv,
                c0x: grad.c0.x as f64,
                c0y: grad.c0.y as f64,
                r0,
                cdx,
                cdy,
                dr,
                a,
            };
            Some(Self {
                ramp,
                spread: grad.spread,
                kind,
                is_opaque,
            })
        }
    }

    pub fn prepare_conic(
        grad: &ConicGradient,
        ctm: Affine2D,
        alpha_scale: f32,
        use_f32: bool,
    ) -> Option<Self> {
        let total = ctm.pre_concat(grad.transform);
        let inv = total.invert()?;
        let ramp = grad.get_or_build_ramp(alpha_scale, use_f32);
        let is_opaque = ramp.is_opaque();
        let sweep = grad.end_angle - grad.start_angle;
        let kind = if !sweep.is_finite() || abs_f32(sweep) <= 1e-9 {
            PreparedKind::Constant(0.0)
        } else {
            PreparedKind::Conic {
                inv,
                cx: grad.center.x,
                cy: grad.center.y,
                start_angle: grad.start_angle,
                sweep,
                inv_sweep: 1.0 / sweep,
            }
        };
        Some(Self {
            ramp,
            spread: grad.spread,
            kind,
            is_opaque,
        })
    }

    #[inline]
    pub fn is_opaque(&self) -> bool {
        self.is_opaque
    }

    #[inline]
    fn solve_two_point_t(
        px: f64,
        py: f64,
        r0: f64,
        cdx: f64,
        cdy: f64,
        dr: f64,
        a: f64,
    ) -> Option<f32> {
        let b = px * cdx + py * cdy + r0 * dr;
        let c = px * px + py * py - r0 * r0;
        if abs_f64(a) <= 1e-12 {
            // Internally tangent circles: -2*B*t + C = 0
            if abs_f64(b) <= 1e-12 {
                return None;
            }
            let t = c / (2.0 * b);
            if r0 + t * dr >= -1e-9 {
                Some(t as f32)
            } else {
                None
            }
        } else {
            let disc = b * b - a * c;
            if disc < 0.0 {
                return None;
            }
            let s = sqrt_f64(disc);
            let inv_a = 1.0 / a;
            let (t_large, t_small) = if a > 0.0 {
                ((b + s) * inv_a, (b - s) * inv_a)
            } else {
                ((b - s) * inv_a, (b + s) * inv_a)
            };
            if r0 + t_large * dr >= -1e-9 {
                Some(t_large as f32)
            } else if r0 + t_small * dr >= -1e-9 {
                Some(t_small as f32)
            } else {
                None
            }
        }
    }

    #[inline]
    fn eval_conic_t(gx: f32, gy: f32, cx: f32, cy: f32, start_angle: f32, sweep: f32, inv_sweep: f32) -> f32 {
        let mut delta = atan2_f32(gy - cy, gx - cx) - start_angle;
        delta -= floor_f32(delta * (1.0 / TAU)) * TAU;
        if sweep < 0.0 && delta > 0.0 {
            delta -= TAU;
        }
        delta * inv_sweep
    }

    pub fn eval_span_u16(&self, x0: usize, y: usize, out: &mut [LinearRgba16]) {
        let fx0 = (x0 as f64) + 0.5;
        let fy = (y as f64) + 0.5;
        match self.kind {
            PreparedKind::Constant(t) => {
                out.fill(self.ramp.sample_u16(t, self.spread));
            }
            PreparedKind::Linear {
                inv,
                p0x,
                p0y,
                ux,
                uy,
                dt_dx,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64);
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64);
                let t0 = (gx0 - p0x) * ux + (gy0 - p0y) * uy;
                for (i, dst) in out.iter_mut().enumerate() {
                    let t = (t0 + (i as f64) * dt_dx) as f32;
                    *dst = self.ramp.sample_u16(t, self.spread);
                }
            }
            PreparedKind::RadialConcentric {
                inv,
                cx,
                cy,
                r0,
                dr,
                inv_dr,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64) - cx;
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64) - cy;
                let step_x = sx as f64;
                let step_y = ky as f64;
                let degenerate = abs_f64(dr) <= 1e-12;
                let r0_sq = r0 * r0;
                for (i, dst) in out.iter_mut().enumerate() {
                    let px = gx0 + (i as f64) * step_x;
                    let py = gy0 + (i as f64) * step_y;
                    let d2 = px * px + py * py;
                    if degenerate {
                        *dst = if d2 <= r0_sq {
                            self.ramp.sample_u16(1.0, self.spread)
                        } else {
                            LinearRgba16::TRANSPARENT
                        };
                    } else {
                        let t = ((sqrt_f64(d2) - r0) * inv_dr) as f32;
                        *dst = self.ramp.sample_u16(t, self.spread);
                    }
                }
            }
            PreparedKind::RadialTwoPoint {
                inv,
                c0x,
                c0y,
                r0,
                cdx,
                cdy,
                dr,
                a,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64) - c0x;
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64) - c0y;
                let step_x = sx as f64;
                let step_y = ky as f64;
                for (i, dst) in out.iter_mut().enumerate() {
                    let px = gx0 + (i as f64) * step_x;
                    let py = gy0 + (i as f64) * step_y;
                    *dst = match Self::solve_two_point_t(px, py, r0, cdx, cdy, dr, a) {
                        Some(t) => self.ramp.sample_u16(t, self.spread),
                        None => LinearRgba16::TRANSPARENT,
                    };
                }
            }
            PreparedKind::Conic {
                inv,
                cx,
                cy,
                start_angle,
                sweep,
                inv_sweep,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let fx0_32 = (x0 as f32) + 0.5;
                let fy_32 = (y as f32) + 0.5;
                let gx0 = sx * fx0_32 + kx * fy_32 + tx;
                let gy0 = ky * fx0_32 + sy * fy_32 + ty;
                for (i, dst) in out.iter_mut().enumerate() {
                    let fi = i as f32;
                    let gx = gx0 + fi * sx;
                    let gy = gy0 + fi * ky;
                    let t = Self::eval_conic_t(gx, gy, cx, cy, start_angle, sweep, inv_sweep);
                    *dst = self.ramp.sample_u16(t, self.spread);
                }
            }
        }
    }

    pub fn eval_span_f32(&self, x0: usize, y: usize, out: &mut [LinearRgbaF32]) {
        let fx0 = (x0 as f64) + 0.5;
        let fy = (y as f64) + 0.5;
        match self.kind {
            PreparedKind::Constant(t) => {
                out.fill(self.ramp.sample_f32(t, self.spread));
            }
            PreparedKind::Linear {
                inv,
                p0x,
                p0y,
                ux,
                uy,
                dt_dx,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64);
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64);
                let t0 = (gx0 - p0x) * ux + (gy0 - p0y) * uy;
                for (i, dst) in out.iter_mut().enumerate() {
                    let t = (t0 + (i as f64) * dt_dx) as f32;
                    *dst = self.ramp.sample_f32(t, self.spread);
                }
            }
            PreparedKind::RadialConcentric {
                inv,
                cx,
                cy,
                r0,
                dr,
                inv_dr,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64) - cx;
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64) - cy;
                let step_x = sx as f64;
                let step_y = ky as f64;
                let degenerate = abs_f64(dr) <= 1e-12;
                let r0_sq = r0 * r0;
                for (i, dst) in out.iter_mut().enumerate() {
                    let px = gx0 + (i as f64) * step_x;
                    let py = gy0 + (i as f64) * step_y;
                    let d2 = px * px + py * py;
                    if degenerate {
                        *dst = if d2 <= r0_sq {
                            self.ramp.sample_f32(1.0, self.spread)
                        } else {
                            LinearRgbaF32::TRANSPARENT
                        };
                    } else {
                        let t = ((sqrt_f64(d2) - r0) * inv_dr) as f32;
                        *dst = self.ramp.sample_f32(t, self.spread);
                    }
                }
            }
            PreparedKind::RadialTwoPoint {
                inv,
                c0x,
                c0y,
                r0,
                cdx,
                cdy,
                dr,
                a,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let gx0 = (sx as f64) * fx0 + (kx as f64) * fy + (tx as f64) - c0x;
                let gy0 = (ky as f64) * fx0 + (sy as f64) * fy + (ty as f64) - c0y;
                let step_x = sx as f64;
                let step_y = ky as f64;
                for (i, dst) in out.iter_mut().enumerate() {
                    let px = gx0 + (i as f64) * step_x;
                    let py = gy0 + (i as f64) * step_y;
                    *dst = match Self::solve_two_point_t(px, py, r0, cdx, cdy, dr, a) {
                        Some(t) => self.ramp.sample_f32(t, self.spread),
                        None => LinearRgbaF32::TRANSPARENT,
                    };
                }
            }
            PreparedKind::Conic {
                inv,
                cx,
                cy,
                start_angle,
                sweep,
                inv_sweep,
            } => {
                let [sx, ky, kx, sy, tx, ty] = inv.0;
                let fx0_32 = (x0 as f32) + 0.5;
                let fy_32 = (y as f32) + 0.5;
                let gx0 = sx * fx0_32 + kx * fy_32 + tx;
                let gy0 = ky * fx0_32 + sy * fy_32 + ty;
                for (i, dst) in out.iter_mut().enumerate() {
                    let fi = i as f32;
                    let gx = gx0 + fi * sx;
                    let gy = gy0 + fi * ky;
                    let t = Self::eval_conic_t(gx, gy, cx, cy, start_angle, sweep, inv_sweep);
                    *dst = self.ramp.sample_f32(t, self.spread);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::color::ColorSpace;
    use crate::math::dither::DitherMode;
    use crate::scene::Context;
    use crate::surface::{PixelFormat, Surface};

    #[test]
    fn spread_modes_pad_repeat_reflect() {
        assert_eq!(apply_spread(-0.5, SpreadMode::Pad), 0.0);
        assert_eq!(apply_spread(1.5, SpreadMode::Pad), 1.0);
        assert!(abs_f32(apply_spread(0.35, SpreadMode::Pad) - 0.35) < 1e-6);

        assert!(abs_f32(apply_spread(1.25, SpreadMode::Repeat) - 0.25) < 1e-6);
        assert!(abs_f32(apply_spread(-0.25, SpreadMode::Repeat) - 0.75) < 1e-6);

        assert!(abs_f32(apply_spread(1.25, SpreadMode::Reflect) - 0.75) < 1e-6);
        assert!(abs_f32(apply_spread(2.25, SpreadMode::Reflect) - 0.25) < 1e-6);
        assert!(abs_f32(apply_spread(-0.25, SpreadMode::Reflect) - 0.25) < 1e-6);
    }

    #[test]
    fn oklab_vs_linear_srgb_vs_srgb_blue_to_orange_has_no_gray_dead_zone() {
        let blue = Color::from_hex(0x0066FF);
        let orange = Color::from_hex(0xFF4400);
        let stops = [ColorStop::new(0.0, blue), ColorStop::new(1.0, orange)];

        let ramp_oklab = GradientRamp::build(&stops, InterpolationSpace::Oklab, 1.0, true);
        let ramp_oklch = GradientRamp::build(&stops, InterpolationSpace::OklchShorter, 1.0, true);
        let ramp_lin = GradientRamp::build(&stops, InterpolationSpace::LinearSrgb, 1.0, true);
        let ramp_srgb = GradientRamp::build(&stops, InterpolationSpace::Srgb, 1.0, true);

        let mid_oklab = ramp_oklab.sample_f32(0.5, SpreadMode::Pad);
        let mid_oklch = ramp_oklch.sample_f32(0.5, SpreadMode::Pad);
        let mid_lin = ramp_lin.sample_f32(0.5, SpreadMode::Pad);
        let mid_srgb = ramp_srgb.sample_f32(0.5, SpreadMode::Pad);

        // Compute perceptual Oklab chroma C = sqrt(a^2 + b^2) at midpoint:
        // OklchShorter preserves full chroma; Oklab preserves perceptual lightness cleanly
        // without the muddy sRGB gamma dip!
        let ok_lin = Oklab::from_linear_srgb(mid_lin.r, mid_lin.g, mid_lin.b, 1.0);
        let ok_srgb = Oklab::from_linear_srgb(mid_srgb.r, mid_srgb.g, mid_srgb.b, 1.0);
        let ok_lab = Oklab::from_linear_srgb(mid_oklab.r, mid_oklab.g, mid_oklab.b, 1.0);
        let ok_lch = Oklab::from_linear_srgb(mid_oklch.r, mid_oklch.g, mid_oklch.b, 1.0);

        let c_srgb = ok_srgb.to_oklch().c;
        let c_lch = ok_lch.to_oklch().c;
        assert!(c_lch > c_srgb * 1.4, "oklch chroma={c_lch} srgb chroma={c_srgb}");
        // Oklab midpoint lightness is strictly the perceptual average of endpoints:
        let l_expected = 0.5 * (blue.to_oklab().l + orange.to_oklab().l);
        assert!(
            abs_f32(ok_lab.l - l_expected) < 0.01,
            "ok_lab.l={} expected={}",
            ok_lab.l,
            l_expected
        );
        // Linear sRGB midpoint is strictly the linear average of endpoints:
        let b_lin = blue.to_linear_f32();
        let o_lin = orange.to_linear_f32();
        assert!(abs_f32(mid_lin.r - 0.5 * (b_lin.r + o_lin.r)) < 0.005);
        assert!(abs_f32(mid_lin.b - 0.5 * (b_lin.b + o_lin.b)) < 0.005);
        // sRGB midpoint dips lower in linear luminance than linear sRGB:
        assert!(ok_srgb.l < ok_lin.l);
    }

    #[test]
    fn hard_stops_and_unsorted_stops() {
        // Unsorted stops with duplicate offset at 0.5
        let stops = [
            ColorStop::new(1.0, Color::BLUE),
            ColorStop::new(0.5, Color::RED),
            ColorStop::new(0.0, Color::RED),
            ColorStop::new(0.5, Color::BLUE),
        ];
        let ramp = GradientRamp::build(&stops, InterpolationSpace::LinearSrgb, 1.0, false);
        let left = ramp.sample_u16(0.49, SpreadMode::Pad);
        let right = ramp.sample_u16(0.51, SpreadMode::Pad);
        assert_eq!(left, Color::RED.to_linear_rgba16());
        assert_eq!(right, Color::BLUE.to_linear_rgba16());
    }

    #[test]
    fn two_point_focal_radial_inside_on_rim_and_outside() {
        let stops = [ColorStop::new(0.0, Color::WHITE), ColorStop::new(1.0, Color::BLACK)];

        // 1. Focal point strictly inside outer circle (c0=(20,16), r0=0, c1=(16,16), r1=12)
        let g_in = RadialGradient::new_two_point(Point::new(20.0, 16.0), 0.0, Point::new(16.0, 16.0), 12.0)
            .with_stops(&stops);
        let p_in = PreparedGradient::prepare_radial(&g_in, Affine2D::IDENTITY, 1.0, false).unwrap();
        assert!(p_in.is_opaque());
        let mut row = [LinearRgba16::TRANSPARENT; 32];
        p_in.eval_span_u16(0, 16, &mut row);
        // Near focal point x=20 should be brightest (closest to WHITE)
        assert!(row[20].r > 60000);
        // Far rim x=4 (c1.x - r1 = 4) should be darkest (BLACK)
        assert!(row[4].r < 2000);

        // 2. Internally tangent edge case: A == 0 (c0=(28,16), r0=0, c1=(16,16), r1=12)
        let g_tan = RadialGradient::new_two_point(Point::new(28.0, 16.0), 0.0, Point::new(16.0, 16.0), 12.0)
            .with_stops(&stops);
        let p_tan = PreparedGradient::prepare_radial(&g_tan, Affine2D::IDENTITY, 1.0, false).unwrap();
        assert!(!p_tan.is_opaque());
        p_tan.eval_span_u16(0, 16, &mut row);
        // Inside tangent circle (x=16, center of outer circle): valid t=0.5 -> mid gray
        assert!(row[16].a == 65535 && row[16].r > 20000 && row[16].r < 45000);
        // Beyond tangent point (x=30 > 28): outside cone -> transparent!
        assert_eq!(row[30], LinearRgba16::TRANSPARENT);

        // 3. Focal point outside outer circle: c0=(28,16), r0=0, c1=(12,16), r1=8 (dist=16 > dr=8, A > 0)
        let g_out = RadialGradient::new_two_point(Point::new(28.0, 16.0), 0.0, Point::new(12.0, 16.0), 8.0)
            .with_stops(&stops);
        let p_out = PreparedGradient::prepare_radial(&g_out, Affine2D::IDENTITY, 1.0, false).unwrap();
        assert!(!p_out.is_opaque());
        p_out.eval_span_u16(0, 16, &mut row);
        // Behind focal point x=30: outside cone -> transparent!
        assert_eq!(row[30], LinearRgba16::TRANSPARENT);
        // Inside cone x=12 (center of outer circle): valid opaque pixel!
        assert_eq!(row[12].a, 65535);
    }

    #[test]
    fn conic_gradient_quadrants_and_seam() {
        let stops = [
            ColorStop::new(0.0, Color::RED),
            ColorStop::new(0.25, Color::GREEN),
            ColorStop::new(0.5, Color::BLUE),
            ColorStop::new(0.75, Color::WHITE),
            ColorStop::new(1.0, Color::RED),
        ];
        let conic = ConicGradient::new(Point::new(32.0, 32.0), 0.0).with_stops(&stops);
        let prep = PreparedGradient::prepare_conic(&conic, Affine2D::IDENTITY, 1.0, false).unwrap();
        let mut px = [LinearRgba16::TRANSPARENT; 1];

        // +X (angle 0 -> t=0 -> RED)
        prep.eval_span_u16(50, 32, &mut px);
        assert!(px[0].r > 60000 && px[0].g < 5000 && px[0].b < 5000);
        // +Y (angle PI/2 -> t=0.25 -> GREEN)
        prep.eval_span_u16(32, 50, &mut px);
        assert!(px[0].g > 60000 && px[0].r < 5000 && px[0].b < 5000);
        // -X (angle PI -> t=0.5 -> BLUE)
        prep.eval_span_u16(10, 32, &mut px);
        assert!(px[0].b > 60000 && px[0].r < 5000 && px[0].g < 5000);
        // -Y (angle 3*PI/2 -> t=0.75 -> WHITE)
        prep.eval_span_u16(32, 10, &mut px);
        assert!(px[0].r > 60000 && px[0].g > 60000 && px[0].b > 60000);
    }

    #[test]
    fn context_renders_gradients_with_dithering_and_hdr_f32() {
        // 1. 8-bit sRGB with BlueNoise dithering across a 128x64 linear Oklab gradient
        let mut surf = Surface::new_rgba8(128, 64).unwrap();
        {
            let mut ctx = Context::new(&mut surf);
            ctx.set_dither_mode(DitherMode::BlueNoise);
            let grad = LinearGradient::new(Point::new(0.0, 0.0), Point::new(128.0, 0.0))
                .add_stop(0.0, Color::from_hex(0x0066FF))
                .add_stop(1.0, Color::from_hex(0xFF4400))
                .with_interpolation(InterpolationSpace::Oklab);
            ctx.set_paint(grad);
            ctx.rectangle(0.0, 0.0, 128.0, 64.0);
            ctx.fill_current();
        }
        let left = &surf.row(32).unwrap()[0..4];
        let right = &surf.row(32).unwrap()[127 * 4..128 * 4];
        assert!(left[2] > 240 && left[0] < 20, "left={left:?}");
        assert!(right[0] > 240 && right[2] < 20, "right={right:?}");

        // 2. HDR f32 surface preserves super-white gradient stop > 1.0
        let mut hdr =
            Surface::try_new(64, 32, PixelFormat::RgbaF32Linear, ColorSpace::LinearSrgb).unwrap();
        {
            let mut ctx = Context::new(&mut hdr);
            let grad = LinearGradient::new(Point::new(0.0, 0.0), Point::new(64.0, 0.0))
                .add_stop(0.0, Color::from_linear_rgba(0.0, 0.0, 0.0, 1.0))
                .add_stop(1.0, Color::from_linear_rgba(4.0, 2.0, 0.0, 1.0))
                .with_interpolation(InterpolationSpace::LinearSrgb);
            ctx.set_paint(grad);
            ctx.paint();
        }
        let mid = hdr.pixel_at_f32(32, 16).unwrap();
        assert!(abs_f32(mid.r - 2.0) < 0.08 && abs_f32(mid.g - 1.0) < 0.05, "mid={mid:?}");
    }

    #[test]
    fn gradient_visual_png_roundtrip_transforms_strokes_and_edge_cases() {
        // 1. Visual comparison strip: #0066FF -> #FF4400 across Oklab, OklchShorter, LinearSrgb, Srgb
        let mut strip = Surface::new_rgba8(128, 64).unwrap();
        {
            let mut ctx = Context::new(&mut strip);
            ctx.set_dither_mode(DitherMode::BlueNoise);
            let spaces = [
                InterpolationSpace::Oklab,
                InterpolationSpace::OklchShorter,
                InterpolationSpace::LinearSrgb,
                InterpolationSpace::Srgb,
            ];
            for (idx, space) in spaces.into_iter().enumerate() {
                let y = (idx as f32) * 16.0;
                let grad = LinearGradient::new(Point::new(0.0, y), Point::new(128.0, y))
                    .add_stop(0.0, Color::from_hex(0x0066FF))
                    .add_stop(1.0, Color::from_hex(0xFF4400))
                    .with_interpolation(space);
                ctx.set_paint(grad);
                ctx.rectangle(0.0, y, 128.0, 16.0);
                ctx.fill_current();
            }
        }
        #[cfg(feature = "codec-png")]
        {
            let png = crate::codec::png::encode_png(strip.as_ref()).unwrap();
            let decoded = crate::codec::png::decode_png(&png).unwrap();
            assert_eq!(decoded.data(), strip.data());
        }

        // 2. Gradient-painted stroke under CTM transform + paint_with_alpha on radial gradient
        let mut surf = Surface::new_rgba8(64, 64).unwrap();
        {
            let mut ctx = Context::new(&mut surf);
            ctx.set_dither_mode(DitherMode::None);
            ctx.clear(Color::BLACK);

            let rad = RadialGradient::new_two_point(
                Point::new(32.0, 32.0),
                4.0,
                Point::new(32.0, 32.0),
                28.0,
            )
            .add_stop(0.0, Color::WHITE)
            .add_stop(1.0, Color::from_hex(0x0000FF))
            .with_spread(SpreadMode::Reflect);
            ctx.set_paint(rad);
            ctx.paint_with_alpha(0.5);

            ctx.save();
            ctx.translate(8.0, 8.0);
            let mut stroke_grad =
                LinearGradient::new(Point::new(0.0, 0.0), Point::new(48.0, 0.0))
                    .add_stop(0.0, Color::RED)
                    .add_stop(1.0, Color::GREEN)
                    .with_spread(SpreadMode::Repeat);
            stroke_grad.bake();
            ctx.set_paint(stroke_grad);
            ctx.set_line_width(6.0);
            ctx.move_to(0.0, 24.0);
            ctx.line_to(48.0, 24.0);
            ctx.stroke_current();
            ctx.restore();
        }
        // Background center (32, 8) got 50% radial gradient over black (non-black, not full white)
        let bg = &surf.row(8).unwrap()[32 * 4..32 * 4 + 4];
        assert!(bg[2] > 100 && bg[3] == 255, "bg={bg:?}");
        // Left end of translated stroke at device (10, 32) is strongly red
        let st_left = &surf.row(32).unwrap()[10 * 4..10 * 4 + 4];
        assert!(st_left[0] > 230 && st_left[1] < 80, "st_left={st_left:?}");
        // Right end of translated stroke at device (54, 32) is strongly green
        let st_right = &surf.row(32).unwrap()[54 * 4..54 * 4 + 4];
        assert!(st_right[1] > 230 && st_right[0] < 80, "st_right={st_right:?}");

        // 3. Partial negative-sweep conic gradient + transparent stop fade without black halo
        let conic = ConicGradient::new_sweep(
            Point::new(16.0, 16.0),
            0.0,
            -core::f32::consts::FRAC_PI_2,
        )
        .add_stop(0.0, Color::RED)
        .add_stop(1.0, Color::TRANSPARENT)
        .with_interpolation(InterpolationSpace::OklchShorter)
        .with_spread(SpreadMode::Pad);
        let prep = PreparedGradient::prepare_conic(&conic, Affine2D::IDENTITY, 1.0, false).unwrap();
        assert!(!prep.is_opaque());
        let mut px = [LinearRgba16::TRANSPARENT; 1];
        // Angle -PI/4 is halfway along negative sweep [0, -PI/2] -> ~50% alpha, pure red hue!
        prep.eval_span_u16(26, 6, &mut px);
        assert!(px[0].a > 20000 && px[0].a < 45000, "px={:?}", px[0]);
        assert!(px[0].r > px[0].a / 2 && px[0].g < 1000 && px[0].b < 1000, "px={:?}", px[0]);
    }
}
