#![forbid(unsafe_code)]

use alloc::vec::Vec;

use crate::geom::bounds::{cubic_bounds, quad_bounds};
use crate::geom::dash::dash_path_into;
use crate::geom::path::{Path, PathBuilder, PathEvent};
use crate::math::affine::{Affine2D, Point, Rect, Vec2};
use crate::math::float::{abs_f64, atan2_f32, ceil_f32, copysign_f64, sincos_f32, sqrt_f64};
use core::f64::consts::{FRAC_PI_2, PI};

const MAX_OFFSET_DEPTH: u32 = 10;
const EPS_T: f64 = 1e-6;
const EPS_LEN_SQ: f64 = 1e-18;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LineCap {
    #[default]
    Butt,
    Round,
    Square,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum LineJoin {
    #[default]
    Miter,
    Round,
    Bevel,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StrokeStyle<'a> {
    pub width: f32,
    pub cap: LineCap,
    pub join: LineJoin,
    pub miter_limit: f32,
    pub dash_array: Option<&'a [f32]>,
    pub dash_offset: f32,
    pub non_scaling: bool,
}

impl Default for StrokeStyle<'_> {
    #[inline]
    fn default() -> Self {
        Self::new(1.0)
    }
}

impl<'a> StrokeStyle<'a> {
    #[inline]
    pub const fn new(width: f32) -> Self {
        Self {
            width,
            cap: LineCap::Butt,
            join: LineJoin::Miter,
            miter_limit: 4.0,
            dash_array: None,
            dash_offset: 0.0,
            non_scaling: false,
        }
    }

    #[inline]
    pub const fn with_cap(mut self, cap: LineCap) -> Self {
        self.cap = cap;
        self
    }

    #[inline]
    pub const fn with_join(mut self, join: LineJoin) -> Self {
        self.join = join;
        self
    }

    #[inline]
    pub const fn with_miter_limit(mut self, miter_limit: f32) -> Self {
        self.miter_limit = miter_limit;
        self
    }

    #[inline]
    pub const fn with_dashes(mut self, dash_array: &'a [f32], dash_offset: f32) -> Self {
        self.dash_array = Some(dash_array);
        self.dash_offset = dash_offset;
        self
    }

    #[inline]
    pub const fn with_non_scaling(mut self, non_scaling: bool) -> Self {
        self.non_scaling = non_scaling;
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct QuadSeg {
    pub p0: Point,
    pub p1: Point,
    pub p2: Point,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CubicSeg {
    pub p0: Point,
    pub p1: Point,
    pub p2: Point,
    pub p3: Point,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OffsetSegment {
    LineTo(Point),
    QuadTo(Point, Point),
    CubicTo(Point, Point, Point),
}

#[derive(Debug, Clone, Copy)]
struct DVec2 {
    x: f64,
    y: f64,
}

impl DVec2 {
    #[inline]
    const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    fn from_point(p: Point) -> Self {
        Self {
            x: p.x as f64,
            y: p.y as f64,
        }
    }

    #[inline]
    fn to_point(self) -> Point {
        Point::new(self.x as f32, self.y as f32)
    }

    #[inline]
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y)
    }

    #[inline]
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y)
    }

    #[inline]
    fn scale(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s)
    }

    #[inline]
    fn lerp(self, o: Self, t: f64) -> Self {
        Self::new(self.x + (o.x - self.x) * t, self.y + (o.y - self.y) * t)
    }

    #[inline]
    fn dot(self, o: Self) -> f64 {
        self.x * o.x + self.y * o.y
    }

    #[inline]
    fn cross(self, o: Self) -> f64 {
        self.x * o.y - self.y * o.x
    }

    #[inline]
    fn len_sq(self) -> f64 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    fn dist_sq(self, o: Self) -> f64 {
        self.sub(o).len_sq()
    }

    #[inline]
    fn normalize(self) -> Option<Self> {
        let l2 = self.len_sq();
        if !l2.is_finite() || l2 <= EPS_LEN_SQ {
            return None;
        }
        let inv = 1.0 / sqrt_f64(l2);
        Some(self.scale(inv))
    }

    #[inline]
    fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }
}

// Solves a*t^2 + b*t + c = 0 in (0, 1) using the numerically stable Citardauq formula.
fn solve_quadratic_unit(a: f64, b: f64, c: f64, out: &mut [f64], len: &mut usize) {
    let scale = abs_f64(a) + abs_f64(b) + abs_f64(c);
    if scale <= 1e-18 {
        return;
    }
    if abs_f64(a) <= 1e-12 * scale {
        if abs_f64(b) > 1e-15 * scale {
            push_unit_param(-c / b, out, len);
        }
        return;
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return;
    }
    let s = sqrt_f64(disc);
    let q = -0.5 * (b + copysign_f64(s, b));
    if abs_f64(q) > 1e-18 {
        push_unit_param(q / a, out, len);
        push_unit_param(c / q, out, len);
    } else {
        push_unit_param(-0.5 * b / a, out, len);
    }
}

#[inline]
fn push_unit_param(t: f64, out: &mut [f64], len: &mut usize) {
    if !t.is_finite() || t <= EPS_T || t >= 1.0 - EPS_T {
        return;
    }
    if *len < out.len() {
        out[*len] = t;
        *len += 1;
    }
}

fn sort_dedup_params(params: &mut [f64], len: &mut usize) {
    // Insertion sort on <= 6 elements
    for i in 1..*len {
        let key = params[i];
        let mut j = i;
        while j > 0 && params[j - 1] > key {
            params[j] = params[j - 1];
            j -= 1;
        }
        params[j] = key;
    }
    let mut write = 0usize;
    for i in 0..*len {
        if write == 0 || params[i] - params[write - 1] > EPS_T {
            params[write] = params[i];
            write += 1;
        }
    }
    *len = write;
}

impl QuadSeg {
    #[inline]
    pub const fn new(p0: Point, p1: Point, p2: Point) -> Self {
        Self { p0, p1, p2 }
    }

    #[inline]
    pub fn is_degenerate(&self) -> bool {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        p0.dist_sq(p1) <= EPS_LEN_SQ && p1.dist_sq(p2) <= EPS_LEN_SQ
    }

    #[inline]
    pub fn eval(&self, t: f64) -> Point {
        let mt = 1.0 - t;
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        p0.scale(mt * mt)
            .add(p1.scale(2.0 * mt * t))
            .add(p2.scale(t * t))
            .to_point()
    }

    #[inline]
    pub fn tangent(&self, t: f64) -> Vec2 {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let d0 = p1.sub(p0);
        let d1 = p2.sub(p1);
        let v = d0.lerp(d1, t).scale(2.0);
        Vec2::new(v.x as f32, v.y as f32)
    }

    // Degenerate-safe unit start tangent (L'Hopital fallback when P0 == P1).
    pub fn start_unit_tangent(&self) -> Option<Vec2> {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        p1.sub(p0)
            .normalize()
            .or_else(|| p2.sub(p0).normalize())
            .map(|v| Vec2::new(v.x as f32, v.y as f32))
    }

    // Degenerate-safe unit end tangent (L'Hopital fallback when P1 == P2).
    pub fn end_unit_tangent(&self) -> Option<Vec2> {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        p2.sub(p1)
            .normalize()
            .or_else(|| p2.sub(p0).normalize())
            .map(|v| Vec2::new(v.x as f32, v.y as f32))
    }

    #[inline]
    pub fn split_at(&self, t: f64) -> (Self, Self) {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let q0 = p0.lerp(p1, t);
        let q1 = p1.lerp(p2, t);
        let m = q0.lerp(q1, t);
        let mp = m.to_point();
        (
            Self::new(self.p0, q0.to_point(), mp),
            Self::new(mp, q1.to_point(), self.p2),
        )
    }

    #[inline]
    pub fn bounds(&self) -> Rect {
        quad_bounds(self.p0, self.p1, self.p2)
    }
}

impl CubicSeg {
    #[inline]
    pub const fn new(p0: Point, p1: Point, p2: Point, p3: Point) -> Self {
        Self { p0, p1, p2, p3 }
    }

    #[inline]
    pub fn is_degenerate(&self) -> bool {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        p0.dist_sq(p1) <= EPS_LEN_SQ
            && p1.dist_sq(p2) <= EPS_LEN_SQ
            && p2.dist_sq(p3) <= EPS_LEN_SQ
    }

    #[inline]
    pub fn eval(&self, t: f64) -> Point {
        let mt = 1.0 - t;
        let mt2 = mt * mt;
        let t2 = t * t;
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        p0.scale(mt2 * mt)
            .add(p1.scale(3.0 * mt2 * t))
            .add(p2.scale(3.0 * mt * t2))
            .add(p3.scale(t2 * t))
            .to_point()
    }

    #[inline]
    pub fn tangent(&self, t: f64) -> Vec2 {
        let mt = 1.0 - t;
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        let v0 = p1.sub(p0);
        let v1 = p2.sub(p1);
        let v2 = p3.sub(p2);
        let d = v0
            .scale(mt * mt)
            .add(v1.scale(2.0 * mt * t))
            .add(v2.scale(t * t))
            .scale(3.0);
        Vec2::new(d.x as f32, d.y as f32)
    }

    // Degenerate-safe unit start tangent (L'Hopital fallback through P1 -> P2 -> P3).
    pub fn start_unit_tangent(&self) -> Option<Vec2> {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        p1.sub(p0)
            .normalize()
            .or_else(|| p2.sub(p0).normalize())
            .or_else(|| p3.sub(p0).normalize())
            .map(|v| Vec2::new(v.x as f32, v.y as f32))
    }

    // Degenerate-safe unit end tangent (L'Hopital fallback through P2 -> P1 -> P0).
    pub fn end_unit_tangent(&self) -> Option<Vec2> {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        p3.sub(p2)
            .normalize()
            .or_else(|| p3.sub(p1).normalize())
            .or_else(|| p3.sub(p0).normalize())
            .map(|v| Vec2::new(v.x as f32, v.y as f32))
    }

    #[inline]
    pub fn split_at(&self, t: f64) -> (Self, Self) {
        let p0 = DVec2::from_point(self.p0);
        let p1 = DVec2::from_point(self.p1);
        let p2 = DVec2::from_point(self.p2);
        let p3 = DVec2::from_point(self.p3);
        let q0 = p0.lerp(p1, t);
        let q1 = p1.lerp(p2, t);
        let q2 = p2.lerp(p3, t);
        let r0 = q0.lerp(q1, t);
        let r1 = q1.lerp(q2, t);
        let m = r0.lerp(r1, t);
        let mp = m.to_point();
        (
            Self::new(self.p0, q0.to_point(), r0.to_point(), mp),
            Self::new(mp, r1.to_point(), q2.to_point(), self.p3),
        )
    }

    #[inline]
    pub fn bounds(&self) -> Rect {
        cubic_bounds(self.p0, self.p1, self.p2, self.p3)
    }
}

// Analytical quadratic max-curvature / fold-back cusp apex and x/y tangent extrema in (0, 1).
pub fn quad_cusp_and_extrema_params(q: QuadSeg) -> ([f64; 3], usize) {
    let p0 = DVec2::from_point(q.p0);
    let p1 = DVec2::from_point(q.p1);
    let p2 = DVec2::from_point(q.p2);
    let b = p1.sub(p0);
    let a = p2.sub(p1).sub(b); // P0 - 2*P1 + P2

    let mut params = [0.0f64; 3];
    let mut len = 0usize;

    let a_len2 = a.len_sq();
    if a_len2 > EPS_LEN_SQ {
        push_unit_param(-b.dot(a) / a_len2, &mut params, &mut len);
    }
    if abs_f64(a.x) > 1e-12 {
        push_unit_param(-b.x / a.x, &mut params, &mut len);
    }
    if abs_f64(a.y) > 1e-12 {
        push_unit_param(-b.y / a.y, &mut params, &mut len);
    }

    sort_dedup_params(&mut params, &mut len);
    (params, len)
}

// Analytical cubic inflections, cusps, near-cusp loop apex, and x/y tangent extrema in (0, 1).
pub fn cubic_inflection_and_cusp_params(c: CubicSeg) -> ([f64; 6], usize) {
    let p0 = DVec2::from_point(c.p0);
    let p1 = DVec2::from_point(c.p1);
    let p2 = DVec2::from_point(c.p2);
    let p3 = DVec2::from_point(c.p3);

    let v0 = p1.sub(p0);
    let v1 = p2.sub(p1);
    let v2 = p3.sub(p2);

    // B'(t) = 3*(A*t^2 + 2*D*t + C), B''(t) = 6*(A*t + D)
    let coeff_c = v0;
    let coeff_d = v1.sub(v0);
    let coeff_a = v2.sub(v1).sub(coeff_d);

    let mut params = [0.0f64; 6];
    let mut len = 0usize;

    // 1. Inflections & cusps: det(B'(t), B''(t)) = 0 -> det(D, A)*t^2 + det(C, A)*t + det(C, D) = 0
    let qa = coeff_d.cross(coeff_a);
    let qb = coeff_c.cross(coeff_a);
    let qc = coeff_c.cross(coeff_d);
    let prev_len = len;
    solve_quadratic_unit(qa, qb, qc, &mut params, &mut len);

    // If discriminant < 0 (tight loop / near-cusp without real inflection), split at quadratic vertex.
    if len == prev_len && abs_f64(qa) > 1e-15 {
        let disc = qb * qb - 4.0 * qa * qc;
        if disc < 0.0 {
            push_unit_param(-0.5 * qb / qa, &mut params, &mut len);
        }
    }

    // 2. X and Y tangent extrema: B'_x(t) = 0 and B'_y(t) = 0
    solve_quadratic_unit(coeff_a.x, 2.0 * coeff_d.x, coeff_c.x, &mut params, &mut len);
    solve_quadratic_unit(coeff_a.y, 2.0 * coeff_d.y, coeff_c.y, &mut params, &mut len);

    sort_dedup_params(&mut params, &mut len);
    (params, len)
}

// Splits a quadratic into strictly convex, x/y-monotonic sub-quadratics via exact de Casteljau.
pub fn decompose_quad_for_stroke(q: QuadSeg, out: &mut Vec<QuadSeg>) {
    if q.is_degenerate() {
        return;
    }
    let (params, len) = quad_cusp_and_extrema_params(q);
    let mut rem = q;
    let mut prev_t = 0.0f64;
    for &t in &params[..len] {
        let local_t = (t - prev_t) / (1.0 - prev_t);
        if local_t > EPS_T && local_t < 1.0 - EPS_T {
            let (left, right) = rem.split_at(local_t);
            if !left.is_degenerate() {
                out.push(left);
            }
            rem = right;
            prev_t = t;
        }
    }
    if !rem.is_degenerate() {
        out.push(rem);
    }
}

// Splits a cubic into inflection-free, cusp-isolated, x/y-monotonic sub-cubics via de Casteljau.
pub fn decompose_cubic_for_stroke(c: CubicSeg, out: &mut Vec<CubicSeg>) {
    if c.is_degenerate() {
        return;
    }
    let (params, len) = cubic_inflection_and_cusp_params(c);
    let mut rem = c;
    let mut prev_t = 0.0f64;
    for &t in &params[..len] {
        let local_t = (t - prev_t) / (1.0 - prev_t);
        if local_t > EPS_T && local_t < 1.0 - EPS_T {
            let (left, right) = rem.split_at(local_t);
            if !left.is_degenerate() {
                out.push(left);
            }
            rem = right;
            prev_t = t;
        }
    }
    if !rem.is_degenerate() {
        out.push(rem);
    }
}

#[inline]
fn unit_normal_from_vec(t: Vec2) -> Option<DVec2> {
    DVec2::new(t.x as f64, t.y as f64).normalize().map(DVec2::perp)
}

#[inline]
fn eval_quad_offset(q: QuadSeg, t: f64, r: f64, fallback_n: DVec2) -> DVec2 {
    let p = DVec2::from_point(q.eval(t));
    let n = unit_normal_from_vec(q.tangent(t)).unwrap_or(fallback_n);
    p.add(n.scale(r))
}

#[inline]
fn eval_cubic_offset(c: CubicSeg, t: f64, r: f64, fallback_n: DVec2) -> DVec2 {
    let p = DVec2::from_point(c.eval(t));
    let n = unit_normal_from_vec(c.tangent(t)).unwrap_or(fallback_n);
    p.add(n.scale(r))
}

#[inline]
fn eval_quad_dvec(o0: DVec2, ctrl: DVec2, o1: DVec2, t: f64) -> DVec2 {
    let mt = 1.0 - t;
    o0.scale(mt * mt)
        .add(ctrl.scale(2.0 * mt * t))
        .add(o1.scale(t * t))
}

#[inline]
fn rect_contains_with_margin(b: Rect, p: DVec2, margin: f64) -> bool {
    p.x >= (b.min_x as f64) - margin
        && p.x <= (b.max_x as f64) + margin
        && p.y >= (b.min_y as f64) - margin
        && p.y <= (b.max_y as f64) + margin
}

fn offset_quad_recursive(
    q: QuadSeg,
    r: f64,
    tol_sq: f64,
    depth: u32,
    out: &mut Vec<OffsetSegment>,
) {
    let Some(t0) = q.start_unit_tangent() else {
        return;
    };
    let Some(t1) = q.end_unit_tangent() else {
        return;
    };
    let n0 = DVec2::new(-(t0.y as f64), t0.x as f64);
    let n1 = DVec2::new(-(t1.y as f64), t1.x as f64);
    let o0 = DVec2::from_point(q.p0).add(n0.scale(r));
    let o1 = DVec2::from_point(q.p2).add(n1.scale(r));

    if depth >= MAX_OFFSET_DEPTH {
        out.push(OffsetSegment::LineTo(o1.to_point()));
        return;
    }

    let mid_n = n0.add(n1).normalize().unwrap_or(n0);
    let om = eval_quad_offset(q, 0.5, r, mid_n);
    let o25 = eval_quad_offset(q, 0.25, r, mid_n);
    let o75 = eval_quad_offset(q, 0.75, r, mid_n);

    // Check flat chord approximation first
    let chord_mid = o0.lerp(o1, 0.5);
    let chord_25 = o0.lerp(o1, 0.25);
    let chord_75 = o0.lerp(o1, 0.75);
    if om.dist_sq(chord_mid) <= tol_sq
        && o25.dist_sq(chord_25) <= tol_sq
        && o75.dist_sq(chord_75) <= tol_sq
    {
        out.push(OffsetSegment::LineTo(o1.to_point()));
        return;
    }

    // Fit quadratic passing through o0 (t=0), om (t=0.5), o1 (t=1)
    let ctrl = om.scale(2.0).sub(chord_mid);
    let dot_t = (t0.x as f64) * (t1.x as f64) + (t0.y as f64) * (t1.y as f64);
    if dot_t >= 0.5 {
        let q25 = eval_quad_dvec(o0, ctrl, o1, 0.25);
        let q75 = eval_quad_dvec(o0, ctrl, o1, 0.75);
        let qb = quad_bounds(o0.to_point(), ctrl.to_point(), o1.to_point());
        let cb = q.bounds();
        let bound_margin = abs_f64(r) + sqrt_f64(tol_sq) + 1e-3;
        let in_bounds = rect_contains_with_margin(cb, DVec2::new(qb.min_x as f64, qb.min_y as f64), bound_margin)
            && rect_contains_with_margin(cb, DVec2::new(qb.max_x as f64, qb.max_y as f64), bound_margin);
        if in_bounds && q25.dist_sq(o25) <= tol_sq && q75.dist_sq(o75) <= tol_sq {
            out.push(OffsetSegment::QuadTo(ctrl.to_point(), o1.to_point()));
            return;
        }
    }

    let (left, right) = q.split_at(0.5);
    offset_quad_recursive(left, r, tol_sq, depth + 1, out);
    offset_quad_recursive(right, r, tol_sq, depth + 1, out);
}

fn offset_cubic_recursive(
    c: CubicSeg,
    r: f64,
    tol_sq: f64,
    depth: u32,
    out: &mut Vec<OffsetSegment>,
) {
    let Some(t0) = c.start_unit_tangent() else {
        return;
    };
    let Some(t1) = c.end_unit_tangent() else {
        return;
    };
    let n0 = DVec2::new(-(t0.y as f64), t0.x as f64);
    let n1 = DVec2::new(-(t1.y as f64), t1.x as f64);
    let o0 = DVec2::from_point(c.p0).add(n0.scale(r));
    let o1 = DVec2::from_point(c.p3).add(n1.scale(r));

    if depth >= MAX_OFFSET_DEPTH {
        out.push(OffsetSegment::LineTo(o1.to_point()));
        return;
    }

    let mid_n = n0.add(n1).normalize().unwrap_or(n0);
    let om = eval_cubic_offset(c, 0.5, r, mid_n);
    let o25 = eval_cubic_offset(c, 0.25, r, mid_n);
    let o75 = eval_cubic_offset(c, 0.75, r, mid_n);

    // Check flat chord approximation first
    let chord_mid = o0.lerp(o1, 0.5);
    let chord_25 = o0.lerp(o1, 0.25);
    let chord_75 = o0.lerp(o1, 0.75);
    if om.dist_sq(chord_mid) <= tol_sq
        && o25.dist_sq(chord_25) <= tol_sq
        && o75.dist_sq(chord_75) <= tol_sq
    {
        out.push(OffsetSegment::LineTo(o1.to_point()));
        return;
    }

    // Fit bounded quadratic passing through o0, om, o1
    let ctrl = om.scale(2.0).sub(chord_mid);
    let dot_t = (t0.x as f64) * (t1.x as f64) + (t0.y as f64) * (t1.y as f64);
    if dot_t >= 0.5 {
        let q25 = eval_quad_dvec(o0, ctrl, o1, 0.25);
        let q75 = eval_quad_dvec(o0, ctrl, o1, 0.75);
        let qb = quad_bounds(o0.to_point(), ctrl.to_point(), o1.to_point());
        let cb = c.bounds();
        let bound_margin = abs_f64(r) + sqrt_f64(tol_sq) + 1e-3;
        let in_bounds = rect_contains_with_margin(cb, DVec2::new(qb.min_x as f64, qb.min_y as f64), bound_margin)
            && rect_contains_with_margin(cb, DVec2::new(qb.max_x as f64, qb.max_y as f64), bound_margin);
        if in_bounds && q25.dist_sq(o25) <= tol_sq && q75.dist_sq(o75) <= tol_sq {
            out.push(OffsetSegment::QuadTo(ctrl.to_point(), o1.to_point()));
            return;
        }
    }

    let (left, right) = c.split_at(0.5);
    offset_cubic_recursive(left, r, tol_sq, depth + 1, out);
    offset_cubic_recursive(right, r, tol_sq, depth + 1, out);
}

// Emits adaptive parallel offset segments for a decomposed quadratic sub-curve.
// Returns the exact start point of the offset curve (if the curve is non-degenerate).
pub fn offset_quad_subcurve(
    q: QuadSeg,
    signed_half_width: f32,
    tolerance: f32,
    out: &mut Vec<OffsetSegment>,
) -> Option<Point> {
    let t0 = q.start_unit_tangent()?;
    let r = signed_half_width as f64;
    let n0 = DVec2::new(-(t0.y as f64), t0.x as f64);
    let start = DVec2::from_point(q.p0).add(n0.scale(r)).to_point();
    let tol = (tolerance.max(1e-4)) as f64;
    offset_quad_recursive(q, r, tol * tol, 0, out);
    Some(start)
}

// Emits adaptive parallel offset segments for a decomposed cubic sub-curve.
// Returns the exact start point of the offset curve (if the curve is non-degenerate).
pub fn offset_cubic_subcurve(
    c: CubicSeg,
    signed_half_width: f32,
    tolerance: f32,
    out: &mut Vec<OffsetSegment>,
) -> Option<Point> {
    let t0 = c.start_unit_tangent()?;
    let r = signed_half_width as f64;
    let n0 = DVec2::new(-(t0.y as f64), t0.x as f64);
    let start = DVec2::from_point(c.p0).add(n0.scale(r)).to_point();
    let tol = (tolerance.max(1e-4)) as f64;
    offset_cubic_recursive(c, r, tol * tol, 0, out);
    Some(start)
}

// Reverses a sequence of offset segments starting at `start` into a PathBuilder.
fn emit_reversed_offset(
    start: Point,
    segs: &[OffsetSegment],
    out: &mut PathBuilder,
    xform: &dyn Fn(Point) -> Point,
) {
    let mut i = segs.len();
    while i > 0 {
        i -= 1;
        let prev_pt = if i == 0 {
            start
        } else {
            offset_seg_end(segs[i - 1])
        };
        match segs[i] {
            OffsetSegment::LineTo(_) => {
                let p = xform(prev_pt);
                out.line_to(p.x, p.y);
            }
            OffsetSegment::QuadTo(ctrl, _) => {
                let c = xform(ctrl);
                let p = xform(prev_pt);
                out.quad_to(c.x, c.y, p.x, p.y);
            }
            OffsetSegment::CubicTo(c1, c2, _) => {
                let tc2 = xform(c2);
                let tc1 = xform(c1);
                let p = xform(prev_pt);
                out.cubic_to(tc2.x, tc2.y, tc1.x, tc1.y, p.x, p.y);
            }
        }
    }
}

#[inline]
fn offset_seg_end(seg: OffsetSegment) -> Point {
    match seg {
        OffsetSegment::LineTo(p)
        | OffsetSegment::QuadTo(_, p)
        | OffsetSegment::CubicTo(_, _, p) => p,
    }
}

fn emit_forward_offset(
    segs: &[OffsetSegment],
    out: &mut PathBuilder,
    xform: &dyn Fn(Point) -> Point,
) {
    for &seg in segs {
        match seg {
            OffsetSegment::LineTo(p) => {
                let tp = xform(p);
                out.line_to(tp.x, tp.y);
            }
            OffsetSegment::QuadTo(c, p) => {
                let tc = xform(c);
                let tp = xform(p);
                out.quad_to(tc.x, tc.y, tp.x, tp.y);
            }
            OffsetSegment::CubicTo(c1, c2, p) => {
                let tc1 = xform(c1);
                let tc2 = xform(c2);
                let tp = xform(p);
                out.cubic_to(tc1.x, tc1.y, tc2.x, tc2.y, tp.x, tp.y);
            }
        }
    }
}

// Emits <= 90-deg cubic Bezier arc segments rotating unit vector `u0` by `sweep` radians around `center`.
fn emit_arc_segments(
    center: DVec2,
    r: f64,
    u0: DVec2,
    u_end: DVec2,
    sweep: f64,
    out: &mut Vec<OffsetSegment>,
) {
    let abs_sweep = abs_f64(sweep);
    if abs_sweep < 1e-6 {
        out.push(OffsetSegment::LineTo(center.add(u_end.scale(r)).to_point()));
        return;
    }
    let segs = (ceil_f32((abs_sweep / FRAC_PI_2) as f32) as usize).clamp(1, 4);
    let delta = sweep / (segs as f64);
    let (sh, ch) = sincos_f32((0.25 * delta) as f32);
    let alpha = (4.0 / 3.0) * ((sh as f64) / (ch as f64));
    let u0p = u0.perp();

    let rot = |ang: f64| -> DVec2 {
        let (s, c) = sincos_f32(ang as f32);
        u0.scale(c as f64).add(u0p.scale(s as f64))
    };

    let mut v0 = u0;
    for i in 0..segs {
        let v1 = if i + 1 == segs {
            u_end
        } else {
            rot(((i + 1) as f64) * delta)
        };
        let p0 = center.add(v0.scale(r));
        let p3 = center.add(v1.scale(r));
        let c1 = p0.add(v0.perp().scale(r * alpha));
        let c2 = p3.sub(v1.perp().scale(r * alpha));
        out.push(OffsetSegment::CubicTo(
            c1.to_point(),
            c2.to_point(),
            p3.to_point(),
        ));
        v0 = v1;
    }
}

fn emit_arc_path(
    center: DVec2,
    r: f64,
    u0: DVec2,
    u_end: DVec2,
    sweep: f64,
    out: &mut PathBuilder,
    xform: &dyn Fn(Point) -> Point,
) {
    let mut segs = Vec::with_capacity(4);
    emit_arc_segments(center, r, u0, u_end, sweep, &mut segs);
    emit_forward_offset(&segs, out, xform);
}

#[allow(clippy::too_many_arguments)]
fn emit_cap(
    pivot: DVec2,
    outward_tangent: DVec2,
    from_dir: DVec2,
    to_dir: DVec2,
    r: f64,
    cap: LineCap,
    out: &mut PathBuilder,
    xform: &dyn Fn(Point) -> Point,
) {
    match cap {
        LineCap::Butt => {
            let end_p = xform(pivot.add(to_dir.scale(r)).to_point());
            out.line_to(end_p.x, end_p.y);
        }
        LineCap::Square => {
            let ext = outward_tangent.scale(r);
            let p1 = xform(pivot.add(from_dir.scale(r)).add(ext).to_point());
            let p2 = xform(pivot.add(to_dir.scale(r)).add(ext).to_point());
            let p3 = xform(pivot.add(to_dir.scale(r)).to_point());
            out.line_to(p1.x, p1.y);
            out.line_to(p2.x, p2.y);
            out.line_to(p3.x, p3.y);
        }
        LineCap::Round => {
            // Both end cap (+N -> -N via +T) and start cap (-N -> +N via -T) sweep -PI.
            emit_arc_path(pivot, r, from_dir, to_dir, -PI, out, xform);
        }
    }
}

fn emit_zero_length_cap(
    pivot: Point,
    r: f64,
    cap: LineCap,
    out: &mut PathBuilder,
    xform: &dyn Fn(Point) -> Point,
) {
    match cap {
        LineCap::Butt => {}
        LineCap::Square => {
            let c = DVec2::from_point(pivot);
            let p0 = xform(DVec2::new(c.x - r, c.y - r).to_point());
            let p1 = xform(DVec2::new(c.x + r, c.y - r).to_point());
            let p2 = xform(DVec2::new(c.x + r, c.y + r).to_point());
            let p3 = xform(DVec2::new(c.x - r, c.y + r).to_point());
            out.move_to(p0.x, p0.y)
                .line_to(p1.x, p1.y)
                .line_to(p2.x, p2.y)
                .line_to(p3.x, p3.y)
                .close();
        }
        LineCap::Round => {
            let c = DVec2::from_point(pivot);
            let start = xform(DVec2::new(c.x + r, c.y).to_point());
            out.move_to(start.x, start.y);
            emit_arc_path(
                c,
                r,
                DVec2::new(1.0, 0.0),
                DVec2::new(-1.0, 0.0),
                PI,
                out,
                xform,
            );
            emit_arc_path(
                c,
                r,
                DVec2::new(-1.0, 0.0),
                DVec2::new(1.0, 0.0),
                PI,
                out,
                xform,
            );
            out.close();
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn emit_outer_join(
    pivot: DVec2,
    r: f64,
    u0: DVec2,
    u1: DVec2,
    dot: f64,
    sweep: f64,
    join: LineJoin,
    miter_limit_sq: f64,
    out: &mut Vec<OffsetSegment>,
) {
    let target = pivot.add(u1.scale(r)).to_point();
    match join {
        LineJoin::Bevel => {
            out.push(OffsetSegment::LineTo(target));
        }
        LineJoin::Miter => {
            // Exact miter-limit check without sin/sqrt: 1/sin(theta/2) <= M <=> (1 + dot) * M^2 >= 2
            let denom = 1.0 + dot;
            if denom > 1e-12 && denom * miter_limit_sq >= 2.0 {
                let miter = pivot.add(u0.add(u1).scale(r / denom)).to_point();
                out.push(OffsetSegment::LineTo(miter));
            }
            out.push(OffsetSegment::LineTo(target));
        }
        LineJoin::Round => {
            emit_arc_segments(pivot, r, u0, u1, sweep, out);
        }
    }
}

struct SubpathStroke {
    left_start: Option<Point>,
    left: Vec<OffsetSegment>,
    right_start: Option<Point>,
    right: Vec<OffsetSegment>,
    first_pivot: Point,
    first_tangent: Option<DVec2>,
    last_pivot: Point,
    last_tangent: Option<DVec2>,
    has_any_verb: bool,
    origin: Point,
}

impl SubpathStroke {
    fn new() -> Self {
        Self {
            left_start: None,
            left: Vec::new(),
            right_start: None,
            right: Vec::new(),
            first_pivot: Point::ZERO,
            first_tangent: None,
            last_pivot: Point::ZERO,
            last_tangent: None,
            has_any_verb: false,
            origin: Point::ZERO,
        }
    }

    fn reset(&mut self, origin: Point) {
        self.left_start = None;
        self.left.clear();
        self.right_start = None;
        self.right.clear();
        self.first_pivot = origin;
        self.first_tangent = None;
        self.last_pivot = origin;
        self.last_tangent = None;
        self.has_any_verb = false;
        self.origin = origin;
    }

    fn is_empty(&self) -> bool {
        self.left_start.is_none() || self.left.is_empty()
    }

    // Emits outer join + inner bevel-to-pivot cleanup at `pivot` between `prev_t` and `next_t`.
    fn emit_join_at(
        &mut self,
        pivot: Point,
        prev_t: DVec2,
        next_t: DVec2,
        r: f64,
        join: LineJoin,
        miter_limit_sq: f64,
    ) {
        let pv = DVec2::from_point(pivot);
        let n0 = prev_t.perp();
        let n1 = next_t.perp();
        let dot = prev_t.dot(next_t).clamp(-1.0, 1.0);
        let cross = prev_t.cross(next_t);

        // Collinear continuation (0-deg turn): offset endpoints already coincide
        if dot > 1.0 - 1e-6 {
            return;
        }

        // 180-deg fold-back U-turn (cross ~= 0, dot ~= -1): inner connects via pivot, outer bevels or rounds
        if abs_f64(cross) <= 1e-9 && dot < 0.0 {
            let l1 = pv.add(n1.scale(r)).to_point();
            let r1 = pv.sub(n1.scale(r)).to_point();
            if join == LineJoin::Round {
                emit_arc_segments(pv, r, n0, n1, -PI, &mut self.left);
                // Inner connects through pivot to prevent negative-winding fold holes
                self.right.push(OffsetSegment::LineTo(pivot));
                self.right.push(OffsetSegment::LineTo(r1));
            } else {
                self.left.push(OffsetSegment::LineTo(pivot));
                self.left.push(OffsetSegment::LineTo(l1));
                self.right.push(OffsetSegment::LineTo(pivot));
                self.right.push(OffsetSegment::LineTo(r1));
            }
            return;
        }

        let sweep = atan2_f32(cross as f32, dot as f32) as f64;
        if cross > 0.0 {
            // Right side (-N) is OUTER; Left side (+N) is INNER (bevel-to-pivot cleanup)
            let l1 = pv.add(n1.scale(r)).to_point();
            self.left.push(OffsetSegment::LineTo(pivot));
            self.left.push(OffsetSegment::LineTo(l1));
            emit_outer_join(
                pv,
                r,
                n0.scale(-1.0),
                n1.scale(-1.0),
                dot,
                sweep,
                join,
                miter_limit_sq,
                &mut self.right,
            );
        } else {
            // Left side (+N) is OUTER; Right side (-N) is INNER (bevel-to-pivot cleanup)
            let r1 = pv.sub(n1.scale(r)).to_point();
            emit_outer_join(pv, r, n0, n1, dot, sweep, join, miter_limit_sq, &mut self.left);
            self.right.push(OffsetSegment::LineTo(pivot));
            self.right.push(OffsetSegment::LineTo(r1));
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn connect_segment_start(
        &mut self,
        start_pt: Point,
        start_t: DVec2,
        ls: Point,
        rs: Point,
        r: f64,
        join: LineJoin,
        miter_limit_sq: f64,
    ) {
        if let Some(prev_t) = self.last_tangent {
            self.emit_join_at(start_pt, prev_t, start_t, r, join, miter_limit_sq);
        } else {
            self.left_start = Some(ls);
            self.right_start = Some(rs);
            self.first_pivot = start_pt;
            self.first_tangent = Some(start_t);
        }
    }

    fn push_line(&mut self, a: Point, b: Point, r: f64, join: LineJoin, miter_limit_sq: f64) {
        self.has_any_verb = true;
        let da = DVec2::from_point(a);
        let db = DVec2::from_point(b);
        let Some(dir) = db.sub(da).normalize() else {
            return;
        };
        let n = dir.perp().scale(r);
        let la = da.add(n).to_point();
        let lb = db.add(n).to_point();
        let ra = da.sub(n).to_point();
        let rb = db.sub(n).to_point();

        self.connect_segment_start(a, dir, la, ra, r, join, miter_limit_sq);
        self.left.push(OffsetSegment::LineTo(lb));
        self.right.push(OffsetSegment::LineTo(rb));
        self.last_pivot = b;
        self.last_tangent = Some(dir);
    }

    #[allow(clippy::too_many_arguments)]
    fn push_quad(
        &mut self,
        q: QuadSeg,
        half_w: f32,
        tol: f32,
        join: LineJoin,
        miter_limit_sq: f64,
        decomposed: &mut Vec<QuadSeg>,
        tmp: &mut Vec<OffsetSegment>,
    ) {
        self.has_any_verb = true;
        decomposed.clear();
        decompose_quad_for_stroke(q, decomposed);
        let r = half_w as f64;
        for &sub in decomposed.iter() {
            let (Some(t0v), Some(t1v)) = (sub.start_unit_tangent(), sub.end_unit_tangent()) else {
                continue;
            };
            let t0 = DVec2::new(t0v.x as f64, t0v.y as f64);
            let t1 = DVec2::new(t1v.x as f64, t1v.y as f64);
            tmp.clear();
            let Some(ls) = offset_quad_subcurve(sub, half_w, tol, tmp) else {
                continue;
            };
            let mut right_tmp = Vec::with_capacity(tmp.len());
            let Some(rs) = offset_quad_subcurve(sub, -half_w, tol, &mut right_tmp) else {
                continue;
            };
            self.connect_segment_start(sub.p0, t0, ls, rs, r, join, miter_limit_sq);
            self.left.extend_from_slice(tmp);
            self.right.extend_from_slice(&right_tmp);
            self.last_pivot = sub.p2;
            self.last_tangent = Some(t1);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn push_cubic(
        &mut self,
        c: CubicSeg,
        half_w: f32,
        tol: f32,
        join: LineJoin,
        miter_limit_sq: f64,
        decomposed: &mut Vec<CubicSeg>,
        tmp: &mut Vec<OffsetSegment>,
    ) {
        self.has_any_verb = true;
        decomposed.clear();
        decompose_cubic_for_stroke(c, decomposed);
        let r = half_w as f64;
        for &sub in decomposed.iter() {
            let (Some(t0v), Some(t1v)) = (sub.start_unit_tangent(), sub.end_unit_tangent()) else {
                continue;
            };
            let t0 = DVec2::new(t0v.x as f64, t0v.y as f64);
            let t1 = DVec2::new(t1v.x as f64, t1v.y as f64);
            tmp.clear();
            let Some(ls) = offset_cubic_subcurve(sub, half_w, tol, tmp) else {
                continue;
            };
            let mut right_tmp = Vec::with_capacity(tmp.len());
            let Some(rs) = offset_cubic_subcurve(sub, -half_w, tol, &mut right_tmp) else {
                continue;
            };
            self.connect_segment_start(sub.p0, t0, ls, rs, r, join, miter_limit_sq);
            self.left.extend_from_slice(tmp);
            self.right.extend_from_slice(&right_tmp);
            self.last_pivot = sub.p3;
            self.last_tangent = Some(t1);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn flush_into(
        &mut self,
        closed: bool,
        r: f64,
        cap: LineCap,
        join: LineJoin,
        miter_limit_sq: f64,
        out: &mut PathBuilder,
        xform: &dyn Fn(Point) -> Point,
    ) {
        if self.is_empty() {
            if self.has_any_verb || closed {
                emit_zero_length_cap(self.origin, r, cap, out, xform);
            }
            self.reset(Point::ZERO);
            return;
        }
        let (Some(ls), Some(rs), Some(first_t), Some(last_t)) = (
            self.left_start,
            self.right_start,
            self.first_tangent,
            self.last_tangent,
        ) else {
            self.reset(Point::ZERO);
            return;
        };

        if closed {
            // Closing seam join between last segment and first segment at first_pivot
            self.emit_join_at(self.first_pivot, last_t, first_t, r, join, miter_limit_sq);

            // Outer/Left contour forward
            let tls = xform(ls);
            out.move_to(tls.x, tls.y);
            emit_forward_offset(&self.left, out, xform);
            out.close();

            // Inner/Right contour reversed
            let right_end = self.right.last().copied().map(offset_seg_end).unwrap_or(rs);
            let tre = xform(right_end);
            out.move_to(tre.x, tre.y);
            emit_reversed_offset(rs, &self.right, out, xform);
            out.close();
        } else {
            // Open subpath: left forward -> end cap -> right reversed -> start cap -> close
            let tls = xform(ls);
            out.move_to(tls.x, tls.y);
            emit_forward_offset(&self.left, out, xform);

            let end_pivot = DVec2::from_point(self.last_pivot);
            let end_n = last_t.perp();
            emit_cap(end_pivot, last_t, end_n, end_n.scale(-1.0), r, cap, out, xform);

            emit_reversed_offset(rs, &self.right, out, xform);

            let start_pivot = DVec2::from_point(self.first_pivot);
            let start_n = first_t.perp();
            emit_cap(
                start_pivot,
                first_t.scale(-1.0),
                start_n.scale(-1.0),
                start_n,
                r,
                cap,
                out,
                xform,
            );
            out.close();
        }
        self.reset(Point::ZERO);
    }
}

// Expands a path stroke into a filled outline in device space, honoring caps, joins, and transform mode.
pub fn expand_stroke_into(
    path: &Path,
    transform: Affine2D,
    style: &StrokeStyle<'_>,
    tolerance: f32,
    out: &mut PathBuilder,
) {
    out.clear();
    if path.is_empty()
        || !transform.is_finite()
        || !style.width.is_finite()
        || style.width <= 0.0
    {
        return;
    }

    let half_w = 0.5 * style.width;
    let r = half_w as f64;
    let base_tol = tolerance.max(1e-4);
    let miter_lim = if style.miter_limit.is_finite() && style.miter_limit >= 1.0 {
        style.miter_limit as f64
    } else {
        1.0
    };
    let miter_limit_sq = miter_lim * miter_lim;

    let (in_xform, out_xform, work_tol) = if style.non_scaling {
        (transform, Affine2D::IDENTITY, base_tol)
    } else {
        let max_s = transform.max_scale_factor().max(1e-4);
        let user_tol = (base_tol / max_s).clamp(1e-5, base_tol);
        (Affine2D::IDENTITY, transform, user_tol)
    };

    // Apply arc-length dashing in the active stroke coordinate space (user-space vs non-scaling screen-space)
    let dashed_storage: Path;
    let effective_in_xform: Affine2D;
    let source_path: &Path = if let Some(dashes) = style.dash_array {
        let mut dash_pb = PathBuilder::new();
        let transformed_for_dash: Path;
        let to_dash = if !in_xform.is_identity() {
            match path.transform(in_xform) {
                Ok(p) => {
                    transformed_for_dash = p;
                    &transformed_for_dash
                }
                Err(_) => return,
            }
        } else {
            path
        };
        if dash_path_into(to_dash, dashes, style.dash_offset, &mut dash_pb) {
            dashed_storage = dash_pb.finish();
            effective_in_xform = Affine2D::IDENTITY;
            &dashed_storage
        } else {
            effective_in_xform = in_xform;
            path
        }
    } else {
        effective_in_xform = in_xform;
        path
    };

    let map_in = |p: Point| {
        if effective_in_xform.is_identity() {
            p
        } else {
            effective_in_xform.transform_point(p)
        }
    };
    let map_out = |p: Point| {
        if out_xform.is_identity() {
            p
        } else {
            out_xform.transform_point(p)
        }
    };

    let mut sub = SubpathStroke::new();
    let mut quad_buf = Vec::new();
    let mut cubic_buf = Vec::new();
    let mut tmp_segs = Vec::new();

    let mut cur = Point::ZERO;
    let mut sub_start = Point::ZERO;
    let mut in_subpath = false;

    for ev in source_path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if in_subpath {
                    sub.flush_into(false, r, style.cap, style.join, miter_limit_sq, out, &map_out);
                }
                let tp = map_in(p);
                cur = tp;
                sub_start = tp;
                in_subpath = true;
                sub.reset(tp);
            }
            PathEvent::LineTo(p) => {
                let tp = map_in(p);
                if in_subpath {
                    sub.push_line(cur, tp, r, style.join, miter_limit_sq);
                } else {
                    sub_start = tp;
                    in_subpath = true;
                    sub.reset(tp);
                }
                cur = tp;
            }
            PathEvent::QuadTo(c, p) => {
                let tc = map_in(c);
                let tp = map_in(p);
                if in_subpath {
                    sub.push_quad(
                        QuadSeg::new(cur, tc, tp),
                        half_w,
                        work_tol,
                        style.join,
                        miter_limit_sq,
                        &mut quad_buf,
                        &mut tmp_segs,
                    );
                } else {
                    sub_start = tp;
                    in_subpath = true;
                    sub.reset(tp);
                }
                cur = tp;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                let tc1 = map_in(c1);
                let tc2 = map_in(c2);
                let tp = map_in(p);
                if in_subpath {
                    sub.push_cubic(
                        CubicSeg::new(cur, tc1, tc2, tp),
                        half_w,
                        work_tol,
                        style.join,
                        miter_limit_sq,
                        &mut cubic_buf,
                        &mut tmp_segs,
                    );
                } else {
                    sub_start = tp;
                    in_subpath = true;
                    sub.reset(tp);
                }
                cur = tp;
            }
            PathEvent::Close => {
                if in_subpath {
                    if cur != sub_start {
                        sub.push_line(cur, sub_start, r, style.join, miter_limit_sq);
                    }
                    sub.flush_into(true, r, style.cap, style.join, miter_limit_sq, out, &map_out);
                    cur = sub_start;
                    in_subpath = false;
                }
            }
        }
    }

    if in_subpath {
        sub.flush_into(false, r, style.cap, style.join, miter_limit_sq, out, &map_out);
    }
}

pub fn expand_stroke_path(
    path: &Path,
    transform: Affine2D,
    style: &StrokeStyle<'_>,
    tolerance: f32,
) -> Path {
    let mut pb = PathBuilder::new();
    expand_stroke_into(path, transform, style, tolerance, &mut pb);
    pb.finish()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::bounds::path_tight_bounds;
    use crate::math::float::abs_f32;

    fn assert_path_finite_and_within(stroke_path: &Path, expected_bounds: Rect, pad: f32) {
        assert!(!stroke_path.is_empty());
        for &p in stroke_path.points() {
            assert!(p.is_finite(), "non-finite stroke point {p:?}");
        }
        let b = path_tight_bounds(stroke_path);
        assert!(b.is_finite());
        assert!(
            b.min_x >= expected_bounds.min_x - pad
                && b.max_x <= expected_bounds.max_x + pad
                && b.min_y >= expected_bounds.min_y - pad
                && b.max_y <= expected_bounds.max_y + pad,
            "stroke bounds {b:?} exceeded expected {expected_bounds:?} + pad {pad}"
        );
    }

    #[test]
    fn analytical_inflection_and_cusp_roots_exact() {
        // 1. Symmetric S-curve with exact inflection at t = 0.5
        let s_curve = CubicSeg::new(
            Point::new(0.0, 0.0),
            Point::new(0.0, 100.0),
            Point::new(100.0, -100.0),
            Point::new(100.0, 0.0),
        );
        let (params, len) = cubic_inflection_and_cusp_params(s_curve);
        assert!(params[..len].iter().any(|&t| abs_f64(t - 0.5) < 1e-6));

        // 2. Exact cusp at t = 0.5 where B'(0.5) = (0, 0)
        let cusp = CubicSeg::new(
            Point::new(0.0, 0.0),
            Point::new(100.0, 100.0),
            Point::new(0.0, 100.0),
            Point::new(100.0, 0.0),
        );
        let (cparams, clen) = cubic_inflection_and_cusp_params(cusp);
        assert!(cparams[..clen].iter().any(|&t| abs_f64(t - 0.5) < 1e-6));

        // 3. Looping cubic (P1 and P2 cross) -> no real inflection, must detect near-cusp apex at t = 0.5
        let loop_cubic = CubicSeg::new(
            Point::new(0.0, 0.0),
            Point::new(110.0, 100.0),
            Point::new(-10.0, 100.0),
            Point::new(100.0, 0.0),
        );
        let (lparams, llen) = cubic_inflection_and_cusp_params(loop_cubic);
        assert!(lparams[..llen].iter().any(|&t| abs_f64(t - 0.5) < 1e-6));

        // 4. Folded-back 180-deg U-turn quadratic -> exact turnaround at t = 0.5
        let fold_q = QuadSeg::new(
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(0.0, 0.0),
        );
        let (qparams, qlen) = quad_cusp_and_extrema_params(fold_q);
        assert_eq!(qlen, 1);
        assert!(abs_f64(qparams[0] - 0.5) < 1e-6);
    }

    #[test]
    fn torture_cusps_loops_u_turns_giant_widths_and_degenerates() {
        // 1. Hairline looping cubic (P1 and P2 crossed)
        let mut pb = PathBuilder::new();
        pb.move_to(0.0, 0.0).cubic_to(110.0, 100.0, -10.0, 100.0, 100.0, 0.0);
        let loop_path = pb.finish();
        let cb = path_tight_bounds(&loop_path);
        let st = expand_stroke_path(&loop_path, Affine2D::IDENTITY, &StrokeStyle::new(12.0), 0.25);
        assert_path_finite_and_within(&st, cb, 6.0 + 0.5);

        // 2. Folded-back 180-deg U-turn cubic
        let mut upb = PathBuilder::new();
        upb.move_to(0.0, 0.0).cubic_to(80.0, 0.0, 80.0, 0.0, 0.0, 0.0);
        let uturn = upb.finish();
        let ub = path_tight_bounds(&uturn);
        let st_u = expand_stroke_path(&uturn, Affine2D::IDENTITY, &StrokeStyle::new(10.0), 0.25);
        assert_path_finite_and_within(&st_u, ub, 5.0 + 0.5);

        // 3. Giant stroke width (w = 200 px on a 10 px curve)
        let mut gpb = PathBuilder::new();
        gpb.move_to(0.0, 0.0).cubic_to(3.0, 10.0, 7.0, 10.0, 10.0, 0.0);
        let tiny_curve = gpb.finish();
        let tb = path_tight_bounds(&tiny_curve);
        let st_giant =
            expand_stroke_path(&tiny_curve, Affine2D::IDENTITY, &StrokeStyle::new(200.0), 0.25);
        assert_path_finite_and_within(&st_giant, tb, 100.0 + 0.5);

        // 4. Degenerate zero-length handles (P0 == P1, P2 == P3, and all-coincident P0..P3)
        let mut dpb = PathBuilder::new();
        dpb.move_to(10.0, 10.0).cubic_to(10.0, 10.0, 50.0, 20.0, 50.0, 20.0);
        let d_path = dpb.finish();
        let db = path_tight_bounds(&d_path);
        let st_d = expand_stroke_path(&d_path, Affine2D::IDENTITY, &StrokeStyle::new(8.0), 0.25);
        assert_path_finite_and_within(&st_d, db, 4.0 + 0.5);

        let mut zpb = PathBuilder::new();
        zpb.move_to(10.0, 10.0).cubic_to(10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
        let z_path = zpb.finish();
        let st_z = expand_stroke_path(&z_path, Affine2D::IDENTITY, &StrokeStyle::new(8.0), 0.25);
        assert!(st_z.points().iter().all(|p| p.is_finite()));
    }

    #[test]
    fn non_uniform_scale_user_space_vs_non_scaling_stroke() {
        // Horizontal line [0, 0] -> [100, 0] and vertical line [0, 0] -> [0, 100], width = 10
        // Under scale(3.0, 0.5):
        // - user-space (non_scaling = false): horizontal stroke height = 10 * 0.5 = 5, vertical stroke width = 10 * 3.0 = 30
        // - non_scaling = true: horizontal stroke height = 10, vertical stroke width = 10
        let scale = Affine2D::scale(3.0, 0.5);

        let mut hpb = PathBuilder::new();
        hpb.move_to(0.0, 0.0).line_to(100.0, 0.0);
        let h_line = hpb.finish();

        let mut vpb = PathBuilder::new();
        vpb.move_to(0.0, 0.0).line_to(0.0, 100.0);
        let v_line = vpb.finish();

        let user_style = StrokeStyle::new(10.0).with_non_scaling(false);
        let h_user = path_tight_bounds(&expand_stroke_path(&h_line, scale, &user_style, 0.25));
        let v_user = path_tight_bounds(&expand_stroke_path(&v_line, scale, &user_style, 0.25));
        assert!(abs_f32(h_user.height() - 5.0) < 1e-3, "h_user={h_user:?}");
        assert!(abs_f32(v_user.width() - 30.0) < 1e-3, "v_user={v_user:?}");

        let non_scale_style = StrokeStyle::new(10.0).with_non_scaling(true);
        let h_ns = path_tight_bounds(&expand_stroke_path(&h_line, scale, &non_scale_style, 0.25));
        let v_ns = path_tight_bounds(&expand_stroke_path(&v_line, scale, &non_scale_style, 0.25));
        assert!(abs_f32(h_ns.height() - 10.0) < 1e-3, "h_ns={h_ns:?}");
        assert!(abs_f32(v_ns.width() - 10.0) < 1e-3, "v_ns={v_ns:?}");
    }

    #[test]
    fn circle_offset_radial_accuracy_and_100_random_cubics() {
        // 1. Circle of radius 50 stroked with width 20 -> outer radius 60, inner radius 40
        let mut cpb = PathBuilder::new();
        cpb.push_circle(0.0, 0.0, 50.0);
        let circle = cpb.finish();
        let stroked = expand_stroke_path(&circle, Affine2D::IDENTITY, &StrokeStyle::new(20.0), 0.1);
        let b = path_tight_bounds(&stroked);
        assert!(abs_f32(b.min_x - (-60.0)) < 0.25 && abs_f32(b.max_x - 60.0) < 0.25);
        assert!(abs_f32(b.min_y - (-60.0)) < 0.25 && abs_f32(b.max_y - 60.0) < 0.25);

        // Verify every decomposed sub-cubic is inflection-free and x/y-monotonic on 100 random cubics
        let mut seed = 0xD1B5_4A32_D192_ED03u64;
        let mut next_f32 = || -> f32 {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (((seed >> 40) as f32) / 8_388_608.0 - 1.0) * 100.0
        };

        let mut subs = Vec::new();
        for i in 0..100 {
            let c = CubicSeg::new(
                Point::new(next_f32(), next_f32()),
                Point::new(next_f32(), next_f32()),
                Point::new(next_f32(), next_f32()),
                Point::new(next_f32(), next_f32()),
            );
            subs.clear();
            decompose_cubic_for_stroke(c, &mut subs);
            for &sub in &subs {
                // Check x and y monotonicity across 8 interior samples
                let mut sx = 0.0f32;
                let mut sy = 0.0f32;
                for k in 1..8 {
                    let t = (k as f64) / 8.0;
                    let d = sub.tangent(t);
                    if abs_f32(d.x) > 1e-2 {
                        if sx == 0.0 {
                            sx = d.x;
                        } else {
                            assert!(sx * d.x >= -1e-2, "non-monotonic x in sub-cubic {i}");
                        }
                    }
                    if abs_f32(d.y) > 1e-2 {
                        if sy == 0.0 {
                            sy = d.y;
                        } else {
                            assert!(sy * d.y >= -1e-2, "non-monotonic y in sub-cubic {i}");
                        }
                    }
                }
            }

            let mut pb = PathBuilder::new();
            pb.move_to(c.p0.x, c.p0.y)
                .cubic_to(c.p1.x, c.p1.y, c.p2.x, c.p2.y, c.p3.x, c.p3.y);
            let p = pb.finish();
            let width = if i % 10 == 0 { 120.0 } else { 8.0 };
            let st = expand_stroke_path(&p, Affine2D::IDENTITY, &StrokeStyle::new(width), 0.25);
            assert_path_finite_and_within(&st, c.bounds(), 0.5 * width + 0.5);
        }
    }

    #[test]
    fn all_9_cap_join_combinations_angles_and_miter_boundary() {
        use crate::geom::query::point_in_fill;
        use crate::math::float::sincos_f32;

        // Test all 9 (LineCap x LineJoin) combinations across 5 deg (acute), 90 deg, 175 deg (obtuse), and 180 deg fold-back
        let angles_deg = [5.0f32, 90.0, 175.0, 180.0];
        let caps = [LineCap::Butt, LineCap::Round, LineCap::Square];
        let joins = [LineJoin::Miter, LineJoin::Round, LineJoin::Bevel];

        for &deg in &angles_deg {
            let rad = deg * (core::f32::consts::PI / 180.0);
            let (s, c) = sincos_f32(rad);
            let mut pb = PathBuilder::new();
            // Arm 1: (100, 0) -> (0, 0), Arm 2: (0, 0) -> (100*cos(deg), 100*sin(deg))
            // Wait: interior angle between the two arms is `deg`!
            pb.move_to(100.0, 0.0)
                .line_to(0.0, 0.0)
                .line_to(100.0 * c, 100.0 * s);
            let v_path = pb.finish();

            for &cap in &caps {
                for &join in &joins {
                    let style = StrokeStyle::new(20.0)
                        .with_cap(cap)
                        .with_join(join)
                        .with_miter_limit(4.0);
                    let st = expand_stroke_path(&v_path, Affine2D::IDENTITY, &style, 0.1);
                    assert!(!st.is_empty());
                    for &p in st.points() {
                        assert!(p.is_finite(), "deg={deg} cap={cap:?} join={join:?}");
                    }
                    // Vertex pivot (0, 0) must ALWAYS be covered with zero inner-cusp hole!
                    assert!(
                        point_in_fill(&st, Point::new(0.0, 0.0), crate::geom::FillRule::NonZero, 0.1),
                        "pivot hole for deg={deg} cap={cap:?} join={join:?}"
                    );
                }
            }
        }

        // Exact miter-limit transition boundary at interior angle 60 deg:
        // sin(30 deg) = 0.5 => exact miter ratio = 1 / 0.5 = 2.0, miter tip at distance 2.0 * r = 20.0 from (0,0)
        let rad60 = 60.0f32 * (core::f32::consts::PI / 180.0);
        let (s60, c60) = sincos_f32(rad60);
        let mut mpb = PathBuilder::new();
        mpb.move_to(100.0, 0.0)
            .line_to(0.0, 0.0)
            .line_to(100.0 * c60, 100.0 * s60);
        let v60 = mpb.finish();

        // Point along the outward bisector at distance 16.0 (> bevel distance ~8.66, < miter tip 20.0)
        let bis_rad = 210.0f32 * (core::f32::consts::PI / 180.0);
        let (sb, cb) = sincos_f32(bis_rad);
        let probe = Point::new(16.0 * cb, 16.0 * sb);

        let miter_ok = expand_stroke_path(
            &v60,
            Affine2D::IDENTITY,
            &StrokeStyle::new(20.0).with_join(LineJoin::Miter).with_miter_limit(2.01),
            0.1,
        );
        let miter_bevel = expand_stroke_path(
            &v60,
            Affine2D::IDENTITY,
            &StrokeStyle::new(20.0).with_join(LineJoin::Miter).with_miter_limit(1.99),
            0.1,
        );
        assert!(point_in_fill(&miter_ok, probe, crate::geom::FillRule::NonZero, 0.1));
        assert!(!point_in_fill(&miter_bevel, probe, crate::geom::FillRule::NonZero, 0.1));
    }

    #[test]
    fn zero_length_subpaths_cap_semantics() {
        let mut p_line = PathBuilder::new();
        p_line.move_to(50.0, 50.0).line_to(50.0, 50.0);
        let mut p_close = PathBuilder::new();
        p_close.move_to(50.0, 50.0).close();
        let mut p_cubic = PathBuilder::new();
        p_cubic
            .move_to(50.0, 50.0)
            .cubic_to(50.0, 50.0, 50.0, 50.0, 50.0, 50.0);

        for path in [p_line.finish(), p_close.finish(), p_cubic.finish()] {
            // Butt -> empty
            let butt = expand_stroke_path(
                &path,
                Affine2D::IDENTITY,
                &StrokeStyle::new(20.0).with_cap(LineCap::Butt),
                0.1,
            );
            assert!(butt.is_empty());

            // Square -> 20x20 square centered at (50, 50)
            let sq = expand_stroke_path(
                &path,
                Affine2D::IDENTITY,
                &StrokeStyle::new(20.0).with_cap(LineCap::Square),
                0.1,
            );
            let sb = path_tight_bounds(&sq);
            assert!(abs_f32(sb.min_x - 40.0) < 1e-3 && abs_f32(sb.max_x - 60.0) < 1e-3);
            assert!(abs_f32(sb.min_y - 40.0) < 1e-3 && abs_f32(sb.max_y - 60.0) < 1e-3);

            // Round -> circle of radius 10 centered at (50, 50)
            let rnd = expand_stroke_path(
                &path,
                Affine2D::IDENTITY,
                &StrokeStyle::new(20.0).with_cap(LineCap::Round),
                0.1,
            );
            let rb = path_tight_bounds(&rnd);
            assert!(abs_f32(rb.min_x - 40.0) < 0.05 && abs_f32(rb.max_x - 60.0) < 0.05);
            assert!(abs_f32(rb.min_y - 40.0) < 0.05 && abs_f32(rb.max_y - 60.0) < 0.05);
            assert!(crate::geom::query::point_in_fill(
                &rnd,
                Point::new(50.0, 50.0),
                crate::geom::FillRule::NonZero,
                0.1
            ));
            // Corner (59, 59) is at distance sqrt(162) ~= 12.73 > 10 -> inside square, outside round
            assert!(crate::geom::query::point_in_fill(
                &sq,
                Point::new(59.0, 59.0),
                crate::geom::FillRule::NonZero,
                0.1
            ));
            assert!(!crate::geom::query::point_in_fill(
                &rnd,
                Point::new(59.0, 59.0),
                crate::geom::FillRule::NonZero,
                0.1
            ));
        }
    }

    #[test]
    fn translucent_stroked_star_has_uniform_opacity_no_double_blend() {
        use crate::math::color::Color;
        use crate::math::dither::DitherMode;
        use crate::scene::Context;
        use crate::surface::Surface;

        // 5-point acute self-intersecting star stroked with alpha = 0.5 white over black
        let mut spb = PathBuilder::new();
        for i in 0..5 {
            let ang = ((i * 2) as f32) * (core::f32::consts::TAU / 5.0)
                - core::f32::consts::FRAC_PI_2;
            let (s, c) = crate::math::float::sincos_f32(ang);
            let x = 64.0 + 46.0 * c;
            let y = 64.0 + 46.0 * s;
            if i == 0 {
                spb.move_to(x, y);
            } else {
                spb.line_to(x, y);
            }
        }
        spb.close();
        let star = spb.finish();

        let mut surf = Surface::new_rgba8(128, 128).unwrap();
        {
            let mut ctx = Context::new(&mut surf);
            ctx.set_dither_mode(DitherMode::None);
            ctx.clear(Color::BLACK);
            ctx.set_paint(Color::new(1.0, 1.0, 1.0, 0.5));
            let style = StrokeStyle::new(8.0)
                .with_join(LineJoin::Miter)
                .with_miter_limit(10.0);
            ctx.stroke(&star, &style);
        }

        // 50% linear white over black maps to ~188 sRGB. No pixel anywhere in the framebuffer may exceed 190
        // (double-blending at vertices/intersections would yield 75% linear = ~225 sRGB!).
        let mut max_r = 0u8;
        let mut bright_count = 0usize;
        for y in 0..128u32 {
            let row = surf.row(y).unwrap();
            for px in row.chunks_exact(4) {
                if px[0] > max_r {
                    max_r = px[0];
                }
                if px[0] >= 184 {
                    bright_count += 1;
                }
            }
        }
        assert!(bright_count > 500, "stroke too sparse: {bright_count}");
        assert!(
            max_r <= 190,
            "double-blending detected at overlapping stroke joints: max_r={max_r}"
        );
    }
}
