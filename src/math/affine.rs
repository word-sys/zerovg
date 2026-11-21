#![forbid(unsafe_code)]

use core::ops::{Add, Mul, Neg, Sub};

use super::float::{
    abs_f64, ceil_f32, clamp_finite_f32, floor_f32, is_finite_pair, sincos_f32, sqrt_f32, sqrt_f64,
};

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    pub x: f32,
    pub y: f32,
}

impl Point {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        is_finite_pair(self.x, self.y)
    }

    #[inline]
    pub fn to_vec2(self) -> Vec2 {
        Vec2::new(self.x, self.y)
    }

    #[inline]
    pub fn midpoint(self, other: Self) -> Self {
        Self::new(0.5 * (self.x + other.x), 0.5 * (self.y + other.y))
    }

    #[inline]
    pub fn lerp(self, other: Self, t: f32) -> Self {
        Self::new(
            self.x + (other.x - self.x) * t,
            self.y + (other.y - self.y) * t,
        )
    }

    #[inline]
    pub fn distance_sq(self, other: Self) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        dx * dx + dy * dy
    }

    #[inline]
    pub fn distance(self, other: Self) -> f32 {
        sqrt_f32(self.distance_sq(other))
    }
}

impl Add<Vec2> for Point {
    type Output = Point;
    #[inline]
    fn add(self, rhs: Vec2) -> Self::Output {
        Point::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub<Vec2> for Point {
    type Output = Point;
    #[inline]
    fn sub(self, rhs: Vec2) -> Self::Output {
        Point::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Sub<Point> for Point {
    type Output = Vec2;
    #[inline]
    fn sub(self, rhs: Point) -> Self::Output {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point64 {
    pub x: f64,
    pub y: f64,
}

impl Point64 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn from_point(p: Point) -> Self {
        Self::new(p.x as f64, p.y as f64)
    }

    #[inline]
    pub fn to_point(self) -> Point {
        Point::new(self.x as f32, self.y as f32)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0 };

    #[inline]
    pub const fn new(x: f32, y: f32) -> Self {
        Self { x, y }
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        is_finite_pair(self.x, self.y)
    }

    #[inline]
    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }

    #[inline]
    pub fn cross(self, other: Self) -> f32 {
        self.x * other.y - self.y * other.x
    }

    #[inline]
    pub fn length_sq(self) -> f32 {
        self.x * self.x + self.y * self.y
    }

    #[inline]
    pub fn length(self) -> f32 {
        sqrt_f32(self.length_sq())
    }

    #[inline]
    pub fn normalize(self) -> Option<Self> {
        let len = self.length();
        if !len.is_finite() || len <= 1e-12 {
            return None;
        }
        let inv = 1.0 / len;
        Some(Self::new(self.x * inv, self.y * inv))
    }

    #[inline]
    pub fn perp(self) -> Self {
        Self::new(-self.y, self.x)
    }
}

impl Add for Vec2 {
    type Output = Vec2;
    #[inline]
    fn add(self, rhs: Vec2) -> Self::Output {
        Vec2::new(self.x + rhs.x, self.y + rhs.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;
    #[inline]
    fn sub(self, rhs: Vec2) -> Self::Output {
        Vec2::new(self.x - rhs.x, self.y - rhs.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Vec2;
    #[inline]
    fn mul(self, rhs: f32) -> Self::Output {
        Vec2::new(self.x * rhs, self.y * rhs)
    }
}

impl Neg for Vec2 {
    type Output = Vec2;
    #[inline]
    fn neg(self) -> Self::Output {
        Vec2::new(-self.x, -self.y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Rect {
    pub min_x: f32,
    pub min_y: f32,
    pub max_x: f32,
    pub max_y: f32,
}

impl Rect {
    pub const EMPTY: Self = Self {
        min_x: f32::INFINITY,
        min_y: f32::INFINITY,
        max_x: f32::NEG_INFINITY,
        max_y: f32::NEG_INFINITY,
    };

    #[inline]
    pub const fn new(min_x: f32, min_y: f32, max_x: f32, max_y: f32) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[inline]
    pub fn from_xywh(x: f32, y: f32, w: f32, h: f32) -> Self {
        if w <= 0.0 || h <= 0.0 || !is_finite_pair(x, y) || !is_finite_pair(w, h) {
            return Self::EMPTY;
        }
        Self::new(x, y, x + w, y + h)
    }

    #[inline]
    pub fn is_empty(self) -> bool {
        !(self.min_x < self.max_x && self.min_y < self.max_y)
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        is_finite_pair(self.min_x, self.min_y) && is_finite_pair(self.max_x, self.max_y)
    }

    #[inline]
    pub fn width(self) -> f32 {
        if self.is_empty() {
            0.0
        } else {
            self.max_x - self.min_x
        }
    }

    #[inline]
    pub fn height(self) -> f32 {
        if self.is_empty() {
            0.0
        } else {
            self.max_y - self.min_y
        }
    }

    #[inline]
    pub fn union_point(self, p: Point) -> Self {
        if !p.is_finite() {
            return self;
        }
        Self {
            min_x: if p.x < self.min_x { p.x } else { self.min_x },
            min_y: if p.y < self.min_y { p.y } else { self.min_y },
            max_x: if p.x > self.max_x { p.x } else { self.max_x },
            max_y: if p.y > self.max_y { p.y } else { self.max_y },
        }
    }

    #[inline]
    pub fn union(self, other: Self) -> Self {
        if other.is_empty() {
            return self;
        }
        if self.is_empty() {
            return other;
        }
        Self {
            min_x: if other.min_x < self.min_x { other.min_x } else { self.min_x },
            min_y: if other.min_y < self.min_y { other.min_y } else { self.min_y },
            max_x: if other.max_x > self.max_x { other.max_x } else { self.max_x },
            max_y: if other.max_y > self.max_y { other.max_y } else { self.max_y },
        }
    }

    #[inline]
    pub fn intersect(self, other: Self) -> Self {
        let r = Self {
            min_x: if other.min_x > self.min_x { other.min_x } else { self.min_x },
            min_y: if other.min_y > self.min_y { other.min_y } else { self.min_y },
            max_x: if other.max_x < self.max_x { other.max_x } else { self.max_x },
            max_y: if other.max_y < self.max_y { other.max_y } else { self.max_y },
        };
        if r.is_empty() { Self::EMPTY } else { r }
    }

    #[inline]
    pub fn intersects(self, other: Self) -> bool {
        !self.intersect(other).is_empty()
    }

    #[inline]
    pub fn inflate(self, dx: f32, dy: f32) -> Self {
        if self.is_empty() {
            return Self::EMPTY;
        }
        Self::new(
            self.min_x - dx,
            self.min_y - dy,
            self.max_x + dx,
            self.max_y + dy,
        )
    }

    #[inline]
    pub fn contains(self, p: Point) -> bool {
        p.x >= self.min_x && p.x < self.max_x && p.y >= self.min_y && p.y < self.max_y
    }

    #[inline]
    pub fn to_irect_out(self) -> IRect {
        if self.is_empty() || !self.is_finite() {
            return IRect::EMPTY;
        }
        const LIM: f32 = 1_073_741_824.0;
        let min_x = floor_f32(clamp_finite_f32(self.min_x, -LIM, LIM)) as i32;
        let min_y = floor_f32(clamp_finite_f32(self.min_y, -LIM, LIM)) as i32;
        let max_x = ceil_f32(clamp_finite_f32(self.max_x, -LIM, LIM)) as i32;
        let max_y = ceil_f32(clamp_finite_f32(self.max_y, -LIM, LIM)) as i32;
        IRect::new(min_x, min_y, max_x, max_y)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct IRect {
    pub min_x: i32,
    pub min_y: i32,
    pub max_x: i32,
    pub max_y: i32,
}

impl IRect {
    pub const EMPTY: Self = Self {
        min_x: 0,
        min_y: 0,
        max_x: 0,
        max_y: 0,
    };

    #[inline]
    pub const fn new(min_x: i32, min_y: i32, max_x: i32, max_y: i32) -> Self {
        Self {
            min_x,
            min_y,
            max_x,
            max_y,
        }
    }

    #[inline]
    pub fn from_xywh(x: i32, y: i32, w: i32, h: i32) -> Self {
        if w <= 0 || h <= 0 {
            return Self::EMPTY;
        }
        Self::new(x, y, x.saturating_add(w), y.saturating_add(h))
    }

    #[inline]
    pub const fn is_empty(self) -> bool {
        self.min_x >= self.max_x || self.min_y >= self.max_y
    }

    #[inline]
    pub const fn width(self) -> i32 {
        if self.is_empty() {
            0
        } else {
            self.max_x - self.min_x
        }
    }

    #[inline]
    pub const fn height(self) -> i32 {
        if self.is_empty() {
            0
        } else {
            self.max_y - self.min_y
        }
    }

    #[inline]
    pub fn intersect(self, other: Self) -> Self {
        let r = Self {
            min_x: self.min_x.max(other.min_x),
            min_y: self.min_y.max(other.min_y),
            max_x: self.max_x.min(other.max_x),
            max_y: self.max_y.min(other.max_y),
        };
        if r.is_empty() { Self::EMPTY } else { r }
    }

    #[inline]
    pub fn union(self, other: Self) -> Self {
        if other.is_empty() {
            return self;
        }
        if self.is_empty() {
            return other;
        }
        Self {
            min_x: self.min_x.min(other.min_x),
            min_y: self.min_y.min(other.min_y),
            max_x: self.max_x.max(other.max_x),
            max_y: self.max_y.max(other.max_y),
        }
    }

    #[inline]
    pub const fn contains(self, x: i32, y: i32) -> bool {
        x >= self.min_x && x < self.max_x && y >= self.min_y && y < self.max_y
    }
}

// 2x3 affine matrix [sx, ky, kx, sy, tx, ty]:
// x' = sx * x + kx * y + tx
// y' = ky * x + sy * y + ty
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Affine2D(pub [f32; 6]);

impl Default for Affine2D {
    #[inline]
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Affine2D {
    pub const IDENTITY: Self = Self([1.0, 0.0, 0.0, 1.0, 0.0, 0.0]);

    #[inline]
    pub const fn from_coeffs(sx: f32, ky: f32, kx: f32, sy: f32, tx: f32, ty: f32) -> Self {
        Self([sx, ky, kx, sy, tx, ty])
    }

    #[inline]
    pub const fn translate(tx: f32, ty: f32) -> Self {
        Self([1.0, 0.0, 0.0, 1.0, tx, ty])
    }

    #[inline]
    pub const fn scale(sx: f32, sy: f32) -> Self {
        Self([sx, 0.0, 0.0, sy, 0.0, 0.0])
    }

    #[inline]
    pub const fn scale_uniform(s: f32) -> Self {
        Self::scale(s, s)
    }

    #[inline]
    pub fn rotate(radians: f32) -> Self {
        let (s, c) = sincos_f32(radians);
        Self([c, s, -s, c, 0.0, 0.0])
    }

    #[inline]
    pub fn skew(ax: f32, ay: f32) -> Self {
        let (sx, cx) = sincos_f32(ax);
        let (sy, cy) = sincos_f32(ay);
        let tx = if cx == 0.0 { f32::NAN } else { sx / cx };
        let ty = if cy == 0.0 { f32::NAN } else { sy / cy };
        Self([1.0, ty, tx, 1.0, 0.0, 0.0])
    }

    #[inline]
    pub fn is_finite(self) -> bool {
        self.0.iter().all(|v| v.is_finite())
    }

    #[inline]
    pub fn is_identity(self) -> bool {
        self.0 == Self::IDENTITY.0
    }

    #[inline]
    pub fn is_translation_only(self) -> bool {
        self.0[0] == 1.0 && self.0[1] == 0.0 && self.0[2] == 0.0 && self.0[3] == 1.0
    }

    #[inline]
    pub fn is_axis_aligned_rect(self) -> bool {
        (self.0[1] == 0.0 && self.0[2] == 0.0) || (self.0[0] == 0.0 && self.0[3] == 0.0)
    }

    #[inline]
    pub fn determinant(self) -> f64 {
        (self.0[0] as f64) * (self.0[3] as f64) - (self.0[1] as f64) * (self.0[2] as f64)
    }

    pub fn invert(self) -> Option<Self> {
        if !self.is_finite() {
            return None;
        }
        let det = self.determinant();
        if !det.is_finite() || abs_f64(det) <= 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        let a = self.0[0] as f64;
        let b = self.0[1] as f64;
        let c = self.0[2] as f64;
        let d = self.0[3] as f64;
        let tx = self.0[4] as f64;
        let ty = self.0[5] as f64;

        let ia = d * inv_det;
        let ib = -b * inv_det;
        let ic = -c * inv_det;
        let id = a * inv_det;
        let itx = -(ia * tx + ic * ty);
        let ity = -(ib * tx + id * ty);

        let inv = Self([ia as f32, ib as f32, ic as f32, id as f32, itx as f32, ity as f32]);
        if inv.is_finite() { Some(inv) } else { None }
    }

    // Applies `other` first, then `self` (self * other).
    #[inline]
    pub fn pre_concat(self, other: Self) -> Self {
        self * other
    }

    // Applies `self` first, then `other` (other * self).
    #[inline]
    pub fn post_concat(self, other: Self) -> Self {
        other * self
    }

    #[inline]
    pub fn transform_point(self, p: Point) -> Point {
        let [sx, ky, kx, sy, tx, ty] = self.0;
        Point::new(sx * p.x + kx * p.y + tx, ky * p.x + sy * p.y + ty)
    }

    #[inline]
    pub fn transform_vec(self, v: Vec2) -> Vec2 {
        let [sx, ky, kx, sy, _, _] = self.0;
        Vec2::new(sx * v.x + kx * v.y, ky * v.x + sy * v.y)
    }

    pub fn transform_rect(self, r: Rect) -> Rect {
        if r.is_empty() {
            return Rect::EMPTY;
        }
        let p0 = self.transform_point(Point::new(r.min_x, r.min_y));
        let p1 = self.transform_point(Point::new(r.max_x, r.min_y));
        let p2 = self.transform_point(Point::new(r.min_x, r.max_y));
        let p3 = self.transform_point(Point::new(r.max_x, r.max_y));
        Rect::EMPTY
            .union_point(p0)
            .union_point(p1)
            .union_point(p2)
            .union_point(p3)
    }

    // Exact largest singular value of the 2x2 linear part.
    pub fn max_scale_factor(self) -> f32 {
        let a = self.0[0] as f64;
        let b = self.0[1] as f64;
        let c = self.0[2] as f64;
        let d = self.0[3] as f64;
        let s1 = a * a + b * b;
        let s2 = c * c + d * d;
        let dot = a * c + b * d;
        let diff = s1 - s2;
        let disc = sqrt_f64(diff * diff + 4.0 * dot * dot);
        sqrt_f64(0.5 * (s1 + s2 + disc)) as f32
    }
}

impl Mul for Affine2D {
    type Output = Affine2D;

    #[inline]
    fn mul(self, rhs: Affine2D) -> Self::Output {
        let [a0, a1, a2, a3, a4, a5] = self.0;
        let [b0, b1, b2, b3, b4, b5] = rhs.0;
        Affine2D([
            a0 * b0 + a2 * b1,
            a1 * b0 + a3 * b1,
            a0 * b2 + a2 * b3,
            a1 * b2 + a3 * b3,
            a0 * b4 + a2 * b5 + a4,
            a1 * b4 + a3 * b5 + a5,
        ])
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::float::abs_f32;

    fn approx_eq(a: f32, b: f32, eps: f32) -> bool {
        abs_f32(a - b) <= eps
    }

    fn approx_affine(a: Affine2D, b: Affine2D, eps: f32) -> bool {
        a.0.iter()
            .zip(b.0.iter())
            .all(|(&x, &y)| approx_eq(x, y, eps))
    }

    #[test]
    fn affine_inverse_and_associativity() {
        let t = Affine2D::translate(120.5, -45.25);
        let r = Affine2D::rotate(0.73);
        let s = Affine2D::scale(2.5, -0.4);
        let k = Affine2D::skew(0.2, -0.15);

        let a = t * r * s * k;
        let b = t.pre_concat(r).pre_concat(s).pre_concat(k);
        assert!(approx_affine(a, b, 1e-6));

        let inv = a.invert().expect("invertible");
        assert!(approx_affine(a * inv, Affine2D::IDENTITY, 1e-5));
        assert!(approx_affine(inv * a, Affine2D::IDENTITY, 1e-5));

        let p = Point::new(13.0, -29.0);
        let q = inv.transform_point(a.transform_point(p));
        assert!(approx_eq(q.x, p.x, 1e-4) && approx_eq(q.y, p.y, 1e-4));

        // post_concat applies self first, then other
        let composed = s.post_concat(t);
        let p_seq = t.transform_point(s.transform_point(p));
        let p_comp = composed.transform_point(p);
        assert!(approx_eq(p_seq.x, p_comp.x, 1e-5) && approx_eq(p_seq.y, p_comp.y, 1e-5));
    }

    #[test]
    fn affine_singular_and_nan_rejection() {
        assert!(Affine2D::scale(0.0, 1.0).invert().is_none());
        assert!(Affine2D::from_coeffs(1.0, 2.0, 2.0, 4.0, 0.0, 0.0).invert().is_none());
        assert!(Affine2D::translate(f32::NAN, 0.0).invert().is_none());
        assert!(Affine2D::translate(f32::INFINITY, 0.0).invert().is_none());
    }

    #[test]
    fn affine_scale_factor_and_predicates() {
        assert!(Affine2D::IDENTITY.is_identity());
        assert!(Affine2D::translate(5.0, -3.0).is_translation_only());
        assert!(Affine2D::scale(2.0, -3.0).is_axis_aligned_rect());
        assert!(Affine2D::from_coeffs(0.0, 1.0, -1.0, 0.0, 10.0, 20.0).is_axis_aligned_rect());
        assert!(!Affine2D::rotate(0.5).is_axis_aligned_rect());

        assert!(approx_eq(Affine2D::scale(3.0, 0.5).max_scale_factor(), 3.0, 1e-6));
        assert!(approx_eq(Affine2D::rotate(1.23).max_scale_factor(), 1.0, 1e-6));
        let rs = Affine2D::rotate(0.8) * Affine2D::scale(4.0, 1.5);
        assert!(approx_eq(rs.max_scale_factor(), 4.0, 1e-5));
    }

    #[test]
    fn rect_and_irect_ops() {
        let r = Rect::from_xywh(10.2, 20.7, 5.1, 3.8);
        assert_eq!(r.to_irect_out(), IRect::new(10, 20, 16, 25));
        assert!(Rect::from_xywh(0.0, 0.0, -1.0, 5.0).is_empty());
        assert!(Rect::from_xywh(f32::NAN, 0.0, 10.0, 10.0).is_empty());

        let rot90 = Affine2D::from_coeffs(0.0, 1.0, -1.0, 0.0, 0.0, 0.0);
        let tr = rot90.transform_rect(Rect::new(0.0, 0.0, 10.0, 20.0));
        assert_eq!(tr, Rect::new(-20.0, 0.0, 0.0, 10.0));

        let a = IRect::new(0, 0, 10, 10);
        let b = IRect::new(5, 5, 15, 12);
        assert_eq!(a.intersect(b), IRect::new(5, 5, 10, 10));
        assert_eq!(a.union(b), IRect::new(0, 0, 15, 12));
        assert!(a.intersect(IRect::new(20, 20, 30, 30)).is_empty());
    }
}
