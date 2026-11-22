#![forbid(unsafe_code)]

use alloc::vec::Vec;
use core::f32::consts::{FRAC_PI_2, PI, TAU};

use crate::error::{Result, ZvgError};
use crate::math::affine::{Affine2D, Point, Rect};
use crate::math::float::{
    abs_f32, abs_f64, acos_f32, ceil_f32, is_finite_pair, sincos_f32, sqrt_f64,
};

// Exact cubic Bezier kappa for 90-degree circular arcs: 4/3 * (sqrt(2) - 1).
const KAPPA: f32 = 0.552_284_8;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum FillRule {
    #[default]
    NonZero,
    EvenOdd,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PathVerb {
    MoveTo = 0,
    LineTo = 1,
    QuadTo = 2,
    CubicTo = 3,
    Close = 4,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PathEvent {
    MoveTo(Point),
    LineTo(Point),
    QuadTo(Point, Point),
    CubicTo(Point, Point, Point),
    Close,
}

#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    verbs: Vec<PathVerb>,
    points: Vec<Point>,
    control_bounds: Rect,
    has_curves: bool,
    axis_aligned_rect: Option<Rect>,
}

impl Default for Path {
    fn default() -> Self {
        Self::empty()
    }
}

impl Path {
    pub fn empty() -> Self {
        Self {
            verbs: Vec::new(),
            points: Vec::new(),
            control_bounds: Rect::EMPTY,
            has_curves: false,
            axis_aligned_rect: None,
        }
    }

    pub(crate) fn from_parts(verbs: Vec<PathVerb>, points: Vec<Point>) -> Self {
        let mut control_bounds = Rect::EMPTY;
        for &p in &points {
            control_bounds = control_bounds.union_point(p);
        }
        let has_curves = verbs
            .iter()
            .any(|v| matches!(v, PathVerb::QuadTo | PathVerb::CubicTo));
        let axis_aligned_rect = detect_axis_aligned_rect(&verbs, &points);
        Self {
            verbs,
            points,
            control_bounds,
            has_curves,
            axis_aligned_rect,
        }
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.verbs.is_empty()
    }

    #[inline]
    pub fn verbs(&self) -> &[PathVerb] {
        &self.verbs
    }

    #[inline]
    pub fn points(&self) -> &[Point] {
        &self.points
    }

    #[inline]
    pub fn has_curves(&self) -> bool {
        self.has_curves
    }

    #[inline]
    pub fn as_rect(&self) -> Option<Rect> {
        self.axis_aligned_rect
    }

    #[inline]
    pub fn control_bounds(&self) -> Rect {
        self.control_bounds
    }

    #[inline]
    pub fn iter(&self) -> PathIter<'_> {
        PathIter {
            verbs: &self.verbs,
            points: &self.points,
            v_idx: 0,
            p_idx: 0,
        }
    }

    pub fn transform(&self, m: Affine2D) -> Result<Self> {
        if !m.is_finite() {
            return Err(ZvgError::InvalidArgument("non-finite affine transform"));
        }
        if self.is_empty() || m.is_identity() {
            return Ok(self.clone());
        }
        let mut pts = Vec::with_capacity(self.points.len());
        for &p in &self.points {
            let tp = m.transform_point(p);
            if !tp.is_finite() {
                return Err(ZvgError::InvalidArgument("transformed point overflowed to non-finite"));
            }
            pts.push(tp);
        }
        Ok(Self::from_parts(self.verbs.clone(), pts))
    }

    pub fn content_hash(&self) -> u64 {
        let mut h = 0xCBF2_9CE4_8422_2325u64;
        let mix = |h: &mut u64, v: u64| {
            *h ^= v;
            *h = h.wrapping_mul(0x0000_0100_0000_01B3);
        };
        mix(&mut h, self.verbs.len() as u64);
        for &v in &self.verbs {
            mix(&mut h, v as u64);
        }
        for &p in &self.points {
            // Canonicalize -0.0 to +0.0 for stable hashing
            let xb = if p.x == 0.0 { 0u32 } else { p.x.to_bits() };
            let yb = if p.y == 0.0 { 0u32 } else { p.y.to_bits() };
            mix(&mut h, ((xb as u64) << 32) | (yb as u64));
        }
        h
    }
}

pub struct PathIter<'a> {
    verbs: &'a [PathVerb],
    points: &'a [Point],
    v_idx: usize,
    p_idx: usize,
}

impl Iterator for PathIter<'_> {
    type Item = PathEvent;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        let verb = *self.verbs.get(self.v_idx)?;
        self.v_idx += 1;
        match verb {
            PathVerb::MoveTo => {
                let p = self.points[self.p_idx];
                self.p_idx += 1;
                Some(PathEvent::MoveTo(p))
            }
            PathVerb::LineTo => {
                let p = self.points[self.p_idx];
                self.p_idx += 1;
                Some(PathEvent::LineTo(p))
            }
            PathVerb::QuadTo => {
                let c = self.points[self.p_idx];
                let p = self.points[self.p_idx + 1];
                self.p_idx += 2;
                Some(PathEvent::QuadTo(c, p))
            }
            PathVerb::CubicTo => {
                let c1 = self.points[self.p_idx];
                let c2 = self.points[self.p_idx + 1];
                let p = self.points[self.p_idx + 2];
                self.p_idx += 3;
                Some(PathEvent::CubicTo(c1, c2, p))
            }
            PathVerb::Close => Some(PathEvent::Close),
        }
    }
}

fn detect_axis_aligned_rect(verbs: &[PathVerb], points: &[Point]) -> Option<Rect> {
    let is_4_or_5 = matches!(
        verbs,
        [
            PathVerb::MoveTo,
            PathVerb::LineTo,
            PathVerb::LineTo,
            PathVerb::LineTo,
            PathVerb::Close
        ] | [
            PathVerb::MoveTo,
            PathVerb::LineTo,
            PathVerb::LineTo,
            PathVerb::LineTo,
            PathVerb::LineTo,
            PathVerb::Close
        ]
    );
    if !is_4_or_5 {
        return None;
    }
    if points.len() == 5 && points[4] != points[0] {
        return None;
    }
    let p = &points[..4];
    // Adjacent edges must alternate strictly horizontal and vertical.
    let e0_h = p[0].y == p[1].y && p[0].x != p[1].x;
    let e0_v = p[0].x == p[1].x && p[0].y != p[1].y;
    let valid = if e0_h {
        p[1].x == p[2].x && p[2].y == p[3].y && p[3].x == p[0].x && p[1].y != p[2].y
    } else if e0_v {
        p[1].y == p[2].y && p[2].x == p[3].x && p[3].y == p[0].y && p[1].x != p[2].x
    } else {
        false
    };
    if !valid {
        return None;
    }
    let mut r = Rect::EMPTY;
    for &pt in p {
        r = r.union_point(pt);
    }
    if r.is_empty() { None } else { Some(r) }
}

#[derive(Debug, Clone, Default)]
pub struct PathBuilder {
    verbs: Vec<PathVerb>,
    points: Vec<Point>,
    current: Option<Point>,
    subpath_start: Option<Point>,
    invalid: bool,
}

impl PathBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_capacity(verbs: usize, points: usize) -> Self {
        Self {
            verbs: Vec::with_capacity(verbs),
            points: Vec::with_capacity(points),
            current: None,
            subpath_start: None,
            invalid: false,
        }
    }

    pub fn clear(&mut self) {
        self.verbs.clear();
        self.points.clear();
        self.current = None;
        self.subpath_start = None;
        self.invalid = false;
    }

    #[inline]
    pub fn is_valid(&self) -> bool {
        !self.invalid
    }

    #[inline]
    pub fn current_point(&self) -> Option<Point> {
        self.current
    }

    #[inline]
    pub fn move_to(&mut self, x: f32, y: f32) -> &mut Self {
        if !is_finite_pair(x, y) {
            self.invalid = true;
            return self;
        }
        let p = Point::new(x, y);
        self.verbs.push(PathVerb::MoveTo);
        self.points.push(p);
        self.current = Some(p);
        self.subpath_start = Some(p);
        self
    }

    #[inline]
    pub fn rel_move_to(&mut self, dx: f32, dy: f32) -> &mut Self {
        let base = self.current.unwrap_or(Point::ZERO);
        self.move_to(base.x + dx, base.y + dy)
    }

    #[inline]
    pub fn line_to(&mut self, x: f32, y: f32) -> &mut Self {
        if !is_finite_pair(x, y) {
            self.invalid = true;
            return self;
        }
        if self.current.is_none() {
            return self.move_to(x, y);
        }
        let p = Point::new(x, y);
        self.verbs.push(PathVerb::LineTo);
        self.points.push(p);
        self.current = Some(p);
        self
    }

    #[inline]
    pub fn rel_line_to(&mut self, dx: f32, dy: f32) -> &mut Self {
        let base = self.current.unwrap_or(Point::ZERO);
        self.line_to(base.x + dx, base.y + dy)
    }

    #[inline]
    pub fn quad_to(&mut self, cx: f32, cy: f32, x: f32, y: f32) -> &mut Self {
        if !is_finite_pair(cx, cy) || !is_finite_pair(x, y) {
            self.invalid = true;
            return self;
        }
        if self.current.is_none() {
            self.move_to(cx, cy);
        }
        let c = Point::new(cx, cy);
        let p = Point::new(x, y);
        self.verbs.push(PathVerb::QuadTo);
        self.points.push(c);
        self.points.push(p);
        self.current = Some(p);
        self
    }

    #[inline]
    pub fn rel_quad_to(&mut self, dcx: f32, dcy: f32, dx: f32, dy: f32) -> &mut Self {
        let base = self.current.unwrap_or(Point::ZERO);
        self.quad_to(base.x + dcx, base.y + dcy, base.x + dx, base.y + dy)
    }

    #[inline]
    pub fn cubic_to(
        &mut self,
        c1x: f32,
        c1y: f32,
        c2x: f32,
        c2y: f32,
        x: f32,
        y: f32,
    ) -> &mut Self {
        if !is_finite_pair(c1x, c1y) || !is_finite_pair(c2x, c2y) || !is_finite_pair(x, y) {
            self.invalid = true;
            return self;
        }
        if self.current.is_none() {
            self.move_to(c1x, c1y);
        }
        let c1 = Point::new(c1x, c1y);
        let c2 = Point::new(c2x, c2y);
        let p = Point::new(x, y);
        self.verbs.push(PathVerb::CubicTo);
        self.points.push(c1);
        self.points.push(c2);
        self.points.push(p);
        self.current = Some(p);
        self
    }

    #[inline]
    pub fn rel_cubic_to(
        &mut self,
        dc1x: f32,
        dc1y: f32,
        dc2x: f32,
        dc2y: f32,
        dx: f32,
        dy: f32,
    ) -> &mut Self {
        let base = self.current.unwrap_or(Point::ZERO);
        self.cubic_to(
            base.x + dc1x,
            base.y + dc1y,
            base.x + dc2x,
            base.y + dc2y,
            base.x + dx,
            base.y + dy,
        )
    }

    #[inline]
    pub fn close(&mut self) -> &mut Self {
        if let Some(start) = self.subpath_start {
            self.verbs.push(PathVerb::Close);
            self.current = Some(start);
        }
        self
    }

    // SVG F.6.5 / F.6.6 elliptical arc parameterization converted to <= 90-deg cubic segments.
    #[allow(clippy::too_many_arguments)]
    pub fn arc_to(
        &mut self,
        rx_in: f32,
        ry_in: f32,
        x_axis_rotation_deg: f32,
        large_arc: bool,
        sweep: bool,
        x2: f32,
        y2: f32,
    ) -> &mut Self {
        if !is_finite_pair(rx_in, ry_in)
            || !x_axis_rotation_deg.is_finite()
            || !is_finite_pair(x2, y2)
        {
            self.invalid = true;
            return self;
        }
        let Some(p1) = self.current else {
            return self.move_to(x2, y2);
        };
        if p1.x == x2 && p1.y == y2 {
            return self;
        }
        let mut rx = abs_f64(rx_in as f64);
        let mut ry = abs_f64(ry_in as f64);
        if rx <= 1e-12 || ry <= 1e-12 {
            return self.line_to(x2, y2);
        }

        let phi = (x_axis_rotation_deg % 360.0) * (PI / 180.0);
        let (sin_phi_f, cos_phi_f) = sincos_f32(phi);
        let sin_phi = sin_phi_f as f64;
        let cos_phi = cos_phi_f as f64;

        let x1 = p1.x as f64;
        let y1 = p1.y as f64;
        let x2d = x2 as f64;
        let y2d = y2 as f64;

        let dx2 = 0.5 * (x1 - x2d);
        let dy2 = 0.5 * (y1 - y2d);
        let x1p = cos_phi * dx2 + sin_phi * dy2;
        let y1p = -sin_phi * dx2 + cos_phi * dy2;

        // Scale radii up if too small to span endpoints (SVG F.6.6.3)
        let x1p2 = x1p * x1p;
        let y1p2 = y1p * y1p;
        let mut rx2 = rx * rx;
        let mut ry2 = ry * ry;
        let lambda = x1p2 / rx2 + y1p2 / ry2;
        if lambda > 1.0 {
            let s = sqrt_f64(lambda);
            rx *= s;
            ry *= s;
            rx2 = rx * rx;
            ry2 = ry * ry;
        }

        let num = (rx2 * ry2 - rx2 * y1p2 - ry2 * x1p2).max(0.0);
        let den = rx2 * y1p2 + ry2 * x1p2;
        let coef_mag = if den > 0.0 { sqrt_f64(num / den) } else { 0.0 };
        let coef = if large_arc == sweep { -coef_mag } else { coef_mag };
        let cxp = coef * (rx * y1p / ry);
        let cyp = coef * (-ry * x1p / rx);

        let cx = cos_phi * cxp - sin_phi * cyp + 0.5 * (x1 + x2d);
        let cy = sin_phi * cxp + cos_phi * cyp + 0.5 * (y1 + y2d);

        let ux = (x1p - cxp) / rx;
        let uy = (y1p - cyp) / ry;
        let vx = (-x1p - cxp) / rx;
        let vy = (-y1p - cyp) / ry;

        let theta1 = vec_angle(1.0, 0.0, ux, uy);
        let mut dtheta = vec_angle(ux, uy, vx, vy);
        if !sweep && dtheta > 0.0 {
            dtheta -= TAU as f64;
        } else if sweep && dtheta < 0.0 {
            dtheta += TAU as f64;
        }

        let segs = (ceil_f32((abs_f64(dtheta) as f32) / FRAC_PI_2) as usize).max(1);
        let delta = dtheta / (segs as f64);
        let half_delta = 0.5 * delta;
        let (sh, ch) = sincos_f32((0.5 * half_delta) as f32);
        let alpha = (4.0 / 3.0) * ((sh as f64) / (ch as f64));

        for i in 0..segs {
            let t0 = theta1 + (i as f64) * delta;
            let t1 = t0 + delta;
            let (s0, c0) = sincos_f32(t0 as f32);
            let (s1, c1) = sincos_f32(t1 as f32);
            let (s0, c0) = (s0 as f64, c0 as f64);
            let (s1, c1) = (s1 as f64, c1 as f64);

            let map = |px: f64, py: f64| -> (f32, f32) {
                let ex = rx * px;
                let ey = ry * py;
                (
                    (cos_phi * ex - sin_phi * ey + cx) as f32,
                    (sin_phi * ex + cos_phi * ey + cy) as f32,
                )
            };

            let (c1x, c1y) = map(c0 - alpha * s0, s0 + alpha * c0);
            let (c2x, c2y) = map(c1 + alpha * s1, s1 - alpha * c1);
            let (ex, ey) = if i + 1 == segs {
                (x2, y2)
            } else {
                map(c1, s1)
            };
            self.cubic_to(c1x, c1y, c2x, c2y, ex, ey);
        }
        self
    }

    // Center-parameterized circular arc matching Cairo arc() / arc_negative().
    pub fn arc_center(
        &mut self,
        cx: f32,
        cy: f32,
        radius: f32,
        start_angle: f32,
        end_angle: f32,
        ccw: bool,
    ) -> &mut Self {
        if !is_finite_pair(cx, cy)
            || !radius.is_finite()
            || !is_finite_pair(start_angle, end_angle)
            || radius < 0.0
        {
            self.invalid = true;
            return self;
        }
        let (ss, cs) = sincos_f32(start_angle);
        let sx = cx + radius * cs;
        let sy = cy + radius * ss;
        if self.current.is_some() {
            self.line_to(sx, sy);
        } else {
            self.move_to(sx, sy);
        }
        if radius == 0.0 {
            return self;
        }

        let mut sweep = end_angle - start_angle;
        if !ccw {
            while sweep < 0.0 {
                sweep += TAU;
            }
            if sweep > TAU {
                sweep %= TAU;
                if sweep == 0.0 {
                    sweep = TAU;
                }
            }
        } else {
            while sweep > 0.0 {
                sweep -= TAU;
            }
            if sweep < -TAU {
                sweep %= TAU;
                if sweep == 0.0 {
                    sweep = -TAU;
                }
            }
        }
        if abs_f32(sweep) < 1e-7 {
            return self;
        }

        let segs = (ceil_f32(abs_f32(sweep) / FRAC_PI_2) as usize).max(1);
        let delta = sweep / (segs as f32);
        let (sh, ch) = sincos_f32(0.25 * delta);
        let alpha = (4.0 / 3.0) * (sh / ch);

        for i in 0..segs {
            let t0 = start_angle + (i as f32) * delta;
            let t1 = t0 + delta;
            let (s0, c0) = sincos_f32(t0);
            let (s1, c1) = sincos_f32(t1);
            let c1x = cx + radius * (c0 - alpha * s0);
            let c1y = cy + radius * (s0 + alpha * c0);
            let c2x = cx + radius * (c1 + alpha * s1);
            let c2y = cy + radius * (s1 - alpha * c1);
            let ex = cx + radius * c1;
            let ey = cy + radius * s1;
            self.cubic_to(c1x, c1y, c2x, c2y, ex, ey);
        }
        self
    }

    pub fn push_rect(&mut self, rect: Rect) -> &mut Self {
        if !rect.is_finite() {
            self.invalid = true;
            return self;
        }
        if rect.is_empty() {
            return self;
        }
        self.move_to(rect.min_x, rect.min_y)
            .line_to(rect.max_x, rect.min_y)
            .line_to(rect.max_x, rect.max_y)
            .line_to(rect.min_x, rect.max_y)
            .close()
    }

    pub fn push_rounded_rect(&mut self, rect: Rect, rx: f32, ry: f32) -> &mut Self {
        if !rect.is_finite() || !is_finite_pair(rx, ry) {
            self.invalid = true;
            return self;
        }
        if rect.is_empty() {
            return self;
        }
        let rx = rx.clamp(0.0, 0.5 * rect.width());
        let ry = ry.clamp(0.0, 0.5 * rect.height());
        if rx == 0.0 || ry == 0.0 {
            return self.push_rect(rect);
        }
        let kx = rx * KAPPA;
        let ky = ry * KAPPA;
        let (x0, y0, x1, y1) = (rect.min_x, rect.min_y, rect.max_x, rect.max_y);

        self.move_to(x0 + rx, y0)
            .line_to(x1 - rx, y0)
            .cubic_to(x1 - rx + kx, y0, x1, y0 + ry - ky, x1, y0 + ry)
            .line_to(x1, y1 - ry)
            .cubic_to(x1, y1 - ry + ky, x1 - rx + kx, y1, x1 - rx, y1)
            .line_to(x0 + rx, y1)
            .cubic_to(x0 + rx - kx, y1, x0, y1 - ry + ky, x0, y1 - ry)
            .line_to(x0, y0 + ry)
            .cubic_to(x0, y0 + ry - ky, x0 + rx - kx, y0, x0 + rx, y0)
            .close()
    }

    pub fn push_ellipse(&mut self, cx: f32, cy: f32, rx: f32, ry: f32) -> &mut Self {
        if !is_finite_pair(cx, cy) || !is_finite_pair(rx, ry) {
            self.invalid = true;
            return self;
        }
        if rx <= 0.0 || ry <= 0.0 {
            return self;
        }
        let kx = rx * KAPPA;
        let ky = ry * KAPPA;
        self.move_to(cx + rx, cy)
            .cubic_to(cx + rx, cy + ky, cx + kx, cy + ry, cx, cy + ry)
            .cubic_to(cx - kx, cy + ry, cx - rx, cy + ky, cx - rx, cy)
            .cubic_to(cx - rx, cy - ky, cx - kx, cy - ry, cx, cy - ry)
            .cubic_to(cx + kx, cy - ry, cx + rx, cy - ky, cx + rx, cy)
            .close()
    }

    pub fn push_circle(&mut self, cx: f32, cy: f32, r: f32) -> &mut Self {
        self.push_ellipse(cx, cy, r, r)
    }

    pub fn push_polygon(&mut self, pts: &[Point], closed: bool) -> &mut Self {
        let Some((&first, rest)) = pts.split_first() else {
            return self;
        };
        self.move_to(first.x, first.y);
        for &p in rest {
            self.line_to(p.x, p.y);
        }
        if closed {
            self.close();
        }
        self
    }

    pub fn try_finish(&self) -> Result<Path> {
        if self.invalid {
            return Err(ZvgError::InvalidArgument("path contains non-finite coordinates"));
        }
        Ok(Path::from_parts(self.verbs.clone(), self.points.clone()))
    }

    pub fn finish(self) -> Path {
        if self.invalid {
            return Path::empty();
        }
        Path::from_parts(self.verbs, self.points)
    }
}

fn vec_angle(ux: f64, uy: f64, vx: f64, vy: f64) -> f64 {
    let dot = ux * vx + uy * vy;
    let len = sqrt_f64((ux * ux + uy * uy) * (vx * vx + vy * vy));
    if len == 0.0 {
        return 0.0;
    }
    let cos_a = (dot / len).clamp(-1.0, 1.0);
    let ang = acos_f32(cos_a as f32) as f64;
    if ux * vy - uy * vx < 0.0 { -ang } else { ang }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn eval_cubic(p0: Point, p1: Point, p2: Point, p3: Point, t: f32) -> Point {
        let mt = 1.0 - t;
        let a = mt * mt * mt;
        let b = 3.0 * mt * mt * t;
        let c = 3.0 * mt * t * t;
        let d = t * t * t;
        Point::new(
            a * p0.x + b * p1.x + c * p2.x + d * p3.x,
            a * p0.y + b * p1.y + c * p2.y + d * p3.y,
        )
    }

    #[test]
    fn path_builder_iteration_and_rect_detection() {
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::from_xywh(10.0, 20.0, 30.0, 40.0));
        let path = pb.try_finish().unwrap();
        assert!(!path.has_curves());
        assert_eq!(path.as_rect(), Some(Rect::new(10.0, 20.0, 40.0, 60.0)));

        let events: Vec<PathEvent> = path.iter().collect();
        assert_eq!(events.len(), 5);
        assert_eq!(events[0], PathEvent::MoveTo(Point::new(10.0, 20.0)));
        assert_eq!(events[4], PathEvent::Close);

        // Rotated rect is no longer axis-aligned
        let rot = path.transform(Affine2D::rotate(0.5)).unwrap();
        assert!(rot.as_rect().is_none());
    }

    #[test]
    fn elliptical_arc_matches_parametric_ellipse() {
        // Test all 4 (large_arc, sweep) combinations on a rotated eccentric ellipse
        let rx = 80.0f32;
        let ry = 30.0f32;
        let rot_deg = 35.0f32;
        let rot_rad = rot_deg * (PI / 180.0);
        let (sr, cr) = sincos_f32(rot_rad);

        for large_arc in [false, true] {
            for sweep in [false, true] {
                let mut pb = PathBuilder::new();
                pb.move_to(10.0, 20.0)
                    .arc_to(rx, ry, rot_deg, large_arc, sweep, 90.0, 50.0);
                let path = pb.try_finish().unwrap();
                assert!(path.has_curves());

                // Verify exact endpoints
                let mut cur = Point::ZERO;
                let mut last = Point::ZERO;
                let mut cubic_count = 0usize;
                for ev in path.iter() {
                    match ev {
                        PathEvent::MoveTo(p) => {
                            cur = p;
                            last = p;
                        }
                        PathEvent::CubicTo(c1, c2, p3) => {
                            cubic_count += 1;
                            // Sample midpoint of every cubic and verify it stays finite and smooth
                            let mid = eval_cubic(cur, c1, c2, p3, 0.5);
                            assert!(mid.is_finite());
                            cur = p3;
                            last = p3;
                        }
                        _ => {}
                    }
                }
                assert!((1..=4).contains(&cubic_count));
                assert_eq!(last, Point::new(90.0, 50.0));
            }
        }

        // Exact centered rotated ellipse verification:
        // Point on ellipse at angle 0 and angle 120 deg -> cubic samples satisfy (x'/rx)^2 + (y'/ry)^2 == 1
        let cx = 100.0f32;
        let cy = 60.0f32;
        let ellipse_pt = |ang: f32| {
            let (s, c) = sincos_f32(ang);
            let ex = rx * c;
            let ey = ry * s;
            Point::new(cx + cr * ex - sr * ey, cy + sr * ex + cr * ey)
        };
        let p_start = ellipse_pt(0.2);
        let p_end = ellipse_pt(2.4);
        let mut pb = PathBuilder::new();
        pb.move_to(p_start.x, p_start.y)
            .arc_to(rx, ry, rot_deg, false, true, p_end.x, p_end.y);
        let path = pb.try_finish().unwrap();

        let mut cur = Point::ZERO;
        let mut max_err = 0.0f32;
        for ev in path.iter() {
            match ev {
                PathEvent::MoveTo(p) => cur = p,
                PathEvent::CubicTo(c1, c2, p3) => {
                    for i in 0..=16 {
                        let pt = eval_cubic(cur, c1, c2, p3, (i as f32) / 16.0);
                        let dx = pt.x - cx;
                        let dy = pt.y - cy;
                        let lx = cr * dx + sr * dy;
                        let ly = -sr * dx + cr * dy;
                        let eq = (lx / rx) * (lx / rx) + (ly / ry) * (ly / ry);
                        let err = abs_f32(eq - 1.0);
                        if err > max_err {
                            max_err = err;
                        }
                    }
                    cur = p3;
                }
                _ => {}
            }
        }
        assert!(max_err < 1e-3, "ellipse radial equation error = {max_err}");
    }

    #[test]
    fn degenerate_arcs_and_nan_rejection() {
        // Zero radius falls back to line_to
        let mut pb = PathBuilder::new();
        pb.move_to(0.0, 0.0)
            .arc_to(0.0, 50.0, 0.0, false, true, 100.0, 20.0);
        let p = pb.try_finish().unwrap();
        assert!(!p.has_curves());
        assert_eq!(p.verbs(), &[PathVerb::MoveTo, PathVerb::LineTo]);

        // NaN / Inf rejected cleanly by try_finish and sanitized by finish
        let mut bad = PathBuilder::new();
        bad.move_to(0.0, 0.0).line_to(f32::NAN, 10.0);
        assert!(bad.try_finish().is_err());
        assert!(bad.finish().is_empty());

        // Circle & rounded rect produce valid closed curved paths
        let mut shapes = PathBuilder::new();
        shapes.push_circle(50.0, 50.0, 25.0);
        shapes.push_rounded_rect(Rect::from_xywh(0.0, 0.0, 100.0, 40.0), 10.0, 10.0);
        let sp = shapes.try_finish().unwrap();
        assert!(sp.has_curves());
        assert_ne!(sp.content_hash(), 0);
    }
}
