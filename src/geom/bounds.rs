#![forbid(unsafe_code)]

use crate::math::affine::{Affine2D, Point, Rect};
use crate::math::float::{abs_f64, copysign_f64, sqrt_f64};

use super::path::{Path, PathEvent};

#[inline]
fn eval_quad_1d(p0: f64, p1: f64, p2: f64, t: f64) -> f64 {
    let mt = 1.0 - t;
    mt * mt * p0 + 2.0 * mt * t * p1 + t * t * p2
}

#[inline]
fn eval_cubic_1d(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let mt = 1.0 - t;
    let mt2 = mt * mt;
    let t2 = t * t;
    mt2 * mt * p0 + 3.0 * mt2 * t * p1 + 3.0 * mt * t2 * p2 + t2 * t * p3
}

pub fn quad_bounds_1d(p0: f32, p1: f32, p2: f32) -> (f32, f32) {
    let (a, b, c) = (p0 as f64, p1 as f64, p2 as f64);
    let mut min_v = a.min(c);
    let mut max_v = a.max(c);

    let denom = a - 2.0 * b + c;
    if abs_f64(denom) > 1e-12 {
        let t = (a - b) / denom;
        if t > 0.0 && t < 1.0 {
            let v = eval_quad_1d(a, b, c, t);
            if v < min_v {
                min_v = v;
            }
            if v > max_v {
                max_v = v;
            }
        }
    }
    (min_v as f32, max_v as f32)
}

// Analytical cubic extrema using Citardauq stable quadratic roots for B'(t) = 0.
pub fn cubic_bounds_1d(p0: f32, p1: f32, p2: f32, p3: f32) -> (f32, f32) {
    let (v0, v1, v2, v3) = (p0 as f64, p1 as f64, p2 as f64, p3 as f64);
    let mut min_v = v0.min(v3);
    let mut max_v = v0.max(v3);

    let a = -v0 + 3.0 * v1 - 3.0 * v2 + v3;
    let b = v0 - 2.0 * v1 + v2;
    let c = v1 - v0;

    let mut include_t = |t: f64| {
        if t > 0.0 && t < 1.0 {
            let v = eval_cubic_1d(v0, v1, v2, v3, t);
            if v < min_v {
                min_v = v;
            }
            if v > max_v {
                max_v = v;
            }
        }
    };

    let scale = abs_f64(b) + abs_f64(c) + 1.0;
    if abs_f64(a) <= 1e-12 * scale {
        // Near-quadratic degenerate cubic: 2*b*t + c = 0
        if abs_f64(b) > 1e-12 {
            include_t(-c / (2.0 * b));
        }
    } else {
        let disc = b * b - a * c;
        if disc >= 0.0 {
            let s = sqrt_f64(disc);
            let q = -(b + copysign_f64(s, b));
            if abs_f64(q) > 1e-15 {
                include_t(q / a);
                include_t(c / q);
            } else {
                include_t(-b / a);
            }
        }
    }

    (min_v as f32, max_v as f32)
}

#[inline]
pub fn quad_bounds(p0: Point, p1: Point, p2: Point) -> Rect {
    let (min_x, max_x) = quad_bounds_1d(p0.x, p1.x, p2.x);
    let (min_y, max_y) = quad_bounds_1d(p0.y, p1.y, p2.y);
    Rect::new(min_x, min_y, max_x, max_y)
}

#[inline]
pub fn cubic_bounds(p0: Point, p1: Point, p2: Point, p3: Point) -> Rect {
    let (min_x, max_x) = cubic_bounds_1d(p0.x, p1.x, p2.x, p3.x);
    let (min_y, max_y) = cubic_bounds_1d(p0.y, p1.y, p2.y, p3.y);
    Rect::new(min_x, min_y, max_x, max_y)
}

pub fn path_tight_bounds(path: &Path) -> Rect {
    if path.is_empty() {
        return Rect::EMPTY;
    }
    if !path.has_curves() {
        return path.control_bounds();
    }
    path_transformed_tight_bounds(path, Affine2D::IDENTITY)
}

pub fn path_transformed_tight_bounds(path: &Path, transform: Affine2D) -> Rect {
    if path.is_empty() || !transform.is_finite() {
        return Rect::EMPTY;
    }
    let is_id = transform.is_identity();
    let xform = |p: Point| if is_id { p } else { transform.transform_point(p) };

    let mut bounds = Rect::EMPTY;
    let mut cur = Point::ZERO;
    let mut start = Point::ZERO;

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                let tp = xform(p);
                cur = tp;
                start = tp;
                bounds = bounds.union_point(tp);
            }
            PathEvent::LineTo(p) => {
                let tp = xform(p);
                bounds = bounds.union_point(tp);
                cur = tp;
            }
            PathEvent::QuadTo(c, p) => {
                let tc = xform(c);
                let tp = xform(p);
                let qb = quad_bounds(cur, tc, tp);
                bounds = bounds.union_point(Point::new(qb.min_x, qb.min_y));
                bounds = bounds.union_point(Point::new(qb.max_x, qb.max_y));
                cur = tp;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                let tc1 = xform(c1);
                let tc2 = xform(c2);
                let tp = xform(p);
                let cb = cubic_bounds(cur, tc1, tc2, tp);
                bounds = bounds.union_point(Point::new(cb.min_x, cb.min_y));
                bounds = bounds.union_point(Point::new(cb.max_x, cb.max_y));
                cur = tp;
            }
            PathEvent::Close => {
                cur = start;
            }
        }
    }
    bounds
}

// Two-stage O(1) control-hull + exact analytical viewport culling.
pub fn path_visible_in_viewport(path: &Path, transform: Affine2D, viewport: Rect) -> bool {
    if path.is_empty() || viewport.is_empty() || !transform.is_finite() {
        return false;
    }
    let hull = transform.transform_rect(path.control_bounds());
    if hull.is_empty() || !hull.is_finite() || !hull.intersects(viewport) {
        return false;
    }
    if !path.has_curves() {
        return true;
    }
    let tight = path_transformed_tight_bounds(path, transform);
    !tight.is_empty() && tight.is_finite() && tight.intersects(viewport)
}

impl Path {
    #[inline]
    pub fn tight_bounds(&self) -> Rect {
        path_tight_bounds(self)
    }

    #[inline]
    pub fn transformed_bounds(&self, transform: Affine2D) -> Rect {
        path_transformed_tight_bounds(self, transform)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::PathBuilder;
    use crate::math::float::abs_f32;

    fn lcg(state: &mut u64) -> f32 {
        *state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (((*state >> 40) as f32) / 8_388_608.0 - 1.0) * 500.0
    }

    #[test]
    fn analytical_bounds_match_dense_sampling_500_curves() {
        let mut rng = 0x9E37_79B9_7F4A_7C15u64;
        const SAMPLES: usize = 2000;

        // 500 random quadratics
        for _ in 0..500 {
            let p0 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let p1 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let p2 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let b = quad_bounds(p0, p1, p2);

            let mut smin_x = f32::INFINITY;
            let mut smax_x = f32::NEG_INFINITY;
            let mut smin_y = f32::INFINITY;
            let mut smax_y = f32::NEG_INFINITY;
            for i in 0..=SAMPLES {
                let t = (i as f64) / (SAMPLES as f64);
                let x = eval_quad_1d(p0.x as f64, p1.x as f64, p2.x as f64, t) as f32;
                let y = eval_quad_1d(p0.y as f64, p1.y as f64, p2.y as f64, t) as f32;
                smin_x = smin_x.min(x);
                smax_x = smax_x.max(x);
                smin_y = smin_y.min(y);
                smax_y = smax_y.max(y);
            }
            assert!(b.min_x <= smin_x + 1e-4 && smin_x - b.min_x < 1e-2);
            assert!(b.max_x >= smax_x - 1e-4 && b.max_x - smax_x < 1e-2);
            assert!(b.min_y <= smin_y + 1e-4 && smin_y - b.min_y < 1e-2);
            assert!(b.max_y >= smax_y - 1e-4 && b.max_y - smax_y < 1e-2);
        }

        // 500 random cubics (loops, S-curves, extreme handles)
        for _ in 0..500 {
            let p0 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let p1 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let p2 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let p3 = Point::new(lcg(&mut rng), lcg(&mut rng));
            let b = cubic_bounds(p0, p1, p2, p3);

            let mut smin_x = f32::INFINITY;
            let mut smax_x = f32::NEG_INFINITY;
            let mut smin_y = f32::INFINITY;
            let mut smax_y = f32::NEG_INFINITY;
            for i in 0..=SAMPLES {
                let t = (i as f64) / (SAMPLES as f64);
                let x = eval_cubic_1d(p0.x as f64, p1.x as f64, p2.x as f64, p3.x as f64, t) as f32;
                let y = eval_cubic_1d(p0.y as f64, p1.y as f64, p2.y as f64, p3.y as f64, t) as f32;
                smin_x = smin_x.min(x);
                smax_x = smax_x.max(x);
                smin_y = smin_y.min(y);
                smax_y = smax_y.max(y);
            }
            assert!(b.min_x <= smin_x + 1e-4 && smin_x - b.min_x < 5e-2);
            assert!(b.max_x >= smax_x - 1e-4 && b.max_x - smax_x < 5e-2);
            assert!(b.min_y <= smin_y + 1e-4 && smin_y - b.min_y < 5e-2);
            assert!(b.max_y >= smax_y - 1e-4 && b.max_y - smax_y < 5e-2);
        }
    }

    #[test]
    fn near_linear_cubic_and_rotated_circle_tightness() {
        // Near-quadratic degenerate cubic (a ~= 0): Citardauq fallback must not blow up
        let (mn, mx) = cubic_bounds_1d(0.0, 50.0, 100.0, 150.00001);
        assert!(abs_f32(mn - 0.0) < 1e-4 && abs_f32(mx - 150.00001) < 1e-4);

        // Rotated circle: transformed analytical bounds stay ~100x100 (NOT bloated 141x141!)
        let mut pb = PathBuilder::new();
        pb.push_circle(0.0, 0.0, 50.0);
        let circle = pb.finish();
        let rot45 = Affine2D::rotate(core::f32::consts::FRAC_PI_4);
        let tight = circle.transformed_bounds(rot45);
        assert!(abs_f32(tight.width() - 100.0) < 0.05, "w={}", tight.width());
        assert!(abs_f32(tight.height() - 100.0) < 0.05, "h={}", tight.height());

        // Viewport culling: curve whose control points poke into viewport while curve stays outside
        let mut cpb = PathBuilder::new();
        cpb.move_to(-10.0, 0.0).cubic_to(20.0, 0.0, 20.0, 100.0, -10.0, 100.0).close();
        let bulge = cpb.finish();
        // Max x of this cubic is at t=0.5: -10*(0.25) + 20*(0.75) = 12.5 -> does NOT reach x >= 15
        let vp = Rect::new(15.0, 0.0, 100.0, 100.0);
        assert!(bulge.control_bounds().intersects(vp));
        assert!(!path_visible_in_viewport(&bulge, Affine2D::IDENTITY, vp));
    }
}
