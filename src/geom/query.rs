#![forbid(unsafe_code)]

use crate::math::affine::{Affine2D, Point, Rect};
use crate::math::float::{abs_f64, sqrt_f32, sqrt_f64};

use super::flatten::{DEFAULT_TOLERANCE, flatten_path_segments};
use super::path::{FillRule, Path, PathEvent};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClosestPointResult {
    pub point: Point,
    pub distance: f32,
    pub subpath_index: usize,
    pub segment_index: usize,
    pub t: f32,
}

// Exact half-open horizontal ray [p.x, +inf) winding number (immune to vertex double-counting).
pub fn winding_number(path: &Path, point: Point, tolerance: f32) -> i32 {
    if path.is_empty() || !point.is_finite() {
        return 0;
    }
    let b = path.control_bounds();
    if point.x < b.min_x || point.x > b.max_x || point.y < b.min_y || point.y > b.max_y {
        return 0;
    }

    let px = point.x as f64;
    let py = point.y as f64;
    let mut winding = 0i32;

    flatten_path_segments(path, Affine2D::IDENTITY, tolerance, true, &mut |a, b| {
        let (ax, ay) = (a.x as f64, a.y as f64);
        let (bx, by) = (b.x as f64, b.y as f64);
        if ay <= py {
            if by > py {
                let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
                if cross > 0.0 {
                    winding += 1;
                }
            }
        } else if by <= py {
            let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
            if cross < 0.0 {
                winding -= 1;
            }
        }
    });

    winding
}

#[inline]
pub fn point_in_fill(path: &Path, point: Point, fill_rule: FillRule, tolerance: f32) -> bool {
    let w = winding_number(path, point, tolerance);
    match fill_rule {
        FillRule::NonZero => w != 0,
        FillRule::EvenOdd => (w & 1) != 0,
    }
}

#[inline]
fn rect_dist_sq(r: Rect, p: Point) -> f64 {
    let dx = if p.x < r.min_x {
        (r.min_x - p.x) as f64
    } else if p.x > r.max_x {
        (p.x - r.max_x) as f64
    } else {
        0.0
    };
    let dy = if p.y < r.min_y {
        (r.min_y - p.y) as f64
    } else if p.y > r.max_y {
        (p.y - r.max_y) as f64
    } else {
        0.0
    };
    dx * dx + dy * dy
}

#[inline]
fn closest_on_line(a: Point, b: Point, p: Point) -> (Point, f64, f64) {
    let ax = a.x as f64;
    let ay = a.y as f64;
    let abx = b.x as f64 - ax;
    let aby = b.y as f64 - ay;
    let len2 = abx * abx + aby * aby;
    let t = if len2 <= 1e-24 {
        0.0
    } else {
        (((p.x as f64 - ax) * abx + (p.y as f64 - ay) * aby) / len2).clamp(0.0, 1.0)
    };
    let qx = ax + abx * t;
    let qy = ay + aby * t;
    let dx = qx - p.x as f64;
    let dy = qy - p.y as f64;
    (Point::new(qx as f32, qy as f32), dx * dx + dy * dy, t)
}

fn closest_on_quad(p0: Point, p1: Point, p2: Point, p: Point) -> (Point, f64, f64) {
    let (x0, y0) = (p0.x as f64, p0.y as f64);
    let (x1, y1) = (p1.x as f64, p1.y as f64);
    let (x2, y2) = (p2.x as f64, p2.y as f64);
    let (px, py) = (p.x as f64, p.y as f64);

    let eval = |t: f64| {
        let mt = 1.0 - t;
        let x = mt * mt * x0 + 2.0 * mt * t * x1 + t * t * x2;
        let y = mt * mt * y0 + 2.0 * mt * t * y1 + t * t * y2;
        let d1x = 2.0 * (mt * (x1 - x0) + t * (x2 - x1));
        let d1y = 2.0 * (mt * (y1 - y0) + t * (y2 - y1));
        let d2x = 2.0 * (x0 - 2.0 * x1 + x2);
        let d2y = 2.0 * (y0 - 2.0 * y1 + y2);
        (x, y, d1x, d1y, d2x, d2y)
    };

    const SAMPLES: usize = 12;
    let mut best_t = 0.0f64;
    let mut best_d2 = f64::INFINITY;
    let mut best_xy = (x0, y0);

    for i in 0..=SAMPLES {
        let t = (i as f64) / (SAMPLES as f64);
        let (x, y, _, _, _, _) = eval(t);
        let d2 = (x - px) * (x - px) + (y - py) * (y - py);
        if d2 < best_d2 {
            best_d2 = d2;
            best_t = t;
            best_xy = (x, y);
        }
    }

    // Damped Newton-Raphson refinement on (B(t) - P) . B'(t) = 0
    let mut t = best_t;
    for _ in 0..8 {
        let (x, y, d1x, d1y, d2x, d2y) = eval(t);
        let rx = x - px;
        let ry = y - py;
        let f = rx * d1x + ry * d1y;
        let fp = d1x * d1x + d1y * d1y + rx * d2x + ry * d2y;
        if abs_f64(fp) <= 1e-18 {
            break;
        }
        let next_t = (t - f / fp).clamp(0.0, 1.0);
        let (nx, ny, _, _, _, _) = eval(next_t);
        let nd2 = (nx - px) * (nx - px) + (ny - py) * (ny - py);
        if nd2 < best_d2 {
            best_d2 = nd2;
            best_t = next_t;
            best_xy = (nx, ny);
        }
        if abs_f64(next_t - t) < 1e-9 {
            break;
        }
        t = next_t;
    }

    (Point::new(best_xy.0 as f32, best_xy.1 as f32), best_d2, best_t)
}

fn closest_on_cubic(p0: Point, p1: Point, p2: Point, p3: Point, p: Point) -> (Point, f64, f64) {
    let (x0, y0) = (p0.x as f64, p0.y as f64);
    let (x1, y1) = (p1.x as f64, p1.y as f64);
    let (x2, y2) = (p2.x as f64, p2.y as f64);
    let (x3, y3) = (p3.x as f64, p3.y as f64);
    let (px, py) = (p.x as f64, p.y as f64);

    let eval = |t: f64| {
        let mt = 1.0 - t;
        let mt2 = mt * mt;
        let t2 = t * t;
        let x = mt2 * mt * x0 + 3.0 * mt2 * t * x1 + 3.0 * mt * t2 * x2 + t2 * t * x3;
        let y = mt2 * mt * y0 + 3.0 * mt2 * t * y1 + 3.0 * mt * t2 * y2 + t2 * t * y3;
        let d1x = 3.0 * (mt2 * (x1 - x0) + 2.0 * mt * t * (x2 - x1) + t2 * (x3 - x2));
        let d1y = 3.0 * (mt2 * (y1 - y0) + 2.0 * mt * t * (y2 - y1) + t2 * (y3 - y2));
        let d2x = 6.0 * (mt * (x2 - 2.0 * x1 + x0) + t * (x3 - 2.0 * x2 + x1));
        let d2y = 6.0 * (mt * (y2 - 2.0 * y1 + y0) + t * (y3 - 2.0 * y2 + y1));
        (x, y, d1x, d1y, d2x, d2y)
    };

    const SAMPLES: usize = 20;
    let mut best_t = 0.0f64;
    let mut best_d2 = f64::INFINITY;
    let mut best_xy = (x0, y0);

    // Track multiple local minima candidates (cubics can have up to 3 distance minima)
    let mut sample_d2 = [0.0f64; SAMPLES + 1];
    for (i, slot) in sample_d2.iter_mut().enumerate() {
        let t = (i as f64) / (SAMPLES as f64);
        let (x, y, _, _, _, _) = eval(t);
        let d2 = (x - px) * (x - px) + (y - py) * (y - py);
        *slot = d2;
        if d2 < best_d2 {
            best_d2 = d2;
            best_t = t;
            best_xy = (x, y);
        }
    }

    for i in 0..=SAMPLES {
        let is_local_min = (i == 0 || sample_d2[i] <= sample_d2[i - 1])
            && (i == SAMPLES || sample_d2[i] <= sample_d2[i + 1]);
        if !is_local_min {
            continue;
        }
        let mut t = (i as f64) / (SAMPLES as f64);
        for _ in 0..8 {
            let (x, y, d1x, d1y, d2x, d2y) = eval(t);
            let rx = x - px;
            let ry = y - py;
            let f = rx * d1x + ry * d1y;
            let fp = d1x * d1x + d1y * d1y + rx * d2x + ry * d2y;
            if abs_f64(fp) <= 1e-18 {
                break;
            }
            let next_t = (t - f / fp).clamp(0.0, 1.0);
            let (nx, ny, _, _, _, _) = eval(next_t);
            let nd2 = (nx - px) * (nx - px) + (ny - py) * (ny - py);
            if nd2 < best_d2 {
                best_d2 = nd2;
                best_t = next_t;
                best_xy = (nx, ny);
            }
            if abs_f64(next_t - t) < 1e-9 {
                break;
            }
            t = next_t;
        }
    }

    (Point::new(best_xy.0 as f32, best_xy.1 as f32), best_d2, best_t)
}

pub fn closest_point_on_path(path: &Path, point: Point) -> Option<ClosestPointResult> {
    if path.is_empty() || !point.is_finite() {
        return None;
    }

    let mut best_d2 = f64::INFINITY;
    let mut best: Option<ClosestPointResult> = None;

    let mut cur = Point::ZERO;
    let mut sub_start = Point::ZERO;
    let mut in_subpath = false;
    let mut subpath_index = 0usize;
    let mut segment_index = 0usize;

    #[inline]
    fn consider(
        best_d2: &mut f64,
        best: &mut Option<ClosestPointResult>,
        pt: Point,
        d2: f64,
        t: f64,
        sp: usize,
        seg: usize,
    ) {
        if d2 < *best_d2 {
            *best_d2 = d2;
            *best = Some(ClosestPointResult {
                point: pt,
                distance: sqrt_f64(d2) as f32,
                subpath_index: sp,
                segment_index: seg,
                t: t as f32,
            });
        }
    }

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if in_subpath {
                    subpath_index += 1;
                }
                cur = p;
                sub_start = p;
                in_subpath = true;
                segment_index = 0;
                // Handle single-point subpath cleanly
                let d2 = p.distance_sq(point) as f64;
                consider(&mut best_d2, &mut best, p, d2, 0.0, subpath_index, 0);
            }
            PathEvent::LineTo(p) => {
                if in_subpath {
                    let (q, d2, t) = closest_on_line(cur, p, point);
                    consider(
                        &mut best_d2,
                        &mut best,
                        q,
                        d2,
                        t,
                        subpath_index,
                        segment_index,
                    );
                    segment_index += 1;
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::QuadTo(c, p) => {
                if in_subpath {
                    let hull = Rect::EMPTY
                        .union_point(cur)
                        .union_point(c)
                        .union_point(p);
                    if rect_dist_sq(hull, point) < best_d2 {
                        let (q, d2, t) = closest_on_quad(cur, c, p, point);
                        consider(
                            &mut best_d2,
                            &mut best,
                            q,
                            d2,
                            t,
                            subpath_index,
                            segment_index,
                        );
                    }
                    segment_index += 1;
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                if in_subpath {
                    let hull = Rect::EMPTY
                        .union_point(cur)
                        .union_point(c1)
                        .union_point(c2)
                        .union_point(p);
                    if rect_dist_sq(hull, point) < best_d2 {
                        let (q, d2, t) = closest_on_cubic(cur, c1, c2, p, point);
                        consider(
                            &mut best_d2,
                            &mut best,
                            q,
                            d2,
                            t,
                            subpath_index,
                            segment_index,
                        );
                    }
                    segment_index += 1;
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::Close => {
                if in_subpath {
                    if cur != sub_start {
                        let (q, d2, t) = closest_on_line(cur, sub_start, point);
                        consider(
                            &mut best_d2,
                            &mut best,
                            q,
                            d2,
                            t,
                            subpath_index,
                            segment_index,
                        );
                        segment_index += 1;
                    }
                    cur = sub_start;
                }
            }
        }
    }

    best.map(|mut r| {
        r.distance = sqrt_f32(r.point.distance_sq(point));
        r
    })
}

pub fn signed_distance_to_path(
    path: &Path,
    point: Point,
    fill_rule: FillRule,
    tolerance: f32,
) -> Option<f32> {
    let cp = closest_point_on_path(path, point)?;
    if point_in_fill(path, point, fill_rule, tolerance) {
        Some(-cp.distance)
    } else {
        Some(cp.distance)
    }
}

impl Path {
    #[inline]
    pub fn contains_point(&self, point: Point, fill_rule: FillRule) -> bool {
        point_in_fill(self, point, fill_rule, DEFAULT_TOLERANCE)
    }

    #[inline]
    pub fn closest_point(&self, point: Point) -> Option<ClosestPointResult> {
        closest_point_on_path(self, point)
    }

    #[inline]
    pub fn signed_distance(&self, point: Point, fill_rule: FillRule) -> Option<f32> {
        signed_distance_to_path(self, point, fill_rule, DEFAULT_TOLERANCE)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::PathBuilder;
    use crate::math::float::{abs_f32, sincos_f32};
    use core::f32::consts::TAU;

    #[test]
    fn self_intersecting_star_nonzero_vs_evenodd() {
        // 5-point pentagram star centered at (50, 50), radius 40
        let mut pb = PathBuilder::new();
        for i in 0..5 {
            let ang = ((i * 2) as f32) * (TAU / 5.0) - core::f32::consts::FRAC_PI_2;
            let (s, c) = sincos_f32(ang);
            let x = 50.0 + 40.0 * c;
            let y = 50.0 + 40.0 * s;
            if i == 0 {
                pb.move_to(x, y);
            } else {
                pb.line_to(x, y);
            }
        }
        pb.close();
        let star = pb.finish();

        // Center has winding +/-2: inside for NonZero, outside for EvenOdd!
        let center = Point::new(50.0, 50.0);
        assert!(star.contains_point(center, FillRule::NonZero));
        assert!(!star.contains_point(center, FillRule::EvenOdd));

        // Tip interior has winding +/-1: inside for both
        let tip = Point::new(50.0, 20.0);
        assert!(star.contains_point(tip, FillRule::NonZero));
        assert!(star.contains_point(tip, FillRule::EvenOdd));

        // Outside point
        let out = Point::new(5.0, 5.0);
        assert!(!star.contains_point(out, FillRule::NonZero));
        assert!(!star.contains_point(out, FillRule::EvenOdd));
    }

    #[test]
    fn vertex_aligned_ray_edge_cases() {
        // Concave diamond / hourglass with vertices explicitly at y = 50
        let mut pb = PathBuilder::new();
        pb.move_to(50.0, 10.0)
            .line_to(90.0, 50.0)
            .line_to(50.0, 90.0)
            .line_to(10.0, 50.0)
            .close();
        let diamond = pb.finish();

        // Ray y = 50 hits BOTH left vertex (10, 50) and right vertex (90, 50)
        assert!(!diamond.contains_point(Point::new(0.0, 50.0), FillRule::NonZero));
        assert!(diamond.contains_point(Point::new(50.0, 50.0), FillRule::NonZero));
        assert!(!diamond.contains_point(Point::new(95.0, 50.0), FillRule::NonZero));

        // Peak vertex y = 10 and bottom vertex y = 90
        assert!(!diamond.contains_point(Point::new(0.0, 10.0), FillRule::NonZero));
        assert!(!diamond.contains_point(Point::new(0.0, 90.0), FillRule::NonZero));
    }

    #[test]
    fn circle_and_cubic_sdf_and_closest_point() {
        let mut pb = PathBuilder::new();
        pb.push_circle(100.0, 100.0, 50.0);
        let circle = pb.finish();

        // Analytical circle SDF: d(p) = dist(p, center) - radius
        for &(px, py) in &[
            (100.0f32, 100.0),
            (120.0, 100.0),
            (150.0, 100.0),
            (170.0, 140.0),
            (30.0, 60.0),
        ] {
            let p = Point::new(px, py);
            let sdf = circle.signed_distance(p, FillRule::NonZero).unwrap();
            let exact = p.distance(Point::new(100.0, 100.0)) - 50.0;
            assert!(
                abs_f32(sdf - exact) < 0.05,
                "p=({px},{py}) sdf={sdf} exact={exact}"
            );
        }

        // Cubic S-curve closest point orthogonality check
        let mut cpb = PathBuilder::new();
        cpb.move_to(0.0, 0.0).cubic_to(30.0, 100.0, 70.0, -100.0, 100.0, 0.0);
        let curve = cpb.finish();
        let query = Point::new(50.0, 25.0);
        let res = curve.closest_point(query).unwrap();
        assert!(res.t > 0.0 && res.t < 1.0);
        // Verify no sampled point on curve is closer than Newton-refined result
        for i in 0..=1000 {
            let t = (i as f32) / 1000.0;
            let mt = 1.0 - t;
            let x = 3.0 * mt * mt * t * 30.0 + 3.0 * mt * t * t * 70.0 + t * t * t * 100.0;
            let y = 3.0 * mt * mt * t * 100.0 + 3.0 * mt * t * t * (-100.0);
            let d = Point::new(x, y).distance(query);
            assert!(res.distance <= d + 1e-4, "res={} sample={}", res.distance, d);
        }
    }
}
