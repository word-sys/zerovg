#![forbid(unsafe_code)]

use crate::math::affine::{Affine2D, Point, Rect};
use crate::math::arena::{EdgeSeg, FrameArena};
use crate::math::fixed::Fixed24_8;
use crate::math::float::{ceil_f32, sqrt_f32};

use super::bounds::{cubic_bounds, path_visible_in_viewport, quad_bounds};
use super::path::{Path, PathEvent};

pub const DEFAULT_TOLERANCE: f32 = 0.25;
pub const MAX_SUBDIVS: usize = 1024;

#[inline]
pub fn wang_quad_subdivs(p0: Point, p1: Point, p2: Point, tolerance: f32) -> usize {
    let eps = tolerance.max(1e-4);
    let dx = p0.x - 2.0 * p1.x + p2.x;
    let dy = p0.y - 2.0 * p1.y + p2.y;
    let m = sqrt_f32(dx * dx + dy * dy);
    if !m.is_finite() || m <= 0.0 {
        return 1;
    }
    let n = ceil_f32(sqrt_f32(m / (4.0 * eps))) as usize;
    n.clamp(1, MAX_SUBDIVS)
}

#[inline]
pub fn wang_cubic_subdivs(p0: Point, p1: Point, p2: Point, p3: Point, tolerance: f32) -> usize {
    let eps = tolerance.max(1e-4);
    let d1x = p0.x - 2.0 * p1.x + p2.x;
    let d1y = p0.y - 2.0 * p1.y + p2.y;
    let d2x = p1.x - 2.0 * p2.x + p3.x;
    let d2y = p1.y - 2.0 * p2.y + p3.y;
    let m1 = sqrt_f32(d1x * d1x + d1y * d1y);
    let m2 = sqrt_f32(d2x * d2x + d2y * d2y);
    let m = if m1 > m2 { m1 } else { m2 };
    if !m.is_finite() || m <= 0.0 {
        return 1;
    }
    let n = ceil_f32(sqrt_f32((0.75 * m) / eps)) as usize;
    n.clamp(1, MAX_SUBDIVS)
}

pub fn flatten_quad<F: FnMut(Point, Point)>(
    p0: Point,
    p1: Point,
    p2: Point,
    tolerance: f32,
    emit: &mut F,
) {
    let n = wang_quad_subdivs(p0, p1, p2, tolerance);
    if n == 1 {
        emit(p0, p2);
        return;
    }
    let (x0, y0) = (p0.x as f64, p0.y as f64);
    let (x1, y1) = (p1.x as f64, p1.y as f64);
    let (x2, y2) = (p2.x as f64, p2.y as f64);
    let ax = x0 - 2.0 * x1 + x2;
    let ay = y0 - 2.0 * y1 + y2;
    let bx = 2.0 * (x1 - x0);
    let by = 2.0 * (y1 - y0);
    let inv_n = 1.0 / (n as f64);

    let mut prev = p0;
    for i in 1..n {
        let t = (i as f64) * inv_n;
        let cur = Point::new(
            ((ax * t + bx) * t + x0) as f32,
            ((ay * t + by) * t + y0) as f32,
        );
        emit(prev, cur);
        prev = cur;
    }
    emit(prev, p2);
}

pub fn flatten_cubic<F: FnMut(Point, Point)>(
    p0: Point,
    p1: Point,
    p2: Point,
    p3: Point,
    tolerance: f32,
    emit: &mut F,
) {
    let n = wang_cubic_subdivs(p0, p1, p2, p3, tolerance);
    if n == 1 {
        emit(p0, p3);
        return;
    }
    let (x0, y0) = (p0.x as f64, p0.y as f64);
    let (x1, y1) = (p1.x as f64, p1.y as f64);
    let (x2, y2) = (p2.x as f64, p2.y as f64);
    let (x3, y3) = (p3.x as f64, p3.y as f64);
    let ax = -x0 + 3.0 * x1 - 3.0 * x2 + x3;
    let ay = -y0 + 3.0 * y1 - 3.0 * y2 + y3;
    let bx = 3.0 * (x0 - 2.0 * x1 + x2);
    let by = 3.0 * (y0 - 2.0 * y1 + y2);
    let cx = 3.0 * (x1 - x0);
    let cy = 3.0 * (y1 - y0);
    let inv_n = 1.0 / (n as f64);

    let mut prev = p0;
    for i in 1..n {
        let t = (i as f64) * inv_n;
        let cur = Point::new(
            (((ax * t + bx) * t + cx) * t + x0) as f32,
            (((ay * t + by) * t + cy) * t + y0) as f32,
        );
        emit(prev, cur);
        prev = cur;
    }
    emit(prev, p3);
}

pub fn flatten_path_segments<F: FnMut(Point, Point)>(
    path: &Path,
    transform: Affine2D,
    tolerance: f32,
    close_open_subpaths: bool,
    emit: &mut F,
) {
    if path.is_empty() || !transform.is_finite() {
        return;
    }
    let is_id = transform.is_identity();
    let xform = |p: Point| if is_id { p } else { transform.transform_point(p) };

    let mut cur = Point::ZERO;
    let mut start = Point::ZERO;
    let mut in_subpath = false;

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if close_open_subpaths && in_subpath && cur != start {
                    emit(cur, start);
                }
                let tp = xform(p);
                cur = tp;
                start = tp;
                in_subpath = true;
            }
            PathEvent::LineTo(p) => {
                let tp = xform(p);
                if in_subpath {
                    emit(cur, tp);
                } else {
                    start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::QuadTo(c, p) => {
                let tc = xform(c);
                let tp = xform(p);
                if in_subpath {
                    flatten_quad(cur, tc, tp, tolerance, emit);
                } else {
                    start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                let tc1 = xform(c1);
                let tc2 = xform(c2);
                let tp = xform(p);
                if in_subpath {
                    flatten_cubic(cur, tc1, tc2, tp, tolerance, emit);
                } else {
                    start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::Close => {
                if in_subpath {
                    if cur != start {
                        emit(cur, start);
                    }
                    cur = start;
                }
            }
        }
    }

    if close_open_subpaths && in_subpath && cur != start {
        emit(cur, start);
    }
}

#[inline]
fn push_edge_fixed(
    edges: &mut alloc::vec::Vec<EdgeSeg>,
    x0: f64,
    y0: f64,
    x1: f64,
    y1: f64,
    w_fixed: i32,
    h_fixed: i32,
) {
    let fy0 = Fixed24_8::from_f64_clamped(y0).raw().clamp(0, h_fixed);
    let fy1 = Fixed24_8::from_f64_clamped(y1).raw().clamp(0, h_fixed);
    if fy0 == fy1 {
        return;
    }
    let fx0 = Fixed24_8::from_f64_clamped(x0).raw().clamp(0, w_fixed);
    let fx1 = Fixed24_8::from_f64_clamped(x1).raw().clamp(0, w_fixed);
    edges.push(EdgeSeg {
        x0: fx0,
        y0: fy0,
        x1: fx1,
        y1: fy1,
    });
}

// Clips a directed segment to [0, w] x [0, h] in f64, projecting x < 0 onto x = 0
// so left-edge backdrop winding is preserved exactly.
fn clip_and_emit_segment(
    edges: &mut alloc::vec::Vec<EdgeSeg>,
    p0: Point,
    p1: Point,
    w: f64,
    h: f64,
    w_fixed: i32,
    h_fixed: i32,
) {
    let (mut x0, mut y0) = (p0.x as f64, p0.y as f64);
    let (mut x1, mut y1) = (p1.x as f64, p1.y as f64);
    if !x0.is_finite() || !y0.is_finite() || !x1.is_finite() || !y1.is_finite() {
        return;
    }
    if y0 == y1 {
        return;
    }
    if (y0 <= 0.0 && y1 <= 0.0) || (y0 >= h && y1 >= h) {
        return;
    }

    // 1. Vertical clip to [0, h]
    let dy = y1 - y0;
    let dx = x1 - x0;
    if y0 < 0.0 {
        x0 += dx * ((0.0 - y0) / dy);
        y0 = 0.0;
    } else if y0 > h {
        x0 += dx * ((h - y0) / dy);
        y0 = h;
    }
    if y1 < 0.0 {
        x1 += dx * ((0.0 - y1) / dy);
        y1 = 0.0;
    } else if y1 > h {
        x1 += dx * ((h - y1) / dy);
        y1 = h;
    }
    if y0 == y1 {
        return;
    }

    // 2. Horizontal clip against [0, w] with exact x < 0 projection to x = 0
    if x0 >= w && x1 >= w {
        return;
    }
    if x0 <= 0.0 && x1 <= 0.0 {
        push_edge_fixed(edges, 0.0, y0, 0.0, y1, w_fixed, h_fixed);
        return;
    }

    // Split at x = 0 if the segment crosses the left viewport boundary
    if x0 < 0.0 {
        let t = (0.0 - x0) / (x1 - x0);
        let y_mid = (y0 + (y1 - y0) * t).clamp(0.0, h);
        push_edge_fixed(edges, 0.0, y0, 0.0, y_mid, w_fixed, h_fixed);
        x0 = 0.0;
        y0 = y_mid;
    } else if x1 < 0.0 {
        let t = (0.0 - x0) / (x1 - x0);
        let y_mid = (y0 + (y1 - y0) * t).clamp(0.0, h);
        push_edge_fixed(edges, 0.0, y_mid, 0.0, y1, w_fixed, h_fixed);
        x1 = 0.0;
        y1 = y_mid;
    }

    // Clip remaining x >= 0 segment against right boundary x = w
    if x0 >= w && x1 >= w {
        return;
    }
    if x0 > w {
        let t = (w - x0) / (x1 - x0);
        y0 = (y0 + (y1 - y0) * t).clamp(0.0, h);
        x0 = w;
    } else if x1 > w {
        let t = (w - x0) / (x1 - x0);
        y1 = (y0 + (y1 - y0) * t).clamp(0.0, h);
        x1 = w;
    }

    push_edge_fixed(edges, x0, y0, x1, y1, w_fixed, h_fixed);
}

// Stage 1: transform, analytically cull, adaptively flatten, clip (preserving x < 0 winding),
// and emit 24.8 fixed-point segments directly into FrameArena::edges.
// Returns the range of indices appended to `arena.edges`.
pub fn flatten_path_for_fill(
    path: &Path,
    transform: Affine2D,
    viewport_width: u32,
    viewport_height: u32,
    tolerance: f32,
    arena: &mut FrameArena,
) -> core::ops::Range<usize> {
    let start_idx = arena.edges.len();
    if path.is_empty() || viewport_width == 0 || viewport_height == 0 || !transform.is_finite() {
        return start_idx..start_idx;
    }
    let w_f32 = viewport_width as f32;
    let h_f32 = viewport_height as f32;
    let viewport = Rect::new(0.0, 0.0, w_f32, h_f32);
    if !path_visible_in_viewport(path, transform, viewport) {
        return start_idx..start_idx;
    }

    let w = viewport_width as f64;
    let h = viewport_height as f64;
    let w_fixed = Fixed24_8::from_int(viewport_width as i32).raw();
    let h_fixed = Fixed24_8::from_int(viewport_height as i32).raw();
    let is_id = transform.is_identity();
    let xform = |p: Point| if is_id { p } else { transform.transform_point(p) };

    let edges = &mut arena.edges;
    let mut cur = Point::ZERO;
    let mut sub_start = Point::ZERO;
    let mut in_subpath = false;

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if in_subpath && cur != sub_start {
                    clip_and_emit_segment(edges, cur, sub_start, w, h, w_fixed, h_fixed);
                }
                let tp = xform(p);
                cur = tp;
                sub_start = tp;
                in_subpath = true;
            }
            PathEvent::LineTo(p) => {
                let tp = xform(p);
                if in_subpath {
                    clip_and_emit_segment(edges, cur, tp, w, h, w_fixed, h_fixed);
                } else {
                    sub_start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::QuadTo(c, p) => {
                let tc = xform(c);
                let tp = xform(p);
                if in_subpath {
                    let qb = quad_bounds(cur, tc, tp);
                    if qb.max_y <= 0.0 || qb.min_y >= h_f32 || qb.min_x >= w_f32 {
                        // Entire curve outside vertically or strictly to the right
                    } else if qb.max_x <= 0.0 {
                        // Entire curve strictly left of viewport: project directly onto x = 0
                        clip_and_emit_segment(edges, cur, tp, w, h, w_fixed, h_fixed);
                    } else {
                        flatten_quad(cur, tc, tp, tolerance, &mut |a, b| {
                            clip_and_emit_segment(edges, a, b, w, h, w_fixed, h_fixed);
                        });
                    }
                } else {
                    sub_start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                let tc1 = xform(c1);
                let tc2 = xform(c2);
                let tp = xform(p);
                if in_subpath {
                    let cb = cubic_bounds(cur, tc1, tc2, tp);
                    if cb.max_y <= 0.0 || cb.min_y >= h_f32 || cb.min_x >= w_f32 {
                        // Outside vertically or strictly right
                    } else if cb.max_x <= 0.0 {
                        // Strictly left: single exact vertical backdrop projection
                        clip_and_emit_segment(edges, cur, tp, w, h, w_fixed, h_fixed);
                    } else {
                        flatten_cubic(cur, tc1, tc2, tp, tolerance, &mut |a, b| {
                            clip_and_emit_segment(edges, a, b, w, h, w_fixed, h_fixed);
                        });
                    }
                } else {
                    sub_start = tp;
                    in_subpath = true;
                }
                cur = tp;
            }
            PathEvent::Close => {
                if in_subpath {
                    if cur != sub_start {
                        clip_and_emit_segment(edges, cur, sub_start, w, h, w_fixed, h_fixed);
                    }
                    cur = sub_start;
                }
            }
        }
    }

    if in_subpath && cur != sub_start {
        clip_and_emit_segment(edges, cur, sub_start, w, h, w_fixed, h_fixed);
    }

    start_idx..arena.edges.len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::PathBuilder;
    use crate::math::float::abs_f32;

    fn point_to_seg_dist(p: Point, a: Point, b: Point) -> f32 {
        let ab = b - a;
        let len2 = ab.length_sq();
        if len2 <= 1e-12 {
            return p.distance(a);
        }
        let t = ((p - a).dot(ab) / len2).clamp(0.0, 1.0);
        let proj = Point::new(a.x + ab.x * t, a.y + ab.y * t);
        p.distance(proj)
    }

    #[test]
    fn wang_flattening_hausdorff_error_across_extreme_zooms() {
        let p0 = Point::new(0.0, 0.0);
        let p1 = Point::new(20.0, 90.0);
        let p2 = Point::new(80.0, -40.0);
        let p3 = Point::new(100.0, 50.0);

        for scale in [0.05f32, 1.0, 25.0, 200.0] {
            let m = Affine2D::scale_uniform(scale);
            let tp0 = m.transform_point(p0);
            let tp1 = m.transform_point(p1);
            let tp2 = m.transform_point(p2);
            let tp3 = m.transform_point(p3);

            let mut segs = alloc::vec::Vec::new();
            flatten_cubic(tp0, tp1, tp2, tp3, DEFAULT_TOLERANCE, &mut |a, b| {
                segs.push((a, b));
            });

            // Watertight endpoints & continuity
            assert_eq!(segs.first().unwrap().0, tp0);
            assert_eq!(segs.last().unwrap().1, tp3);
            for w in segs.windows(2) {
                assert_eq!(w[0].1, w[1].0);
            }

            // Verify curve deviation <= DEFAULT_TOLERANCE at dense samples
            let mut max_dev = 0.0f32;
            for i in 0..=200 {
                let t = (i as f32) / 200.0;
                let mt = 1.0 - t;
                let pt = Point::new(
                    mt * mt * mt * tp0.x
                        + 3.0 * mt * mt * t * tp1.x
                        + 3.0 * mt * t * t * tp2.x
                        + t * t * t * tp3.x,
                    mt * mt * mt * tp0.y
                        + 3.0 * mt * mt * t * tp1.y
                        + 3.0 * mt * t * t * tp2.y
                        + t * t * t * tp3.y,
                );
                let mut best = f32::INFINITY;
                for &(a, b) in &segs {
                    best = best.min(point_to_seg_dist(pt, a, b));
                }
                max_dev = max_dev.max(best);
            }
            assert!(
                max_dev <= DEFAULT_TOLERANCE + 1e-3,
                "scale={scale} max_dev={max_dev}"
            );
        }
    }

    #[test]
    fn left_edge_projection_and_huge_coordinate_winding_conservation() {
        // Huge rectangle extending to x = -1_000_000 covering [0, 50) x [10, 90) inside a 100x100 viewport
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(-1_000_000.0, 10.0, 50.0, 90.0));
        let path = pb.finish();

        let mut arena = FrameArena::new();
        let range = flatten_path_for_fill(&path, Affine2D::IDENTITY, 100, 100, DEFAULT_TOLERANCE, &mut arena);
        let edges = &arena.edges[range];
        assert_eq!(edges.len(), 2);

        // One edge at x = 50 going down (+80 px), one projected edge at x = 0 going up (-80 px)
        let mut sum_dy = 0i64;
        let mut saw_left_edge = false;
        let mut saw_right_edge = false;
        for e in edges {
            assert!(e.x0 >= 0 && e.x1 >= 0 && e.x0 <= 100 * 256 && e.x1 <= 100 * 256);
            assert!(e.y0 >= 0 && e.y1 >= 0 && e.y0 <= 100 * 256 && e.y1 <= 100 * 256);
            sum_dy += (e.y1 - e.y0) as i64;
            if e.x0 == 0 && e.x1 == 0 {
                saw_left_edge = true;
                assert_eq!(e.y1 - e.y0, -80 * 256);
            }
            if e.x0 == 50 * 256 && e.x1 == 50 * 256 {
                saw_right_edge = true;
                assert_eq!(e.y1 - e.y0, 80 * 256);
            }
        }
        assert!(saw_left_edge && saw_right_edge);
        assert_eq!(sum_dy, 0);

        // Closed polygon completely outside on left (max_x <= 0) must cull completely (0 edges)
        let mut off = PathBuilder::new();
        off.push_rect(Rect::new(-200.0, 10.0, -10.0, 90.0));
        let off_path = off.finish();
        arena.reset();
        let r_off = flatten_path_for_fill(&off_path, Affine2D::IDENTITY, 100, 100, DEFAULT_TOLERANCE, &mut arena);
        assert!(r_off.is_empty());

        // Circle zoomed so left arc is strictly in x < 0: net winding sum at y=50 must stay exact
        let mut cpb = PathBuilder::new();
        cpb.push_circle(-100.0, 50.0, 150.0);
        let circle = cpb.finish();
        arena.reset();
        let r_circ = flatten_path_for_fill(&circle, Affine2D::IDENTITY, 100, 100, DEFAULT_TOLERANCE, &mut arena);
        let c_edges = &arena.edges[r_circ];
        assert!(!c_edges.is_empty());
        // At scanline y = 50 (12800 in 24.8), signed crossing count across all emitted edges must be 0!
        let probe_y = 50 * 256 + 128;
        let mut net_crossing = 0i32;
        let mut left_crossing = 0i32;
        for e in c_edges {
            if (e.y0 <= probe_y && e.y1 > probe_y) || (e.y1 <= probe_y && e.y0 > probe_y) {
                let dir = if e.y1 > e.y0 { 1 } else { -1 };
                net_crossing += dir;
                if e.x0 == 0 && e.x1 == 0 {
                    left_crossing += dir;
                }
            }
        }
        assert_eq!(net_crossing, 0);
        assert_eq!(abs_f32(left_crossing as f32), 1.0);
    }
}
