#![forbid(unsafe_code)]

use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::flatten::flatten_path_segments;
use crate::geom::path::Path;
use crate::math::affine::{Affine2D, Point};
use crate::math::float::{abs_f64, round_ties_even_f64};

// Grid bound 2^38 guarantees all i128 cross products (<= 2^79) and rational numerators
// (t_num * dx <= 2^118) fit strictly inside i128 with zero overflow.
pub const GRID_MAX: i64 = 1 << 38;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Default)]
pub struct GridPoint {
    pub x: i64,
    pub y: i64,
}

impl GridPoint {
    #[inline]
    pub const fn new(x: i64, y: i64) -> Self {
        Self { x, y }
    }
}

pub type HotPixel = GridPoint;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct GridSeg {
    pub a: GridPoint,
    pub b: GridPoint,
    pub poly_id: u8,
    pub dir: i8,
}

impl GridSeg {
    #[inline]
    pub const fn new(a: GridPoint, b: GridPoint, poly_id: u8, dir: i8) -> Self {
        Self { a, b, poly_id, dir }
    }

    #[inline]
    pub const fn is_degenerate(&self) -> bool {
        self.a.x == self.b.x && self.a.y == self.b.y
    }

    #[inline]
    pub const fn is_vertical(&self) -> bool {
        self.a.x == self.b.x
    }

    // Returns endpoints ordered lexicographically (lo <= hi) and whether orientation was flipped.
    #[inline]
    pub fn ordered_endpoints(&self) -> (GridPoint, GridPoint, bool) {
        if self.a <= self.b {
            (self.a, self.b, false)
        } else {
            (self.b, self.a, true)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridTransform {
    origin_x: f64,
    origin_y: f64,
    scale: f64,
    inv_scale: f64,
}

impl GridTransform {
    pub fn from_paths(paths: &[&Path], tolerance: f32) -> Self {
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;

        for &path in paths {
            flatten_path_segments(path, Affine2D::IDENTITY, tolerance, true, &mut |a, b| {
                for p in [a, b] {
                    let x = p.x as f64;
                    let y = p.y as f64;
                    if x.is_finite() && y.is_finite() {
                        if x < min_x {
                            min_x = x;
                        }
                        if x > max_x {
                            max_x = x;
                        }
                        if y < min_y {
                            min_y = y;
                        }
                        if y > max_y {
                            max_y = y;
                        }
                    }
                }
            });
        }

        Self::from_bounds(min_x, min_y, max_x, max_y)
    }

    pub fn from_bounds(min_x: f64, min_y: f64, max_x: f64, max_y: f64) -> Self {
        if !min_x.is_finite() || !min_y.is_finite() || !max_x.is_finite() || !max_y.is_finite() {
            return Self {
                origin_x: 0.0,
                origin_y: 0.0,
                scale: 1.0,
                inv_scale: 1.0,
            };
        }
        // Compute midpoint without overflow even for extreme +/-1e30 bounds
        let origin_x = 0.5 * min_x + 0.5 * max_x;
        let origin_y = 0.5 * min_y + 0.5 * max_y;
        let hx = abs_f64(max_x - origin_x).max(abs_f64(origin_x - min_x));
        let hy = abs_f64(max_y - origin_y).max(abs_f64(origin_y - min_y));
        let half_span = hx.max(hy);

        if !half_span.is_finite() || half_span <= 1e-300 {
            return Self {
                origin_x,
                origin_y,
                scale: 1.0,
                inv_scale: 1.0,
            };
        }

        let scale = (GRID_MAX as f64) / half_span;
        let inv_scale = half_span / (GRID_MAX as f64);
        Self {
            origin_x,
            origin_y,
            scale,
            inv_scale,
        }
    }

    #[inline]
    pub fn to_grid(&self, p: Point) -> GridPoint {
        let gx = round_ties_even_f64(((p.x as f64) - self.origin_x) * self.scale);
        let gy = round_ties_even_f64(((p.y as f64) - self.origin_y) * self.scale);
        let max_f = GRID_MAX as f64;
        GridPoint {
            x: gx.clamp(-max_f, max_f) as i64,
            y: gy.clamp(-max_f, max_f) as i64,
        }
    }

    #[inline]
    pub fn from_grid(&self, gp: GridPoint) -> Point {
        Point::new(
            (self.origin_x + (gp.x as f64) * self.inv_scale) as f32,
            (self.origin_y + (gp.y as f64) * self.inv_scale) as f32,
        )
    }

    pub fn flatten_path_to_grid(&self, path: &Path, poly_id: u8, tolerance: f32, out: &mut Vec<GridSeg>) {
        flatten_path_segments(path, Affine2D::IDENTITY, tolerance, true, &mut |a, b| {
            if !a.is_finite() || !b.is_finite() {
                return;
            }
            let ga = self.to_grid(a);
            let gb = self.to_grid(b);
            if ga != gb {
                out.push(GridSeg::new(ga, gb, poly_id, 1));
            }
        });
    }
}

// Exact 2D orientation determinant in i128: > 0 Left (CCW), == 0 Collinear, < 0 Right (CW).
#[inline]
pub const fn orient2d_det(a: GridPoint, b: GridPoint, c: GridPoint) -> i128 {
    let abx = (b.x as i128) - (a.x as i128);
    let aby = (b.y as i128) - (a.y as i128);
    let acx = (c.x as i128) - (a.x as i128);
    let acy = (c.y as i128) - (a.y as i128);
    abx * acy - aby * acx
}

#[inline]
pub fn orient2d(a: GridPoint, b: GridPoint, c: GridPoint) -> Ordering {
    orient2d_det(a, b, c).cmp(&0)
}

// Computes exact round-to-nearest-ties-to-even(base + num / den) with den > 0.
// Symmetric across segment ordering even on exact half-integer ties.
#[inline]
fn add_div_round_ties_even_i128(base: i64, num: i128, den: i128) -> i64 {
    debug_assert!(den > 0);
    let mut q = (base as i128) + (num / den);
    let r = num % den;
    let twice_r = r.unsigned_abs() << 1;
    let den_u = den as u128;
    if twice_r > den_u || (twice_r == den_u && (q & 1) != 0) {
        q += if r >= 0 { 1 } else { -1 };
    }
    q.clamp(-GRID_MAX as i128, GRID_MAX as i128) as i64
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentIntersection {
    None,
    Point(HotPixel),
    Overlap(HotPixel, HotPixel),
}

// Exact i128 rational segment-segment intersection with deterministic ties-to-even grid rounding.
pub fn intersect_segments(s1: GridSeg, s2: GridSeg) -> SegmentIntersection {
    let a = s1.a;
    let b = s1.b;
    let c = s2.a;
    let d = s2.b;

    // Fast AABB rejection
    if a.x.max(b.x) < c.x.min(d.x)
        || c.x.max(d.x) < a.x.min(b.x)
        || a.y.max(b.y) < c.y.min(d.y)
        || c.y.max(d.y) < a.y.min(b.y)
    {
        return SegmentIntersection::None;
    }

    let d1x = (b.x as i128) - (a.x as i128);
    let d1y = (b.y as i128) - (a.y as i128);
    let d2x = (d.x as i128) - (c.x as i128);
    let d2y = (d.y as i128) - (c.y as i128);
    let wx = (c.x as i128) - (a.x as i128);
    let wy = (c.y as i128) - (a.y as i128);

    let denom = d1x * d2y - d1y * d2x;
    if denom != 0 {
        let mut t_num = wx * d2y - wy * d2x;
        let mut u_num = wx * d1y - wy * d1x;
        let den = if denom < 0 {
            t_num = -t_num;
            u_num = -u_num;
            -denom
        } else {
            denom
        };

        if t_num < 0 || t_num > den || u_num < 0 || u_num > den {
            return SegmentIntersection::None;
        }

        if t_num == 0 {
            return SegmentIntersection::Point(a);
        }
        if t_num == den {
            return SegmentIntersection::Point(b);
        }
        if u_num == 0 {
            return SegmentIntersection::Point(c);
        }
        if u_num == den {
            return SegmentIntersection::Point(d);
        }

        let x = add_div_round_ties_even_i128(a.x, t_num * d1x, den);
        let y = add_div_round_ties_even_i128(a.y, t_num * d1y, den);
        return SegmentIntersection::Point(GridPoint::new(x, y));
    }

    // Parallel: check exact collinearity
    if orient2d_det(a, b, c) != 0 {
        return SegmentIntersection::None;
    }

    let (a_lo, a_hi) = if a <= b { (a, b) } else { (b, a) };
    let (c_lo, c_hi) = if c <= d { (c, d) } else { (d, c) };
    let lo = a_lo.max(c_lo);
    let hi = a_hi.min(c_hi);
    match lo.cmp(&hi) {
        Ordering::Greater => SegmentIntersection::None,
        Ordering::Equal => SegmentIntersection::Point(lo),
        Ordering::Less => SegmentIntersection::Overlap(lo, hi),
    }
}

#[derive(Debug, Clone, Copy)]
struct SweepSeg {
    lo: GridPoint,
    hi: GridPoint,
    orig: GridSeg,
}

impl SweepSeg {
    #[inline]
    fn from_grid_seg(seg: GridSeg) -> Self {
        let (lo, hi, _) = seg.ordered_endpoints();
        Self { lo, hi, orig: seg }
    }

    // Exact rational y(x) = q + r / dx with 0 <= r < dx (for non-vertical seg with lo.x <= x <= hi.x).
    #[inline]
    fn eval_y_rational(&self, x: i64) -> (i128, i128, i128) {
        let dx = (self.hi.x as i128) - (self.lo.x as i128);
        let dy = (self.hi.y as i128) - (self.lo.y as i128);
        let tx = (x as i128) - (self.lo.x as i128);
        let num = dy * tx;
        let q = (self.lo.y as i128) + num.div_euclid(dx);
        let r = num.rem_euclid(dx);
        (q, r, dx)
    }
}

// Exact i128 comparator of two non-vertical segments at vertical sweep line `x` (tie-broken by slope).
#[inline]
fn cmp_segs_at_x(a: &SweepSeg, b: &SweepSeg, x: i64) -> Ordering {
    let (qa, ra, dxa) = a.eval_y_rational(x);
    let (qb, rb, dxb) = b.eval_y_rational(x);
    match qa.cmp(&qb) {
        Ordering::Equal => match (ra * dxb).cmp(&(rb * dxa)) {
            Ordering::Equal => {
                let dya = (a.hi.y as i128) - (a.lo.y as i128);
                let dyb = (b.hi.y as i128) - (b.lo.y as i128);
                (dya * dxb).cmp(&(dyb * dxa))
            }
            ord => ord,
        },
        ord => ord,
    }
}

#[inline]
fn cmp_seg_y_to_point(seg: &SweepSeg, x: i64, y: i64) -> Ordering {
    let (q, r, _) = seg.eval_y_rational(x);
    match q.cmp(&(y as i128)) {
        Ordering::Equal => {
            if r == 0 {
                Ordering::Equal
            } else {
                Ordering::Greater
            }
        }
        ord => ord,
    }
}

#[inline]
fn cmp_y_only_at_x(a: &SweepSeg, b: &SweepSeg, x: i64) -> Ordering {
    let (qa, ra, dxa) = a.eval_y_rational(x);
    let (qb, rb, dxb) = b.eval_y_rational(x);
    match qa.cmp(&qb) {
        Ordering::Equal => (ra * dxb).cmp(&(rb * dxa)),
        ord => ord,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum EventKind {
    Insert = 0,
    Vertical = 1,
    Remove = 2,
}

#[derive(Debug, Clone, Copy)]
struct SweepEvent {
    x: i64,
    kind: EventKind,
    seg_idx: usize,
}

#[inline]
fn record_intersection(inter: SegmentIntersection, out: &mut Vec<HotPixel>) {
    match inter {
        SegmentIntersection::None => {}
        SegmentIntersection::Point(p) => out.push(p),
        SegmentIntersection::Overlap(p0, p1) => {
            out.push(p0);
            out.push(p1);
        }
    }
}

// Slab-sorted active-edge sweep-line with exact i128 vertical comparator.
// Collects all segment endpoints and interior intersections as sorted, deduplicated HotPixels.
pub fn collect_hot_pixels(segments: &[GridSeg]) -> Vec<HotPixel> {
    let mut hot_pixels: Vec<HotPixel> = Vec::with_capacity(segments.len() * 2);
    let mut sweep_segs: Vec<SweepSeg> = Vec::with_capacity(segments.len());
    let mut events: Vec<SweepEvent> = Vec::with_capacity(segments.len() * 2);

    for &seg in segments {
        if seg.is_degenerate() {
            hot_pixels.push(seg.a);
            continue;
        }
        let s = SweepSeg::from_grid_seg(seg);
        let idx = sweep_segs.len();
        sweep_segs.push(s);

        hot_pixels.push(s.lo);
        hot_pixels.push(s.hi);

        if s.lo.x == s.hi.x {
            events.push(SweepEvent {
                x: s.lo.x,
                kind: EventKind::Vertical,
                seg_idx: idx,
            });
        } else {
            events.push(SweepEvent {
                x: s.lo.x,
                kind: EventKind::Insert,
                seg_idx: idx,
            });
            events.push(SweepEvent {
                x: s.hi.x,
                kind: EventKind::Remove,
                seg_idx: idx,
            });
        }
    }

    if events.is_empty() {
        hot_pixels.sort_unstable();
        hot_pixels.dedup();
        return hot_pixels;
    }

    events.sort_unstable_by(|a, b| {
        a.x.cmp(&b.x)
            .then_with(|| (a.kind as u8).cmp(&(b.kind as u8)))
            .then_with(|| a.seg_idx.cmp(&b.seg_idx))
    });

    // Active non-vertical segment indices, maintained in exact y(sweep_x) order.
    let mut active: Vec<usize> = Vec::new();
    let mut ev_i = 0usize;

    while ev_i < events.len() {
        let x = events[ev_i].x;

        // 1. Re-sort active segments from previous slab's y-order into exact y(x) order.
        // Every y-order inversion corresponds 1-to-1 to an interior intersection inside (prev_x, x].
        for i in 1..active.len() {
            let mut j = i;
            while j > 0 {
                let left_idx = active[j - 1];
                let right_idx = active[j];
                let ord = cmp_segs_at_x(&sweep_segs[left_idx], &sweep_segs[right_idx], x);
                if ord == Ordering::Greater {
                    record_intersection(
                        intersect_segments(sweep_segs[left_idx].orig, sweep_segs[right_idx].orig),
                        &mut hot_pixels,
                    );
                    active.swap(j - 1, j);
                    j -= 1;
                } else {
                    break;
                }
            }
        }

        // Check equal-y runs at `x` (catches touching intersections without order inversion)
        let mut run = 0usize;
        while run + 1 < active.len() {
            let mut end = run + 1;
            while end < active.len()
                && cmp_y_only_at_x(&sweep_segs[active[run]], &sweep_segs[active[end]], x)
                    == Ordering::Equal
            {
                end += 1;
            }
            if end - run > 1 {
                // Record intersections between first segment of equal-y run and others if non-collinear
                let base_idx = active[run];
                for &other_idx in &active[run + 1..end] {
                    record_intersection(
                        intersect_segments(sweep_segs[base_idx].orig, sweep_segs[other_idx].orig),
                        &mut hot_pixels,
                    );
                }
            }
            run = end;
        }

        // 2. Process all Insert events at `x`
        while ev_i < events.len() && events[ev_i].x == x && events[ev_i].kind == EventKind::Insert {
            let seg_idx = events[ev_i].seg_idx;
            let pos = active.partition_point(|&a_idx| {
                cmp_segs_at_x(&sweep_segs[a_idx], &sweep_segs[seg_idx], x) == Ordering::Less
            });
            active.insert(pos, seg_idx);
            ev_i += 1;
        }

        // 3. Process all Vertical segments at `x` via binary search over active y(x) order
        while ev_i < events.len()
            && events[ev_i].x == x
            && events[ev_i].kind == EventKind::Vertical
        {
            let v_idx = events[ev_i].seg_idx;
            let v = &sweep_segs[v_idx];
            let lo_pos = active
                .partition_point(|&a_idx| cmp_seg_y_to_point(&sweep_segs[a_idx], x, v.lo.y) == Ordering::Less);
            let hi_pos = active.partition_point(|&a_idx| {
                cmp_seg_y_to_point(&sweep_segs[a_idx], x, v.hi.y) != Ordering::Greater
            });
            for &a_idx in &active[lo_pos..hi_pos] {
                record_intersection(
                    intersect_segments(v.orig, sweep_segs[a_idx].orig),
                    &mut hot_pixels,
                );
            }
            ev_i += 1;
        }

        // 4. Process all Remove events at `x`
        while ev_i < events.len() && events[ev_i].x == x && events[ev_i].kind == EventKind::Remove {
            let seg_idx = events[ev_i].seg_idx;
            if let Some(pos) = active.iter().position(|&id| id == seg_idx) {
                active.remove(pos);
            }
            ev_i += 1;
        }
    }

    hot_pixels.sort_unstable();
    hot_pixels.dedup();
    hot_pixels
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::PathBuilder;
    use crate::math::float::sincos_f32;

    fn brute_force_hot_pixels(segments: &[GridSeg]) -> Vec<HotPixel> {
        let mut pts = Vec::new();
        for &s in segments {
            pts.push(s.a);
            pts.push(s.b);
        }
        for i in 0..segments.len() {
            for j in (i + 1)..segments.len() {
                record_intersection(intersect_segments(segments[i], segments[j]), &mut pts);
            }
        }
        pts.sort_unstable();
        pts.dedup();
        pts
    }

    #[test]
    fn exact_i128_orient2d_and_symmetric_half_integer_intersections() {
        let a = GridPoint::new(0, 0);
        let b = GridPoint::new(10, 0);
        assert_eq!(orient2d(a, b, GridPoint::new(5, 5)), Ordering::Greater);
        assert_eq!(orient2d(a, b, GridPoint::new(5, 0)), Ordering::Equal);
        assert_eq!(orient2d(a, b, GridPoint::new(5, -5)), Ordering::Less);

        // Exact half-integer crossing at (1.5, 1.5) -> ties-to-even must give (2, 2) regardless of segment order!
        let s1 = GridSeg::new(GridPoint::new(0, 0), GridPoint::new(3, 3), 0, 1);
        let s2 = GridSeg::new(GridPoint::new(0, 3), GridPoint::new(3, 0), 1, 1);
        let i12 = intersect_segments(s1, s2);
        let i21 = intersect_segments(s2, s1);
        assert_eq!(i12, SegmentIntersection::Point(GridPoint::new(2, 2)));
        assert_eq!(i12, i21);

        // Reversed segment endpoints must also yield identical result
        let s1_rev = GridSeg::new(s1.b, s1.a, 0, -1);
        let s2_rev = GridSeg::new(s2.b, s2.a, 1, -1);
        assert_eq!(intersect_segments(s1_rev, s2_rev), i12);

        // Full GRID_MAX scale cross product must never overflow i128
        let big1 = GridSeg::new(
            GridPoint::new(-GRID_MAX, -GRID_MAX),
            GridPoint::new(GRID_MAX, GRID_MAX),
            0,
            1,
        );
        let big2 = GridSeg::new(
            GridPoint::new(-GRID_MAX, GRID_MAX),
            GridPoint::new(GRID_MAX, -GRID_MAX),
            1,
            1,
        );
        assert_eq!(
            intersect_segments(big1, big2),
            SegmentIntersection::Point(GridPoint::new(0, 0))
        );
    }

    #[test]
    fn torture_100_coincident_collinear_and_50_multi_intersection_point() {
        // 1. 100 coincident collinear segments on y = 1000
        let mut collinear = Vec::new();
        for i in 0..100i64 {
            collinear.push(GridSeg::new(
                GridPoint::new(i * 10, 1000),
                GridPoint::new(i * 10 + 500, 1000),
                (i & 1) as u8,
                1,
            ));
        }
        let hp_col = collect_hot_pixels(&collinear);
        let ref_col = brute_force_hot_pixels(&collinear);
        assert_eq!(hp_col, ref_col);

        // 2. 50 segments all intersecting at a single central point (0, 0)
        let mut star = Vec::new();
        for i in 0..50 {
            let ang = (i as f32) * (core::f32::consts::PI / 50.0);
            let (s, c) = sincos_f32(ang);
            let dx = (c as f64 * 100_000.0) as i64;
            let dy = (s as f64 * 100_000.0) as i64;
            star.push(GridSeg::new(
                GridPoint::new(-dx, -dy),
                GridPoint::new(dx, dy),
                (i & 1) as u8,
                1,
            ));
        }
        let hp_star = collect_hot_pixels(&star);
        let ref_star = brute_force_hot_pixels(&star);
        assert!(hp_star.binary_search(&GridPoint::new(0, 0)).is_ok());
        assert_eq!(hp_star, ref_star);
    }

    #[test]
    fn torture_nearly_parallel_vertical_horizontal_grid_and_extreme_1e25() {
        // 1. Nearly parallel segments differing by 1e-6 crossing near x = 50.0
        let mut p1 = PathBuilder::new();
        p1.move_to(0.0, 0.0).line_to(100.0, 1e-6);
        let mut p2 = PathBuilder::new();
        p2.move_to(0.0, 1e-6).line_to(100.0, 0.0);
        let path1 = p1.finish();
        let path2 = p2.finish();
        let xf = GridTransform::from_paths(&[&path1, &path2], 0.25);
        let s1 = GridSeg::new(xf.to_grid(Point::new(0.0, 0.0)), xf.to_grid(Point::new(100.0, 1e-6)), 0, 1);
        let s2 = GridSeg::new(xf.to_grid(Point::new(0.0, 1e-6)), xf.to_grid(Point::new(100.0, 0.0)), 1, 1);
        let hp_par = collect_hot_pixels(&[s1, s2]);
        assert_eq!(hp_par.len(), 5); // 4 endpoints + 1 exact midpoint crossing
        let mid_grid = xf.to_grid(Point::new(50.0, 0.5e-6));
        assert!(hp_par.iter().any(|&p| (p.x - mid_grid.x).abs() <= 1 && (p.y - mid_grid.y).abs() <= 1));

        // 2. 20x20 vertical & horizontal grid -> 400 interior intersections + 80 endpoints = 480 hot pixels
        let mut grid_segs = Vec::new();
        for i in 1..=20i64 {
            grid_segs.push(GridSeg::new(
                GridPoint::new(i * 100, 0),
                GridPoint::new(i * 100, 2100),
                0,
                1,
            ));
            grid_segs.push(GridSeg::new(
                GridPoint::new(0, i * 100),
                GridPoint::new(2100, i * 100),
                1,
                1,
            ));
        }
        let hp_grid = collect_hot_pixels(&grid_segs);
        let ref_grid = brute_force_hot_pixels(&grid_segs);
        assert_eq!(hp_grid.len(), 480);
        assert_eq!(hp_grid, ref_grid);

        // 3. Extreme coordinates (+/- 1e25) with crossing diagonals -> zero overflow, exact midpoint
        let big_xf = GridTransform::from_bounds(-1e25, -1e25, 1e25, 1e25);
        let e1 = GridSeg::new(
            big_xf.to_grid(Point::new(-1e25, -1e25)),
            big_xf.to_grid(Point::new(1e25, 1e25)),
            0,
            1,
        );
        let e2 = GridSeg::new(
            big_xf.to_grid(Point::new(-1e25, 1e25)),
            big_xf.to_grid(Point::new(1e25, -1e25)),
            1,
            1,
        );
        let hp_big = collect_hot_pixels(&[e1, e2]);
        assert!(hp_big.binary_search(&GridPoint::new(0, 0)).is_ok());
        assert_eq!(big_xf.from_grid(GridPoint::new(0, 0)), Point::ZERO);
    }

    #[test]
    fn randomized_sweep_matches_brute_force_exact() {
        let mut seed = 0x517C_C1B7_2722_0A95u64;
        let mut next_i64 = || -> i64 {
            seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((seed >> 48) as i64) - 32_768
        };

        for _ in 0..20 {
            let mut segs = Vec::with_capacity(60);
            for i in 0..60 {
                let x0 = next_i64() / 4;
                let y0 = next_i64() / 4;
                let x1 = if i % 7 == 0 { x0 } else { next_i64() / 4 };
                let y1 = if i % 9 == 0 { y0 } else { next_i64() / 4 };
                segs.push(GridSeg::new(
                    GridPoint::new(x0, y0),
                    GridPoint::new(x1, y1),
                    (i & 1) as u8,
                    1,
                ));
            }
            let sweep = collect_hot_pixels(&segs);
            let brute = brute_force_hot_pixels(&segs);
            assert_eq!(sweep, brute);
        }
    }
}
