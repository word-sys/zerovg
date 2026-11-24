#![forbid(unsafe_code)]

use alloc::vec;
use alloc::vec::Vec;
use core::cmp::Ordering;

use crate::geom::flatten::flatten_path_segments;
use crate::geom::path::{FillRule, Path, PathBuilder};
use crate::geom::stroke::{LineCap, LineJoin, StrokeStyle, expand_stroke_path};
use crate::math::affine::{Affine2D, Point};
use crate::math::float::{abs_f64, round_ties_even_f64};

// Grid bound 2^38 guarantees all i128 cross products (<= 2^79) and rational numerators
// (t_num * dx <= 2^118) fit strictly inside i128 with zero overflow.
pub const GRID_MAX: i64 = 1 << 38;
const MAX_SNAP_ITERS: usize = 16;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BooleanOp {
    Union,
    Intersection,
    Difference,
    Xor,
    Simplify,
}

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

// Exact i128 test: does open segment (a, b) intersect the closed unit square [h - 0.5, h + 0.5]^2?
#[inline]
fn seg_hits_hot_pixel(a: GridPoint, b: GridPoint, h: HotPixel) -> bool {
    if h == a || h == b {
        return false;
    }
    let min_x = a.x.min(b.x);
    let max_x = a.x.max(b.x);
    let min_y = a.y.min(b.y);
    let max_y = a.y.max(b.y);
    if h.x < min_x || h.x > max_x || h.y < min_y || h.y > max_y {
        return false;
    }
    let dx = (b.x as i128) - (a.x as i128);
    let dy = (b.y as i128) - (a.y as i128);
    let c = orient2d_det(a, b, h);
    (c.unsigned_abs() << 1) <= dx.unsigned_abs() + dy.unsigned_abs()
}

// Hobby's Iterated Snap-Rounding: splits every segment passing through any hot-pixel unit square
// and iterates until convergence (zero interior crossings remain).
pub fn hobby_snap_round(input: &[GridSeg]) -> Vec<GridSeg> {
    let mut segs: Vec<GridSeg> = input
        .iter()
        .copied()
        .filter(|s| !s.is_degenerate())
        .collect();
    if segs.is_empty() {
        return segs;
    }

    let mut hits: Vec<(i128, HotPixel)> = Vec::new();
    let mut next_segs: Vec<GridSeg> = Vec::with_capacity(segs.len() * 2);

    for _iter in 0..MAX_SNAP_ITERS {
        let hot_pixels = collect_hot_pixels(&segs);
        if hot_pixels.is_empty() {
            break;
        }
        next_segs.clear();
        let mut any_split = false;

        for &s in &segs {
            let min_x = s.a.x.min(s.b.x);
            let max_x = s.a.x.max(s.b.x);
            let lo = hot_pixels.partition_point(|hp| hp.x < min_x);
            let hi = hot_pixels.partition_point(|hp| hp.x <= max_x);

            hits.clear();
            let dx = (s.b.x as i128) - (s.a.x as i128);
            let dy = (s.b.y as i128) - (s.a.y as i128);
            for &hp in &hot_pixels[lo..hi] {
                if seg_hits_hot_pixel(s.a, s.b, hp) {
                    let proj = ((hp.x as i128) - (s.a.x as i128)) * dx
                        + ((hp.y as i128) - (s.a.y as i128)) * dy;
                    hits.push((proj, hp));
                }
            }

            if hits.is_empty() {
                next_segs.push(s);
            } else {
                any_split = true;
                hits.sort_unstable_by(|u, v| u.0.cmp(&v.0).then_with(|| u.1.cmp(&v.1)));
                let mut prev = s.a;
                for &(_, hp) in &hits {
                    if hp != prev {
                        next_segs.push(GridSeg::new(prev, hp, s.poly_id, s.dir));
                        prev = hp;
                    }
                }
                if prev != s.b {
                    next_segs.push(GridSeg::new(prev, s.b, s.poly_id, s.dir));
                }
            }
        }

        core::mem::swap(&mut segs, &mut next_segs);
        if !any_split {
            break;
        }
    }

    segs
}

#[derive(Debug, Clone, Copy)]
struct ArrEdge {
    lo: GridPoint,
    hi: GridPoint,
    delta_a: i32,
    delta_b: i32,
    right_wa: i32,
    right_wb: i32,
}

impl ArrEdge {
    #[inline]
    fn is_vertical(&self) -> bool {
        self.lo.x == self.hi.x
    }

    #[inline]
    fn as_sweep_seg(&self) -> SweepSeg {
        SweepSeg {
            lo: self.lo,
            hi: self.hi,
            orig: GridSeg::new(self.lo, self.hi, 0, 1),
        }
    }
}

#[inline]
fn is_winding_inside(w: i32, rule: FillRule) -> bool {
    match rule {
        FillRule::NonZero => w != 0,
        FillRule::EvenOdd => (w & 1) != 0,
    }
}

#[inline]
fn eval_boolean_op(op: BooleanOp, a: bool, b: bool) -> bool {
    match op {
        BooleanOp::Union => a || b,
        BooleanOp::Intersection => a && b,
        BooleanOp::Difference => a && !b,
        BooleanOp::Xor => a ^ b,
        BooleanOp::Simplify => a,
    }
}

fn build_arrangement(snapped: &[GridSeg]) -> Vec<ArrEdge> {
    if snapped.is_empty() {
        return Vec::new();
    }
    let mut raw: Vec<(GridPoint, GridPoint, i32, i32)> = Vec::with_capacity(snapped.len());
    for &s in snapped {
        if s.is_degenerate() {
            continue;
        }
        let (lo, hi, flipped) = s.ordered_endpoints();
        let signed = if flipped { -(s.dir as i32) } else { s.dir as i32 };
        let (da, db) = if s.poly_id == 0 {
            (signed, 0)
        } else {
            (0, signed)
        };
        raw.push((lo, hi, da, db));
    }

    raw.sort_unstable_by(|u, v| u.0.cmp(&v.0).then_with(|| u.1.cmp(&v.1)));

    let mut edges: Vec<ArrEdge> = Vec::with_capacity(raw.len());
    for (lo, hi, da, db) in raw {
        if let Some(last) = edges.last_mut()
            && last.lo == lo
            && last.hi == hi
        {
            last.delta_a = last.delta_a.wrapping_add(da);
            last.delta_b = last.delta_b.wrapping_add(db);
            continue;
        }
        edges.push(ArrEdge {
            lo,
            hi,
            delta_a: da,
            delta_b: db,
            right_wa: 0,
            right_wb: 0,
        });
    }

    edges.retain(|e| e.delta_a != 0 || e.delta_b != 0);
    assign_arrangement_windings(&mut edges);
    edges
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum WindEventKind {
    Remove = 0,
    Insert = 1,
    Vertical = 2,
}

#[derive(Debug, Clone, Copy)]
struct WindEvent {
    x: i64,
    kind: WindEventKind,
    edge_idx: usize,
}

fn assign_arrangement_windings(edges: &mut [ArrEdge]) {
    let mut events: Vec<WindEvent> = Vec::with_capacity(edges.len() * 2);
    for (i, e) in edges.iter().enumerate() {
        if e.is_vertical() {
            events.push(WindEvent {
                x: e.lo.x,
                kind: WindEventKind::Vertical,
                edge_idx: i,
            });
        } else {
            events.push(WindEvent {
                x: e.lo.x,
                kind: WindEventKind::Insert,
                edge_idx: i,
            });
            events.push(WindEvent {
                x: e.hi.x,
                kind: WindEventKind::Remove,
                edge_idx: i,
            });
        }
    }

    events.sort_unstable_by(|a, b| {
        a.x.cmp(&b.x)
            .then_with(|| (a.kind as u8).cmp(&(b.kind as u8)))
            .then_with(|| {
                if a.kind == WindEventKind::Insert && b.kind == WindEventKind::Insert {
                    cmp_segs_at_x(
                        &edges[a.edge_idx].as_sweep_seg(),
                        &edges[b.edge_idx].as_sweep_seg(),
                        a.x,
                    )
                } else {
                    a.edge_idx.cmp(&b.edge_idx)
                }
            })
    });

    let mut active: Vec<usize> = Vec::new();
    let mut ev_i = 0usize;

    while ev_i < events.len() {
        let x = events[ev_i].x;

        // 1. Remove edges ending at x
        while ev_i < events.len()
            && events[ev_i].x == x
            && events[ev_i].kind == WindEventKind::Remove
        {
            let idx = events[ev_i].edge_idx;
            if let Some(pos) = active.iter().position(|&id| id == idx) {
                active.remove(pos);
            }
            ev_i += 1;
        }

        // Re-verify vertical ordering at x in case snap-rounding converged with tiny residual touch
        active.sort_unstable_by(|&a, &b| {
            cmp_segs_at_x(&edges[a].as_sweep_seg(), &edges[b].as_sweep_seg(), x)
        });

        // 2. Insert non-vertical edges starting at x (already sorted by (y(x), slope) ascending)
        while ev_i < events.len()
            && events[ev_i].x == x
            && events[ev_i].kind == WindEventKind::Insert
        {
            let idx = events[ev_i].edge_idx;
            let seg = edges[idx].as_sweep_seg();
            let pos = active.partition_point(|&a_idx| {
                cmp_segs_at_x(&edges[a_idx].as_sweep_seg(), &seg, x) == Ordering::Less
            });
            if pos > 0 {
                let below = edges[active[pos - 1]];
                edges[idx].right_wa = below.right_wa.wrapping_add(below.delta_a);
                edges[idx].right_wb = below.right_wb.wrapping_add(below.delta_b);
            }
            active.insert(pos, idx);
            ev_i += 1;
        }

        // 3. Assign right (+x) winding to vertical edges at x
        while ev_i < events.len()
            && events[ev_i].x == x
            && events[ev_i].kind == WindEventKind::Vertical
        {
            let idx = events[ev_i].edge_idx;
            let lo_y = edges[idx].lo.y;
            let pos = active.partition_point(|&a_idx| {
                cmp_seg_y_to_point(&edges[a_idx].as_sweep_seg(), x, lo_y) != Ordering::Greater
            });
            if pos > 0 {
                let below = edges[active[pos - 1]];
                edges[idx].right_wa = below.right_wa.wrapping_add(below.delta_a);
                edges[idx].right_wb = below.right_wb.wrapping_add(below.delta_b);
            }
            ev_i += 1;
        }
    }
}

#[inline]
fn angle_quadrant(dx: i128, dy: i128) -> u8 {
    if dy >= 0 && dx > 0 {
        0
    } else if dx <= 0 && dy > 0 {
        1
    } else if dy <= 0 && dx < 0 {
        2
    } else {
        3
    }
}

// Exact CCW polar angle comparator in i128 (zero trig).
#[inline]
fn cmp_dir_ccw(a: (i128, i128), b: (i128, i128)) -> Ordering {
    let qa = angle_quadrant(a.0, a.1);
    let qb = angle_quadrant(b.0, b.1);
    match qa.cmp(&qb) {
        Ordering::Equal => (b.0 * a.1).cmp(&(a.0 * b.1)),
        ord => ord,
    }
}

fn extract_contours(
    half_edges: &[(GridPoint, GridPoint)],
    xf: &GridTransform,
) -> Path {
    if half_edges.is_empty() {
        return Path::empty();
    }

    // Collect unique vertices and index outgoing half-edges per vertex
    let mut verts: Vec<GridPoint> = Vec::with_capacity(half_edges.len() * 2);
    for &(u, v) in half_edges {
        verts.push(u);
        verts.push(v);
    }
    verts.sort_unstable();
    verts.dedup();

    let mut outgoing: Vec<Vec<(usize, usize)>> = vec![Vec::new(); verts.len()];
    let mut edge_to: Vec<usize> = Vec::with_capacity(half_edges.len());
    let mut edge_dir: Vec<(i128, i128)> = Vec::with_capacity(half_edges.len());

    for (eid, &(u, v)) in half_edges.iter().enumerate() {
        let ui = verts.binary_search(&u).unwrap();
        let vi = verts.binary_search(&v).unwrap();
        outgoing[ui].push((eid, vi));
        edge_to.push(vi);
        edge_dir.push(((v.x as i128) - (u.x as i128), (v.y as i128) - (u.y as i128)));
    }

    // Sort outgoing half-edges at each vertex by CCW polar angle
    for list in &mut outgoing {
        list.sort_unstable_by(|&(ea, _), &(eb, _)| cmp_dir_ccw(edge_dir[ea], edge_dir[eb]));
    }

    let mut used = vec![false; half_edges.len()];
    let mut pb = PathBuilder::new();
    let mut loop_pts: Vec<GridPoint> = Vec::new();
    let mut simplified: Vec<GridPoint> = Vec::new();

    for start_eid in 0..half_edges.len() {
        if used[start_eid] {
            continue;
        }
        loop_pts.clear();
        let start_u = half_edges[start_eid].0;
        let mut cur_eid = start_eid;

        // Defensive cap prevents infinite loops on malformed degenerate graphs
        for _step in 0..=half_edges.len() {
            if used[cur_eid] {
                break;
            }
            used[cur_eid] = true;
            let (u, v) = half_edges[cur_eid];
            loop_pts.push(u);
            if v == start_u {
                break;
            }

            let vi = edge_to[cur_eid];
            let rev_dir = ((u.x as i128) - (v.x as i128), (u.y as i128) - (v.y as i128));
            let list = &outgoing[vi];
            if list.is_empty() {
                break;
            }

            // Pick first unused outgoing edge clockwise from rev_dir (tightest left-interior turn)
            let pos = list.partition_point(|&(eid, _)| cmp_dir_ccw(edge_dir[eid], rev_dir) == Ordering::Less);
            let mut chosen: Option<usize> = None;
            let n = list.len();
            for offset in 1..=n {
                let idx = (pos + n - offset) % n;
                let cand = list[idx].0;
                if !used[cand] {
                    chosen = Some(cand);
                    break;
                }
            }
            match chosen {
                Some(next_eid) => cur_eid = next_eid,
                None => break,
            }
        }

        // Collapse collinear consecutive vertices (including wrap-around seam)
        simplified.clear();
        for &p in &loop_pts {
            while simplified.len() >= 2 {
                let a = simplified[simplified.len() - 2];
                let b = simplified[simplified.len() - 1];
                if orient2d_det(a, b, p) == 0 {
                    simplified.pop();
                } else {
                    break;
                }
            }
            if simplified.last().copied() != Some(p) {
                simplified.push(p);
            }
        }
        // Trim wrap-around collinearity
        loop {
            let n = simplified.len();
            if n < 3 {
                break;
            }
            if orient2d_det(simplified[n - 2], simplified[n - 1], simplified[0]) == 0 {
                simplified.pop();
            } else if orient2d_det(simplified[n - 1], simplified[0], simplified[1]) == 0 {
                simplified.remove(0);
            } else {
                break;
            }
        }

        if simplified.len() >= 3 {
            let p0 = xf.from_grid(simplified[0]);
            pb.move_to(p0.x, p0.y);
            for &gp in &simplified[1..] {
                let p = xf.from_grid(gp);
                pb.line_to(p.x, p.y);
            }
            pb.close();
        }
    }

    pb.finish()
}

// Executes a snap-rounded exact boolean operation on two paths.
pub fn boolean_op(
    subject: &Path,
    clip: &Path,
    op: BooleanOp,
    subject_rule: FillRule,
    clip_rule: FillRule,
    tolerance: f32,
) -> Path {
    let tol = if tolerance.is_finite() && tolerance > 0.0 {
        tolerance.max(1e-4)
    } else {
        0.25
    };

    if subject.is_empty() && clip.is_empty() {
        return Path::empty();
    }
    match op {
        BooleanOp::Intersection | BooleanOp::Difference | BooleanOp::Simplify
            if subject.is_empty() =>
        {
            return Path::empty();
        }
        BooleanOp::Intersection if clip.is_empty() => {
            return Path::empty();
        }
        _ => {}
    }

    let xf = if op == BooleanOp::Simplify {
        GridTransform::from_paths(&[subject], tol)
    } else {
        GridTransform::from_paths(&[subject, clip], tol)
    };

    let mut grid_segs: Vec<GridSeg> = Vec::new();
    xf.flatten_path_to_grid(subject, 0, tol, &mut grid_segs);
    if op != BooleanOp::Simplify {
        xf.flatten_path_to_grid(clip, 1, tol, &mut grid_segs);
    }
    if grid_segs.is_empty() {
        return Path::empty();
    }

    let snapped = hobby_snap_round(&grid_segs);
    let arrangement = build_arrangement(&snapped);

    let mut boundary: Vec<(GridPoint, GridPoint)> = Vec::new();
    for e in &arrangement {
        let left_wa = e.right_wa.wrapping_add(e.delta_a);
        let left_wb = e.right_wb.wrapping_add(e.delta_b);
        let in_left = eval_boolean_op(
            op,
            is_winding_inside(left_wa, subject_rule),
            is_winding_inside(left_wb, clip_rule),
        );
        let in_right = eval_boolean_op(
            op,
            is_winding_inside(e.right_wa, subject_rule),
            is_winding_inside(e.right_wb, clip_rule),
        );
        if in_left && !in_right {
            boundary.push((e.lo, e.hi));
        } else if !in_left && in_right {
            boundary.push((e.hi, e.lo));
        }
    }

    extract_contours(&boundary, &xf)
}

#[inline]
pub fn simplify_path(path: &Path, fill_rule: FillRule, tolerance: f32) -> Path {
    boolean_op(
        path,
        &Path::empty(),
        BooleanOp::Simplify,
        fill_rule,
        FillRule::NonZero,
        tolerance,
    )
}

// Inflates (delta > 0) or deflates (delta < 0) a closed path cleanly via the snap-rounded arrangement.
pub fn offset_path(
    path: &Path,
    delta: f32,
    join: LineJoin,
    miter_limit: f32,
    tolerance: f32,
) -> Path {
    if path.is_empty() || !delta.is_finite() {
        return Path::empty();
    }
    let tol = if tolerance.is_finite() && tolerance > 0.0 {
        tolerance.max(1e-4)
    } else {
        0.25
    };
    let base = simplify_path(path, FillRule::NonZero, tol);
    if base.is_empty() || delta == 0.0 {
        return base;
    }

    let width = 2.0 * (abs_f64(delta as f64) as f32);
    let style = StrokeStyle::new(width)
        .with_cap(LineCap::Round)
        .with_join(join)
        .with_miter_limit(miter_limit);
    let band = expand_stroke_path(&base, Affine2D::IDENTITY, &style, tol);

    if delta > 0.0 {
        boolean_op(
            &base,
            &band,
            BooleanOp::Union,
            FillRule::NonZero,
            FillRule::NonZero,
            tol,
        )
    } else {
        boolean_op(
            &base,
            &band,
            BooleanOp::Difference,
            FillRule::NonZero,
            FillRule::NonZero,
            tol,
        )
    }
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

    fn polygon_signed_area(path: &Path) -> f64 {
        let mut area2 = 0.0f64;
        flatten_path_segments(path, Affine2D::IDENTITY, 0.1, true, &mut |a, b| {
            area2 += (a.x as f64) * (b.y as f64) - (b.x as f64) * (a.y as f64);
        });
        0.5 * area2
    }

    fn make_random_polygon(seed: &mut u64) -> Path {
        let mut next_u64 = || -> u64 {
            *seed = seed
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            *seed
        };
        let n = 3 + (next_u64() % 6) as usize;
        let mut pb = PathBuilder::new();
        for i in 0..n {
            let x = (((next_u64() >> 40) as f32) / ((1u32 << 24) as f32)) * 100.0;
            let y = (((next_u64() >> 40) as f32) / ((1u32 << 24) as f32)) * 100.0;
            if i == 0 {
                pb.move_to(x, y);
            } else {
                pb.line_to(x, y);
            }
        }
        pb.close();
        pb.finish()
    }

    #[test]
    fn algebraic_set_theory_laws_across_1000_randomized_polygon_pairs() {
        let mut seed = 0xCAFE_BABE_1234_5678u64;
        for case in 0..1000 {
            let a = make_random_polygon(&mut seed);
            let b = make_random_polygon(&mut seed);
            let rule = if (case & 1) == 0 {
                FillRule::NonZero
            } else {
                FillRule::EvenOdd
            };

            let sa = simplify_path(&a, rule, 0.25);
            let sb = simplify_path(&b, rule, 0.25);
            let area_a = polygon_signed_area(&sa);
            let area_b = polygon_signed_area(&sb);
            assert!(area_a >= -1e-2 && area_b >= -1e-2);

            let u = boolean_op(&a, &b, BooleanOp::Union, rule, rule, 0.25);
            let inter = boolean_op(&a, &b, BooleanOp::Intersection, rule, rule, 0.25);
            let area_u = polygon_signed_area(&u);
            let area_i = polygon_signed_area(&inter);

            // Law 1: Area(A U B) + Area(A n B) == Area(A) + Area(B)
            let lhs = area_u + area_i;
            let rhs = area_a + area_b;
            let tol = 1e-2 * (rhs.abs() + 1.0);
            assert!(
                abs_f64(lhs - rhs) <= tol,
                "case {case} inclusion-exclusion failed: U={area_u} I={area_i} A={area_a} B={area_b}"
            );

            // Law 2 & 3 on every 10th pair (keeps debug test time crisp while still checking 100 full sets!)
            if case % 10 == 0 {
                let d_ab = boolean_op(&a, &b, BooleanOp::Difference, rule, rule, 0.25);
                let d_ba = boolean_op(&b, &a, BooleanOp::Difference, rule, rule, 0.25);
                let u_diff = boolean_op(
                    &d_ab,
                    &d_ba,
                    BooleanOp::Union,
                    FillRule::NonZero,
                    FillRule::NonZero,
                    0.25,
                );
                let xor = boolean_op(&a, &b, BooleanOp::Xor, rule, rule, 0.25);
                let area_ud = polygon_signed_area(&u_diff);
                let area_xor = polygon_signed_area(&xor);
                assert!(
                    abs_f64(area_ud - area_xor) <= tol,
                    "case {case} xor law failed: (A\\B)U(B\\A)={area_ud}, xor={area_xor}"
                );

                // Law 3: Simplify(Simplify(A)) has identical area
                let ssa = simplify_path(&sa, FillRule::NonZero, 0.25);
                assert!(
                    abs_f64(polygon_signed_area(&ssa) - area_a) <= tol,
                    "case {case} simplify idempotence failed"
                );
            }
        }
    }

    #[test]
    fn offset_path_inflate_deflate_round_join_and_pinch_split() {
        use crate::math::affine::Rect;

        // 1. Inflate & deflate 40x60 rectangle by +/-10 with Miter join
        let mut rpb = PathBuilder::new();
        rpb.push_rect(Rect::from_xywh(100.0, 100.0, 40.0, 60.0));
        let rect = rpb.finish();

        let inflated = offset_path(&rect, 10.0, LineJoin::Miter, 4.0, 0.1);
        assert!(
            abs_f64(polygon_signed_area(&inflated) - (60.0 * 80.0)) < 0.5,
            "inflated area={}",
            polygon_signed_area(&inflated)
        );

        let deflated = offset_path(&rect, -10.0, LineJoin::Miter, 4.0, 0.1);
        assert!(
            abs_f64(polygon_signed_area(&deflated) - (20.0 * 40.0)) < 0.5,
            "deflated area={}",
            polygon_signed_area(&deflated)
        );

        // Over-deflate by -25 (> half width 20) -> completely vanishes to empty path
        let vanished = offset_path(&rect, -25.0, LineJoin::Miter, 4.0, 0.1);
        assert!(vanished.is_empty());

        // 2. Round-join inflate area: 40*60 + 2*10*(40+60) + pi*10^2 = 2400 + 2000 + 314.159 = 4714.159
        let rounded = offset_path(&rect, 10.0, LineJoin::Round, 4.0, 0.02);
        let expected_round = 2400.0 + 2000.0 + core::f64::consts::PI * 100.0;
        assert!(
            abs_f64(polygon_signed_area(&rounded) - expected_round) < 1.5,
            "rounded area={}, expected={expected_round}",
            polygon_signed_area(&rounded)
        );

        // 3. Dumbbell pinch-off: two 40x40 squares connected by a 10px bridge, deflated by -7px
        // Bridge vanishes and shape splits cleanly into two disjoint 26x26 squares (total area = 1352)
        let mut dpb = PathBuilder::new();
        dpb.push_rect(Rect::from_xywh(0.0, 0.0, 40.0, 40.0));
        dpb.push_rect(Rect::from_xywh(40.0, 15.0, 20.0, 10.0));
        dpb.push_rect(Rect::from_xywh(60.0, 0.0, 40.0, 40.0));
        let dumbbell = dpb.finish();
        let split = offset_path(&dumbbell, -7.0, LineJoin::Miter, 4.0, 0.1);
        let move_count = split
            .verbs()
            .iter()
            .filter(|&&v| v == crate::geom::PathVerb::MoveTo)
            .count();
        assert_eq!(move_count, 2, "expected dumbbell to split into 2 disjoint contours");
        assert!(
            abs_f64(polygon_signed_area(&split) - (2.0 * 26.0 * 26.0)) < 1.0,
            "split area={}",
            polygon_signed_area(&split)
        );
    }

    #[test]
    fn extreme_coordinate_fuzz_seeds_never_panic() {
        // NaN, Inf, subnormals, 1e30, micro-slivers 1e-7 -> must never panic
        let mut p_nan = PathBuilder::new();
        p_nan.move_to(f32::NAN, 0.0).line_to(10.0, f32::INFINITY).close();
        let p_nan = p_nan.finish();
        assert!(simplify_path(&p_nan, FillRule::NonZero, 0.25).is_empty());

        let mut p_huge = PathBuilder::new();
        p_huge.move_to(-1e30, -1e30).line_to(1e30, -1e30).line_to(1e30, 1e30).line_to(-1e30, 1e30).close();
        let p_huge = p_huge.finish();
        let sh = simplify_path(&p_huge, FillRule::NonZero, 0.25);
        assert!(!sh.is_empty());

        let mut p_sliver = PathBuilder::new();
        p_sliver.move_to(0.0, 0.0).line_to(100.0, 0.0).line_to(100.0, 1e-7).line_to(0.0, 1e-7).close();
        let p_sliver = p_sliver.finish();
        let _ = boolean_op(
            &p_sliver,
            &p_sliver,
            BooleanOp::Xor,
            FillRule::NonZero,
            FillRule::NonZero,
            0.25,
        );

        let mut p_sub = PathBuilder::new();
        let sub = f32::MIN_POSITIVE * 0.5;
        p_sub.move_to(0.0, 0.0).line_to(sub, 0.0).line_to(0.0, sub).close();
        let p_sub = p_sub.finish();
        let _ = simplify_path(&p_sub, FillRule::NonZero, 0.25);
    }
}
