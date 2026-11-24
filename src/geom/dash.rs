#![forbid(unsafe_code)]

use alloc::vec::Vec;

use crate::geom::path::{Path, PathBuilder, PathEvent};
use crate::geom::stroke::{CubicSeg, QuadSeg};
use crate::math::affine::Point;
use crate::math::float::{abs_f64, sqrt_f64};

const MAX_ARC_DEPTH: u32 = 10;
const ARC_EPS: f64 = 1e-4;

// 5-point Gauss-Legendre nodes and weights mapped to [0, 1] (exact up to degree 9).
const GL5_T: [f64; 5] = [
    0.046_910_077_030_668_004,
    0.230_765_344_947_158_45,
    0.5,
    0.769_234_655_052_841_5,
    0.953_089_922_969_332,
];

const GL5_W: [f64; 5] = [
    0.118_463_442_528_094_54,
    0.239_314_335_249_683_24,
    0.284_444_444_444_444_44,
    0.239_314_335_249_683_24,
    0.118_463_442_528_094_54,
];

#[inline]
fn line_length(a: Point, b: Point) -> f64 {
    let dx = b.x as f64 - a.x as f64;
    let dy = b.y as f64 - a.y as f64;
    sqrt_f64(dx * dx + dy * dy)
}

#[inline]
fn quad_speed(q: QuadSeg, t: f64) -> f64 {
    let mt = 1.0 - t;
    let d0x = q.p1.x as f64 - q.p0.x as f64;
    let d0y = q.p1.y as f64 - q.p0.y as f64;
    let d1x = q.p2.x as f64 - q.p1.x as f64;
    let d1y = q.p2.y as f64 - q.p1.y as f64;
    let dx = 2.0 * (mt * d0x + t * d1x);
    let dy = 2.0 * (mt * d0y + t * d1y);
    sqrt_f64(dx * dx + dy * dy)
}

#[inline]
fn cubic_speed(c: CubicSeg, t: f64) -> f64 {
    let mt = 1.0 - t;
    let v0x = c.p1.x as f64 - c.p0.x as f64;
    let v0y = c.p1.y as f64 - c.p0.y as f64;
    let v1x = c.p2.x as f64 - c.p1.x as f64;
    let v1y = c.p2.y as f64 - c.p1.y as f64;
    let v2x = c.p3.x as f64 - c.p2.x as f64;
    let v2y = c.p3.y as f64 - c.p2.y as f64;
    let dx = 3.0 * (mt * mt * v0x + 2.0 * mt * t * v1x + t * t * v2x);
    let dy = 3.0 * (mt * mt * v0y + 2.0 * mt * t * v1y + t * t * v2y);
    sqrt_f64(dx * dx + dy * dy)
}

#[inline]
fn gl5_integrate<F: Fn(f64) -> f64>(speed: &F, t0: f64, t1: f64) -> f64 {
    let dt = t1 - t0;
    let mut sum = 0.0;
    for i in 0..5 {
        sum += GL5_W[i] * speed(t0 + dt * GL5_T[i]);
    }
    sum * dt
}

fn arc_length_adaptive<F: Fn(f64) -> f64>(
    speed: &F,
    t0: f64,
    t1: f64,
    whole: f64,
    eps: f64,
    depth: u32,
) -> f64 {
    let tm = 0.5 * (t0 + t1);
    let left = gl5_integrate(speed, t0, tm);
    let right = gl5_integrate(speed, tm, t1);
    let halves = left + right;
    let tol = eps * (1.0 + halves);
    if depth >= MAX_ARC_DEPTH || abs_f64(halves - whole) <= tol {
        return halves;
    }
    arc_length_adaptive(speed, t0, tm, left, eps * 0.5, depth + 1)
        + arc_length_adaptive(speed, tm, t1, right, eps * 0.5, depth + 1)
}

#[inline]
fn arc_length_interval<F: Fn(f64) -> f64>(speed: &F, t0: f64, t1: f64) -> f64 {
    if t1 <= t0 {
        return 0.0;
    }
    let whole = gl5_integrate(speed, t0, t1);
    arc_length_adaptive(speed, t0, t1, whole, ARC_EPS, 0)
}

pub fn quad_arc_length(q: QuadSeg) -> f64 {
    let speed = |t: f64| quad_speed(q, t);
    arc_length_interval(&speed, 0.0, 1.0)
}

pub fn cubic_arc_length(c: CubicSeg) -> f64 {
    let speed = |t: f64| cubic_speed(c, t);
    arc_length_interval(&speed, 0.0, 1.0)
}

// Monotonic bracket + safeguarded Newton-Raphson inversion of s(t) = target_s.
fn invert_arc_length<F: Fn(f64) -> f64>(speed: &F, target_s: f64, total_s: f64) -> f64 {
    if target_s <= 0.0 || total_s <= 0.0 {
        return 0.0;
    }
    if target_s >= total_s {
        return 1.0;
    }

    let mut lo = 0.0f64;
    let mut hi = 1.0f64;
    let mut s_lo = 0.0f64;
    let mut t = (target_s / total_s).clamp(1e-6, 1.0 - 1e-6);
    let tol = 1e-5 * total_s.max(1.0);

    for _ in 0..16 {
        let s_t = s_lo + arc_length_interval(speed, lo, t);
        let err = s_t - target_s;
        if abs_f64(err) <= tol || hi - lo <= 1e-9 {
            return t;
        }
        if err < 0.0 {
            lo = t;
            s_lo = s_t;
        } else {
            hi = t;
        }
        let spd = speed(t);
        let next_t = if spd > 1e-12 {
            let cand = t - err / spd;
            if cand > lo && cand < hi {
                cand
            } else {
                0.5 * (lo + hi)
            }
        } else {
            0.5 * (lo + hi)
        };
        t = next_t;
    }
    t
}

pub fn quad_t_at_arc_length(q: QuadSeg, target_s: f64) -> f64 {
    let total_s = quad_arc_length(q);
    let speed = |t: f64| quad_speed(q, t);
    invert_arc_length(&speed, target_s, total_s)
}

pub fn cubic_t_at_arc_length(c: CubicSeg, target_s: f64) -> f64 {
    let total_s = cubic_arc_length(c);
    let speed = |t: f64| cubic_speed(c, t);
    invert_arc_length(&speed, target_s, total_s)
}

#[derive(Debug, Clone, Copy)]
enum DashSeg {
    Line(Point, Point),
    Quad(QuadSeg),
    Cubic(CubicSeg),
}

impl DashSeg {
    #[inline]
    fn start(&self) -> Point {
        match *self {
            Self::Line(a, _) => a,
            Self::Quad(q) => q.p0,
            Self::Cubic(c) => c.p0,
        }
    }

    #[inline]
    fn end(&self) -> Point {
        match *self {
            Self::Line(_, b) => b,
            Self::Quad(q) => q.p2,
            Self::Cubic(c) => c.p3,
        }
    }

    #[inline]
    fn length(&self) -> f64 {
        match *self {
            Self::Line(a, b) => line_length(a, b),
            Self::Quad(q) => quad_arc_length(q),
            Self::Cubic(c) => cubic_arc_length(c),
        }
    }

    fn split_at_arc_length(&self, s: f64, total_s: f64) -> (Self, Self) {
        match *self {
            Self::Line(a, b) => {
                let t = if total_s > 0.0 {
                    (s / total_s).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let mid = Point::new(
                    (a.x as f64 + (b.x as f64 - a.x as f64) * t) as f32,
                    (a.y as f64 + (b.y as f64 - a.y as f64) * t) as f32,
                );
                (Self::Line(a, mid), Self::Line(mid, b))
            }
            Self::Quad(q) => {
                let speed = |t: f64| quad_speed(q, t);
                let t = invert_arc_length(&speed, s, total_s);
                let (l, r) = q.split_at(t);
                (Self::Quad(l), Self::Quad(r))
            }
            Self::Cubic(c) => {
                let speed = |t: f64| cubic_speed(c, t);
                let t = invert_arc_length(&speed, s, total_s);
                let (l, r) = c.split_at(t);
                (Self::Cubic(l), Self::Cubic(r))
            }
        }
    }

    #[inline]
    fn emit_into(&self, pb: &mut PathBuilder) {
        match *self {
            Self::Line(_, b) => {
                pb.line_to(b.x, b.y);
            }
            Self::Quad(q) => {
                pb.quad_to(q.p1.x, q.p1.y, q.p2.x, q.p2.y);
            }
            Self::Cubic(c) => {
                pb.cubic_to(c.p1.x, c.p1.y, c.p2.x, c.p2.y, c.p3.x, c.p3.y);
            }
        }
    }
}

struct DashPattern {
    lengths: Vec<f64>,
    start_idx: usize,
    start_rem: f64,
}

impl DashPattern {
    // Validates, duplicates odd-length arrays, wraps negative offsets, and returns None if disabled.
    fn new(dash_array: &[f32], dash_offset: f32) -> Option<Self> {
        if dash_array.is_empty() || !dash_offset.is_finite() {
            return None;
        }
        for &v in dash_array {
            if !v.is_finite() || v < 0.0 {
                return None;
            }
        }

        let dup = (dash_array.len() & 1) == 1;
        let mut lengths = Vec::with_capacity(if dup {
            dash_array.len() * 2
        } else {
            dash_array.len()
        });
        for &v in dash_array {
            lengths.push(v as f64);
        }
        if dup {
            for &v in dash_array {
                lengths.push(v as f64);
            }
        }

        let period: f64 = lengths.iter().sum();
        if !period.is_finite() || period <= 1e-9 {
            return None;
        }

        // Wrap negative and oversized offsets into [0, period)
        let mut offset = (dash_offset as f64) % period;
        if offset < 0.0 {
            offset += period;
        }
        if offset >= period {
            offset = 0.0;
        }

        let mut start_idx = 0usize;
        // Strict `offset > 0.0` preserves zero-length leading "on" dash [0.0, 10.0] at offset == 0
        while offset > 0.0 && offset >= lengths[start_idx] {
            offset -= lengths[start_idx];
            start_idx = (start_idx + 1) % lengths.len();
        }
        let start_rem = lengths[start_idx] - offset;

        Some(Self {
            lengths,
            start_idx,
            start_rem,
        })
    }
}

fn emit_piece(segs: &[DashSeg], closed: bool, out: &mut PathBuilder) {
    let Some(first) = segs.first() else {
        return;
    };
    let s = first.start();
    out.move_to(s.x, s.y);
    for seg in segs {
        seg.emit_into(out);
    }
    if closed {
        out.close();
    }
}

fn dash_subpath(
    segs: &[DashSeg],
    origin: Point,
    closed: bool,
    pattern: &DashPattern,
    out: &mut PathBuilder,
) {
    if segs.is_empty() {
        // Zero-length subpath: emit dot if dash pattern starts "on"
        if (pattern.start_idx & 1) == 0 {
            out.move_to(origin.x, origin.y).line_to(origin.x, origin.y);
        }
        return;
    }

    let mut idx = pattern.start_idx;
    let mut rem = pattern.start_rem;
    let pat_len = pattern.lengths.len();

    // Track pieces for closed-path seam merging: if both start (s=0) and end (s=perimeter)
    // are active "on" dashes, merge last_piece + first_piece seamlessly.
    let mut started_on = false;
    let mut started_zero_dot = false;
    let mut first_piece: Vec<DashSeg> = Vec::new();
    let mut has_first_piece = false;
    let mut current_piece: Vec<DashSeg> = Vec::new();
    let mut saw_off_gap = false;

    let advance_zero_dashes = |at: Point,
                               is_end_of_closed: bool,
                               idx: &mut usize,
                               rem: &mut f64,
                               saw_off: &mut bool,
                               started_zero: &mut bool,
                               at_start: bool,
                               out: &mut PathBuilder| {
        let mut guard = 0usize;
        while *rem <= 1e-9 && guard < pat_len {
            let is_on = (*idx & 1) == 0;
            if is_on {
                // Skip duplicate dot right on the closing seam if one was emitted at s = 0
                if !(is_end_of_closed && *started_zero) {
                    out.move_to(at.x, at.y).line_to(at.x, at.y);
                    if at_start {
                        *started_zero = true;
                    }
                }
            } else {
                *saw_off = true;
            }
            *idx = (*idx + 1) % pat_len;
            *rem = pattern.lengths[*idx];
            guard += 1;
        }
    };

    if (idx & 1) == 0 {
        if rem > 1e-9 {
            started_on = true;
        } else {
            advance_zero_dashes(
                segs[0].start(),
                false,
                &mut idx,
                &mut rem,
                &mut saw_off_gap,
                &mut started_zero_dot,
                true,
                out,
            );
            if (idx & 1) == 0 && rem > 1e-9 {
                started_on = true;
            }
        }
    } else {
        saw_off_gap = true;
    }

    let flush_current = |cur: &mut Vec<DashSeg>,
                         first: &mut Vec<DashSeg>,
                         has_first: &mut bool,
                         out: &mut PathBuilder| {
        if cur.is_empty() {
            return;
        }
        if closed && started_on && !*has_first {
            *has_first = true;
            first.extend_from_slice(cur);
        } else {
            emit_piece(cur, false, out);
        }
        cur.clear();
    };

    for (seg_i, &orig_seg) in segs.iter().enumerate() {
        let mut cur_seg = orig_seg;
        let mut seg_rem = cur_seg.length();
        if seg_rem <= 1e-9 {
            continue;
        }

        let is_last_seg = seg_i + 1 == segs.len();

        while seg_rem > 1e-9 {
            if rem <= 1e-9 {
                advance_zero_dashes(
                    cur_seg.start(),
                    false,
                    &mut idx,
                    &mut rem,
                    &mut saw_off_gap,
                    &mut started_zero_dot,
                    false,
                    out,
                );
            }

            let is_on = (idx & 1) == 0;
            if seg_rem <= rem + 1e-9 {
                if is_on {
                    current_piece.push(cur_seg);
                } else {
                    saw_off_gap = true;
                }
                rem -= seg_rem;
                seg_rem = 0.0;

                if rem <= 1e-9 {
                    let end_pt = cur_seg.end();
                    if is_on {
                        flush_current(&mut current_piece, &mut first_piece, &mut has_first_piece, out);
                    }
                    idx = (idx + 1) % pat_len;
                    rem = pattern.lengths[idx];
                    if !is_last_seg || !closed {
                        advance_zero_dashes(
                            end_pt,
                            is_last_seg && closed,
                            &mut idx,
                            &mut rem,
                            &mut saw_off_gap,
                            &mut started_zero_dot,
                            false,
                            out,
                        );
                    } else if rem <= 1e-9 {
                        advance_zero_dashes(
                            end_pt,
                            true,
                            &mut idx,
                            &mut rem,
                            &mut saw_off_gap,
                            &mut started_zero_dot,
                            false,
                            out,
                        );
                    }
                }
            } else {
                let (head, tail) = cur_seg.split_at_arc_length(rem, seg_rem);
                if is_on {
                    current_piece.push(head);
                    flush_current(&mut current_piece, &mut first_piece, &mut has_first_piece, out);
                } else {
                    saw_off_gap = true;
                }
                seg_rem -= rem;
                cur_seg = tail;

                idx = (idx + 1) % pat_len;
                rem = pattern.lengths[idx];
                advance_zero_dashes(
                    cur_seg.start(),
                    false,
                    &mut idx,
                    &mut rem,
                    &mut saw_off_gap,
                    &mut started_zero_dot,
                    false,
                    out,
                );
            }
        }
    }

    // Finalize remaining open/closed dash pieces
    if closed && started_on {
        if !saw_off_gap && !has_first_piece && !current_piece.is_empty() {
            // Entire closed subpath was covered by a single uninterrupted "on" dash
            emit_piece(&current_piece, true, out);
        } else if has_first_piece && !current_piece.is_empty() {
            // Both start and end of closed subpath are active "on" dashes: merge seamlessly!
            current_piece.extend_from_slice(&first_piece);
            emit_piece(&current_piece, false, out);
        } else {
            if has_first_piece {
                emit_piece(&first_piece, false, out);
            }
            if !current_piece.is_empty() {
                emit_piece(&current_piece, false, out);
            }
        }
    } else if !current_piece.is_empty() {
        emit_piece(&current_piece, false, out);
    }
}

// Walks `path` by exact Gauss-Legendre arc length, emitting dashed subpaths into `out`.
// Returns `false` if dashing is disabled (empty/invalid/all-zero pattern), leaving `out` untouched.
pub fn dash_path_into(
    path: &Path,
    dash_array: &[f32],
    dash_offset: f32,
    out: &mut PathBuilder,
) -> bool {
    let Some(pattern) = DashPattern::new(dash_array, dash_offset) else {
        return false;
    };
    out.clear();
    if path.is_empty() {
        return true;
    }

    let mut segs: Vec<DashSeg> = Vec::new();
    let mut cur = Point::ZERO;
    let mut sub_start = Point::ZERO;
    let mut in_subpath = false;
    let mut has_verb = false;

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if in_subpath && (has_verb || !segs.is_empty()) {
                    dash_subpath(&segs, sub_start, false, &pattern, out);
                }
                segs.clear();
                cur = p;
                sub_start = p;
                in_subpath = true;
                has_verb = false;
            }
            PathEvent::LineTo(p) => {
                if in_subpath {
                    has_verb = true;
                    if line_length(cur, p) > 1e-9 {
                        segs.push(DashSeg::Line(cur, p));
                    }
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::QuadTo(c, p) => {
                if in_subpath {
                    has_verb = true;
                    let q = QuadSeg::new(cur, c, p);
                    if !q.is_degenerate() {
                        segs.push(DashSeg::Quad(q));
                    }
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                if in_subpath {
                    has_verb = true;
                    let c = CubicSeg::new(cur, c1, c2, p);
                    if !c.is_degenerate() {
                        segs.push(DashSeg::Cubic(c));
                    }
                } else {
                    sub_start = p;
                    in_subpath = true;
                }
                cur = p;
            }
            PathEvent::Close => {
                if in_subpath {
                    if line_length(cur, sub_start) > 1e-9 {
                        segs.push(DashSeg::Line(cur, sub_start));
                    }
                    dash_subpath(&segs, sub_start, true, &pattern, out);
                    segs.clear();
                    cur = sub_start;
                    in_subpath = false;
                    has_verb = false;
                }
            }
        }
    }

    if in_subpath && (has_verb || !segs.is_empty()) {
        dash_subpath(&segs, sub_start, false, &pattern, out);
    }

    true
}

pub fn dash_path(path: &Path, dash_array: &[f32], dash_offset: f32) -> Path {
    let mut pb = PathBuilder::new();
    if dash_path_into(path, dash_array, dash_offset, &mut pb) {
        pb.finish()
    } else {
        path.clone()
    }
}

// Measures the exact arc length of every subpath in `path`.
pub fn subpath_arc_lengths(path: &Path) -> Vec<f64> {
    let mut out = Vec::new();
    let mut cur = Point::ZERO;
    let mut start = Point::ZERO;
    let mut in_sub = false;
    let mut len = 0.0f64;

    for ev in path.iter() {
        match ev {
            PathEvent::MoveTo(p) => {
                if in_sub {
                    out.push(len);
                }
                cur = p;
                start = p;
                in_sub = true;
                len = 0.0;
            }
            PathEvent::LineTo(p) => {
                if in_sub {
                    len += line_length(cur, p);
                } else {
                    start = p;
                    in_sub = true;
                }
                cur = p;
            }
            PathEvent::QuadTo(c, p) => {
                if in_sub {
                    len += quad_arc_length(QuadSeg::new(cur, c, p));
                } else {
                    start = p;
                    in_sub = true;
                }
                cur = p;
            }
            PathEvent::CubicTo(c1, c2, p) => {
                if in_sub {
                    len += cubic_arc_length(CubicSeg::new(cur, c1, c2, p));
                } else {
                    start = p;
                    in_sub = true;
                }
                cur = p;
            }
            PathEvent::Close => {
                if in_sub {
                    len += line_length(cur, start);
                    out.push(len);
                    cur = start;
                    in_sub = false;
                    len = 0.0;
                }
            }
        }
    }
    if in_sub {
        out.push(len);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::stroke::{LineCap, StrokeStyle, expand_stroke_path};
    use crate::math::affine::Affine2D;

    #[test]
    fn eccentric_ellipse_dash_uniformity_within_0_1_percent() {
        // Highly eccentric ellipse rx = 400, ry = 40: Ramanujan perimeter ~= 1621.84 px
        let mut pb = PathBuilder::new();
        pb.push_ellipse(500.0, 200.0, 400.0, 40.0);
        let ellipse = pb.finish();

        // Dash length = 20.0, gap = 10.0 -> ~54 dashes around the ellipse
        let dashed = dash_path(&ellipse, &[20.0, 10.0], 0.0);
        let lens = subpath_arc_lengths(&dashed);
        assert!(lens.len() >= 50, "expected >= 50 dashes, got {}", lens.len());

        // Every full dash (except possibly the last merged/remainder seam dash) must match 20.0 within < 0.1%
        for (i, &l) in lens.iter().enumerate() {
            if i + 1 == lens.len() && l != 20.0 {
                // Closing seam dash may be merged with start or remainder depending on perimeter mod 30
                continue;
            }
            let rel_err = abs_f64(l - 20.0) / 20.0;
            assert!(
                rel_err < 0.001,
                "dash {i} arc length {l} deviated from 20.0 by {:.4}%",
                rel_err * 100.0
            );
        }

        // Also verify dashed ellipse preserves smooth curves (has_curves == true)
        assert!(dashed.has_curves());
    }

    #[test]
    fn odd_array_negative_offset_and_disabled_dash_edge_cases() {
        // Straight line of length 100
        let mut pb = PathBuilder::new();
        pb.move_to(0.0, 0.0).line_to(100.0, 0.0);
        let line = pb.finish();

        // Odd array [5, 2, 1] -> [5, 2, 1, 5, 2, 1] (on=5, off=2, on=1, off=5, on=2, off=1; period=16)
        let odd = dash_path(&line, &[5.0, 2.0, 1.0], 0.0);
        let odd_lens = subpath_arc_lengths(&odd);
        // First cycle: 5, 1, 2; second cycle: 5, 1, 2 ...
        assert!(odd_lens.len() >= 6);
        assert!(abs_f64(odd_lens[0] - 5.0) < 1e-4);
        assert!(abs_f64(odd_lens[1] - 1.0) < 1e-4);
        assert!(abs_f64(odd_lens[2] - 2.0) < 1e-4);
        assert!(abs_f64(odd_lens[3] - 5.0) < 1e-4);
        assert!(abs_f64(odd_lens[4] - 1.0) < 1e-4);
        assert!(abs_f64(odd_lens[5] - 2.0) < 1e-4);

        // Negative offset: [10, 10] with offset = -4.0 == offset = +16.0 (4 units of gap left, then full 10-unit dash)
        let neg = dash_path(&line, &[10.0, 10.0], -4.0);
        let pos = dash_path(&line, &[10.0, 10.0], 16.0);
        assert_eq!(neg, pos);
        let neg_lens = subpath_arc_lengths(&neg);
        assert!(abs_f64(neg_lens[0] - 10.0) < 1e-4);
        // First dash starts at x = 4.0
        assert!(abs_f64(neg.points()[0].x as f64 - 4.0) < 1e-4);

        // Empty, all-zero, or negative dash entry disables dashing safely
        assert_eq!(dash_path(&line, &[], 0.0), line);
        assert_eq!(dash_path(&line, &[0.0, 0.0], 0.0), line);
        assert_eq!(dash_path(&line, &[5.0, -2.0], 0.0), line);
        assert_eq!(dash_path(&line, &[f32::NAN, 5.0], 0.0), line);
    }

    #[test]
    fn dotted_lines_zero_length_dashes_with_round_caps() {
        // Line of length 60 with [0.0, 12.0] -> exact dots at x = 0, 12, 24, 36, 48, 60 (6 dots!)
        let mut pb = PathBuilder::new();
        pb.move_to(0.0, 20.0).line_to(60.0, 20.0);
        let line = pb.finish();

        let dotted = dash_path(&line, &[0.0, 12.0], 0.0);
        let lens = subpath_arc_lengths(&dotted);
        assert_eq!(lens.len(), 6, "expected 6 dots, got {lens:?}");
        for &l in &lens {
            assert!(l < 1e-6);
        }

        // Stroking with Round cap must produce 6 filled circles centered at x = 0, 12, 24, 36, 48, 60
        let style = StrokeStyle::new(6.0)
            .with_cap(LineCap::Round)
            .with_dashes(&[0.0, 12.0], 0.0);
        let stroked = expand_stroke_path(&line, Affine2D::IDENTITY, &style, 0.1);
        for k in 0..6 {
            let x = (k as f32) * 12.0;
            assert!(
                crate::geom::query::point_in_fill(
                    &stroked,
                    Point::new(x, 20.0),
                    crate::geom::FillRule::NonZero,
                    0.1
                ),
                "missing dot at x={x}"
            );
            if k < 5 {
                // Midpoint between dots must be empty
                assert!(!crate::geom::query::point_in_fill(
                    &stroked,
                    Point::new(x + 6.0, 20.0),
                    crate::geom::FillRule::NonZero,
                    0.1
                ));
            }
        }
    }

    #[test]
    fn closed_path_start_end_dash_seam_merging() {
        // Square perimeter = 4 * 25 = 100. Pattern = [15, 10] (period = 25), offset = 5:
        // s=0..10 is ON (first 10 units), then 10..20 OFF, 20..35 ON, 35..45 OFF,
        // 45..60 ON, 60..70 OFF, 70..85 ON, 85..95 OFF, 95..100 ON (last 5 units).
        // Last 5 units (95..100) + first 10 units (0..10) MUST merge into ONE single 15-unit dash!
        // Total = 4 dashes, EVERY dash length == 15.0!
        let mut pb = PathBuilder::new();
        pb.move_to(0.0, 0.0)
            .line_to(25.0, 0.0)
            .line_to(25.0, 25.0)
            .line_to(0.0, 25.0)
            .close();
        let sq = pb.finish();

        let dashed = dash_path(&sq, &[15.0, 10.0], 5.0);
        let lens = subpath_arc_lengths(&dashed);
        assert_eq!(lens.len(), 4, "expected 4 merged dashes, got {lens:?}");
        for (i, &l) in lens.iter().enumerate() {
            assert!(
                abs_f64(l - 15.0) < 1e-3,
                "closed merged dash {i} length = {l}, expected 15.0"
            );
        }
    }
}
