#![forbid(unsafe_code)]

use crate::geom::FillRule;
use crate::math::arena::{FrameArena, TILE_AREA, TILE_SIZE, TileCmd, TileSeg};
use crate::math::fixed::mul_div_i64;
use crate::tile::{TILE_FLAG_CRISP_EDGES, TILE_FLAG_EVEN_ODD, TileBinner};

pub const STRIP_HEIGHT: usize = 4;
pub const STRIPS_PER_TILE: usize = TILE_SIZE / STRIP_HEIGHT;
pub const DIFF_STRIDE: usize = TILE_SIZE + 1;
pub const DIFF_LEN: usize = TILE_SIZE * DIFF_STRIDE;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum AntialiasMode {
    #[default]
    Analytic,
    None,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoverageSpan<'a> {
    Solid { y: usize, x0: usize, x1: usize },
    Fractional { y: usize, x0: usize, cov: &'a [u16] },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TileFillParams {
    pub head_chunk: u32,
    pub backdrop: i32,
    pub fill_rule: FillRule,
    pub mode: AntialiasMode,
    pub tile_width: usize,
    pub tile_height: usize,
}

impl TileFillParams {
    #[inline]
    pub const fn from_cmd(cmd: TileCmd, tile_width: usize, tile_height: usize) -> Self {
        let fill_rule = if (cmd.flags & TILE_FLAG_EVEN_ODD) != 0 {
            FillRule::EvenOdd
        } else {
            FillRule::NonZero
        };
        let mode = if (cmd.flags & TILE_FLAG_CRISP_EDGES) != 0 {
            AntialiasMode::None
        } else {
            AntialiasMode::Analytic
        };
        Self {
            head_chunk: cmd.head_chunk,
            backdrop: cmd.backdrop,
            fill_rule,
            mode,
            tile_width,
            tile_height,
        }
    }
}

#[inline]
pub const fn acc_to_cov_u16(acc: i32, fill_rule: FillRule) -> u16 {
    let a = match fill_rule {
        FillRule::NonZero => acc.unsigned_abs(),
        FillRule::EvenOdd => {
            let m = acc.unsigned_abs() & 262_143;
            if m > 131_072 { 262_144 - m } else { m }
        }
    };
    if a >= 131_072 {
        65535
    } else {
        ((a - (a >> 16)) >> 1) as u16
    }
}

#[inline]
pub const fn cov_u16_to_u8(cov: u16) -> u8 {
    (((cov as u32) * 255 + 32767) / 65535) as u8
}

// L1-resident per-tile coverage scratchpad (4,480 B).
#[derive(Clone)]
pub struct TileRasterScratch {
    pub diff: [i32; DIFF_LEN],
    pub cov_strip: [[u16; TILE_SIZE]; STRIP_HEIGHT],
}

impl Default for TileRasterScratch {
    fn default() -> Self {
        Self::new()
    }
}

impl TileRasterScratch {
    pub const fn new() -> Self {
        Self {
            diff: [0; DIFF_LEN],
            cov_strip: [[0; TILE_SIZE]; STRIP_HEIGHT],
        }
    }

    #[inline]
    pub fn clear_diff(&mut self) {
        self.diff.fill(0);
    }

    // Single sequential pass over a tile's TileSegmentChunk list in analytic mode.
    pub fn accumulate_analytic(&mut self, head_chunk: u32, arena: &FrameArena) {
        self.diff.fill(0);
        for seg in TileBinner::iter_tile_segs(head_chunk, arena) {
            self.accumulate_seg_analytic(seg);
        }
    }

    // Single sequential pass over a tile's TileSegmentChunk list using pixel-center (y + 0.5) crossings.
    pub fn accumulate_crisp(&mut self, head_chunk: u32, arena: &FrameArena) {
        self.diff.fill(0);
        for seg in TileBinner::iter_tile_segs(head_chunk, arena) {
            self.accumulate_seg_crisp(seg);
        }
    }

    #[inline]
    pub fn accumulate_seg_analytic(&mut self, seg: TileSeg) {
        let x0 = seg.x0 as i32;
        let y0 = seg.y0 as i32;
        let x1 = seg.x1 as i32;
        let y1 = seg.y1 as i32;
        if y0 == y1 {
            return;
        }

        let (sx0, sy0, sx1, sy1, dir) = if y0 < y1 {
            (x0, y0, x1, y1, 1i32)
        } else {
            (x1, y1, x0, y0, -1i32)
        };

        let py_start = (sy0 >> 8).max(0) as usize;
        if py_start >= TILE_SIZE {
            return;
        }
        let py_end = (((sy1 - 1) >> 8) as usize).min(TILE_SIZE - 1);

        let dx = sx1 - sx0;
        let dy = sy1 - sy0;

        // Fast path: vertical segment (covers axis-aligned rects and Stokes' compensating edges on lx = 0)
        if dx == 0 {
            let px = (sx0 >> 8) as usize;
            if px >= TILE_SIZE {
                return;
            }
            let fx = sx0 & 255;
            if fx == 0 {
                for py in py_start..=py_end {
                    let row_y0 = (py as i32) << 8;
                    let y_top = sy0.max(row_y0);
                    let y_bot = sy1.min(row_y0 + 256);
                    let sdy = (y_bot - y_top) * dir;
                    let idx = py * DIFF_STRIDE + px;
                    self.diff[idx] = self.diff[idx].wrapping_add(sdy << 9);
                }
            } else {
                let w_area = 512 - (fx << 1);
                for py in py_start..=py_end {
                    let row_y0 = (py as i32) << 8;
                    let y_top = sy0.max(row_y0);
                    let y_bot = sy1.min(row_y0 + 256);
                    let sdy = (y_bot - y_top) * dir;
                    let area_2x = sdy * w_area;
                    let idx = py * DIFF_STRIDE + px;
                    self.diff[idx] = self.diff[idx].wrapping_add(area_2x);
                    self.diff[idx + 1] = self.diff[idx + 1].wrapping_add((sdy << 9) - area_2x);
                }
            }
            return;
        }

        for py in py_start..=py_end {
            let row_y0 = (py as i32) << 8;
            let row_y1 = row_y0 + 256;
            let y_top = sy0.max(row_y0);
            let y_bot = sy1.min(row_y1);

            let x_top = if y_top == sy0 {
                sx0
            } else {
                sx0 + mul_div_i64(dx, y_top - sy0, dy)
            };
            let x_bot = if y_bot == sy1 {
                sx1
            } else {
                sx0 + mul_div_i64(dx, y_bot - sy0, dy)
            };

            let row_off = py * DIFF_STRIDE;

            if x_top == x_bot {
                let px = (x_top >> 8) as usize;
                if px < TILE_SIZE {
                    let fx = x_top & 255;
                    let sdy = (y_bot - y_top) * dir;
                    let area_2x = sdy * (512 - (fx << 1));
                    let idx = row_off + px;
                    self.diff[idx] = self.diff[idx].wrapping_add(area_2x);
                    self.diff[idx + 1] = self.diff[idx + 1].wrapping_add((sdy << 9) - area_2x);
                }
                continue;
            }

            let (rx0, ry0, rx1, ry1) = if x_top < x_bot {
                (x_top, y_top, x_bot, y_bot)
            } else {
                (x_bot, y_bot, x_top, y_top)
            };

            let px_min = (rx0 >> 8).max(0) as usize;
            if px_min >= TILE_SIZE {
                continue;
            }
            let px_max = (((rx1 - 1) >> 8) as usize).min(TILE_SIZE - 1);

            if px_min == px_max {
                let col_x0 = (px_min as i32) << 8;
                let sdy = (y_bot - y_top) * dir;
                let fx0 = x_top - col_x0;
                let fx1 = x_bot - col_x0;
                let area_2x = sdy * (512 - (fx0 + fx1));
                let idx = row_off + px_min;
                self.diff[idx] = self.diff[idx].wrapping_add(area_2x);
                self.diff[idx + 1] = self.diff[idx + 1].wrapping_add((sdy << 9) - area_2x);
                continue;
            }

            let rdx = rx1 - rx0;
            let rdy = ry1 - ry0;
            let x_sign = if x_top < x_bot { dir } else { -dir };
            let mut prev_y = ry0;

            for px in px_min..=px_max {
                let col_x0 = (px as i32) << 8;
                let col_x1 = col_x0 + 256;
                let cx0 = rx0.max(col_x0);
                let cx1 = rx1.min(col_x1);
                let cy0 = prev_y;
                let cy1 = if cx1 == rx1 {
                    ry1
                } else {
                    ry0 + mul_div_i64(rdy, cx1 - rx0, rdx)
                };
                prev_y = cy1;

                let sdy = (cy1 - cy0) * x_sign;
                if sdy != 0 {
                    let fx0 = cx0 - col_x0;
                    let fx1 = cx1 - col_x0;
                    let area_2x = sdy * (512 - (fx0 + fx1));
                    let idx = row_off + px;
                    self.diff[idx] = self.diff[idx].wrapping_add(area_2x);
                    self.diff[idx + 1] = self.diff[idx + 1].wrapping_add((sdy << 9) - area_2x);
                }
            }
        }
    }

    #[inline]
    pub fn accumulate_seg_crisp(&mut self, seg: TileSeg) {
        let x0 = seg.x0 as i32;
        let y0 = seg.y0 as i32;
        let x1 = seg.x1 as i32;
        let y1 = seg.y1 as i32;
        if y0 == y1 {
            return;
        }

        let (sx0, sy0, sx1, sy1, dir) = if y0 < y1 {
            (x0, y0, x1, y1, 1i32)
        } else {
            (x1, y1, x0, y0, -1i32)
        };

        // Half-open scanline crossing at pixel center cy = py * 256 + 128: sy0 <= cy < sy1
        if sy1 <= 128 {
            return;
        }
        let py_start = ((sy0 + 127) >> 8).max(0) as usize;
        let py_end = (((sy1 - 129) >> 8) as usize).min(TILE_SIZE - 1);
        if py_start > py_end || py_start >= TILE_SIZE {
            return;
        }

        let dx = sx1 - sx0;
        let dy = sy1 - sy0;

        if dx == 0 {
            let px = ((sx0 + 127) >> 8).max(0) as usize;
            if px < TILE_SIZE {
                for py in py_start..=py_end {
                    let idx = py * DIFF_STRIDE + px;
                    self.diff[idx] = self.diff[idx].wrapping_add(dir);
                }
            }
            return;
        }

        for py in py_start..=py_end {
            let cy = ((py as i32) << 8) + 128;
            let x_cross = sx0 + mul_div_i64(dx, cy - sy0, dy);
            let px = ((x_cross + 127) >> 8).max(0) as usize;
            if px < TILE_SIZE {
                let idx = py * DIFF_STRIDE + px;
                self.diff[idx] = self.diff[idx].wrapping_add(dir);
            }
        }
    }

    // Resolves a 4-row strip (`strip_idx` in 0..8) into `self.cov_strip[0..rows][0..tile_width]`.
    pub fn resolve_strip(
        &mut self,
        strip_idx: usize,
        backdrop: i32,
        fill_rule: FillRule,
        mode: AntialiasMode,
        tile_width: usize,
        rows_in_strip: usize,
    ) {
        let base_py = strip_idx * STRIP_HEIGHT;
        let w = tile_width.min(TILE_SIZE);
        let rows = rows_in_strip.min(STRIP_HEIGHT);

        match mode {
            AntialiasMode::Analytic => {
                let init_acc = backdrop.wrapping_shl(17);
                for r in 0..rows {
                    let row_off = (base_py + r) * DIFF_STRIDE;
                    let cov_row = &mut self.cov_strip[r];
                    let mut acc = init_acc;
                    for (px, out) in cov_row[..w].iter_mut().enumerate() {
                        acc = acc.wrapping_add(self.diff[row_off + px]);
                        *out = acc_to_cov_u16(acc, fill_rule);
                    }
                }
            }
            AntialiasMode::None => {
                for r in 0..rows {
                    let row_off = (base_py + r) * DIFF_STRIDE;
                    let cov_row = &mut self.cov_strip[r];
                    let mut acc = backdrop;
                    for (px, out) in cov_row[..w].iter_mut().enumerate() {
                        acc = acc.wrapping_add(self.diff[row_off + px]);
                        let covered = match fill_rule {
                            FillRule::NonZero => acc != 0,
                            FillRule::EvenOdd => (acc & 1) != 0,
                        };
                        *out = if covered { 65535 } else { 0 };
                    }
                }
            }
        }
    }

    // Rasterizes a tile in 8 SIMD-aligned 4-row strips, dispatching contiguous solid and fractional spans.
    pub fn rasterize_tile_spans<F>(
        &mut self,
        params: TileFillParams,
        arena: &FrameArena,
        mut on_span: F,
    ) where
        F: FnMut(CoverageSpan<'_>),
    {
        let w = params.tile_width.min(TILE_SIZE);
        let h = params.tile_height.min(TILE_SIZE);
        if w == 0 || h == 0 {
            return;
        }

        match params.mode {
            AntialiasMode::Analytic => self.accumulate_analytic(params.head_chunk, arena),
            AntialiasMode::None => self.accumulate_crisp(params.head_chunk, arena),
        }

        let num_strips = h.div_ceil(STRIP_HEIGHT);
        for s in 0..num_strips {
            let base_py = s * STRIP_HEIGHT;
            let rows = (h - base_py).min(STRIP_HEIGHT);
            self.resolve_strip(s, params.backdrop, params.fill_rule, params.mode, w, rows);

            for r in 0..rows {
                let y = base_py + r;
                let cov = &self.cov_strip[r][..w];
                let mut x = 0usize;
                while x < w {
                    while x < w && cov[x] == 0 {
                        x += 1;
                    }
                    if x >= w {
                        break;
                    }
                    if cov[x] == 65535 {
                        let start = x;
                        x += 1;
                        while x < w && cov[x] == 65535 {
                            x += 1;
                        }
                        on_span(CoverageSpan::Solid { y, x0: start, x1: x });
                    } else {
                        let start = x;
                        x += 1;
                        while x < w && cov[x] != 0 && cov[x] != 65535 {
                            x += 1;
                        }
                        on_span(CoverageSpan::Fractional {
                            y,
                            x0: start,
                            cov: &cov[start..x],
                        });
                    }
                }
            }
        }
    }

    // Materializes a full 32x32 tile coverage mask (`u16` per pixel in row-major order).
    pub fn rasterize_tile_mask_u16(
        &mut self,
        params: TileFillParams,
        arena: &FrameArena,
        out: &mut [u16; TILE_AREA],
    ) {
        out.fill(0);
        let w = params.tile_width.min(TILE_SIZE);
        let h = params.tile_height.min(TILE_SIZE);
        if w == 0 || h == 0 {
            return;
        }

        match params.mode {
            AntialiasMode::Analytic => self.accumulate_analytic(params.head_chunk, arena),
            AntialiasMode::None => self.accumulate_crisp(params.head_chunk, arena),
        }

        let num_strips = h.div_ceil(STRIP_HEIGHT);
        for s in 0..num_strips {
            let base_py = s * STRIP_HEIGHT;
            let rows = (h - base_py).min(STRIP_HEIGHT);
            self.resolve_strip(s, params.backdrop, params.fill_rule, params.mode, w, rows);
            for r in 0..rows {
                let dst_off = (base_py + r) * TILE_SIZE;
                out[dst_off..dst_off + w].copy_from_slice(&self.cov_strip[r][..w]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use alloc::vec::Vec;
    use crate::geom::flatten::{DEFAULT_TOLERANCE, flatten_path_for_fill};
    use crate::geom::PathBuilder;
    use crate::math::affine::{Affine2D, Rect};
    use crate::math::arena::TileCmdKind;

    fn render_full_mask(
        path: &crate::geom::Path,
        w: u32,
        h: u32,
        fill_rule: FillRule,
        mode: AntialiasMode,
    ) -> (Vec<u16>, usize, usize) {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();
        let mut scratch = TileRasterScratch::new();
        binner.reset(w, h);

        let flags = if mode == AntialiasMode::None {
            TILE_FLAG_CRISP_EDGES
        } else {
            0
        };
        let range = flatten_path_for_fill(path, Affine2D::IDENTITY, w, h, DEFAULT_TOLERANCE, &mut arena);
        binner.bin_path(range, fill_rule, 0, flags, &mut arena);

        let w_us = w as usize;
        let h_us = h as usize;
        let mut out = vec![0u16; w_us * h_us];
        let mut solid_spans = 0usize;
        let mut cov_spans = 0usize;

        for ty in 0..binner.tiles_y() {
            for tx in 0..binner.tiles_x() {
                let tid = binner.tile_index(tx, ty);
                let ox = tx * TILE_SIZE;
                let oy = ty * TILE_SIZE;
                let tw = (w_us - ox).min(TILE_SIZE);
                let th = (h_us - oy).min(TILE_SIZE);

                for cmd in binner.tile_cmds(tid, &arena) {
                    match cmd.kind {
                        TileCmdKind::SolidTile => {
                            for py in 0..th {
                                for px in 0..tw {
                                    out[(oy + py) * w_us + ox + px] = 65535;
                                }
                            }
                            solid_spans += th;
                        }
                        TileCmdKind::PathFill => {
                            let params = TileFillParams::from_cmd(cmd, tw, th);
                            scratch.rasterize_tile_spans(params, &arena, |span| match span {
                                CoverageSpan::Solid { y, x0, x1 } => {
                                    solid_spans += 1;
                                    for px in x0..x1 {
                                        out[(oy + y) * w_us + ox + px] = 65535;
                                    }
                                }
                                CoverageSpan::Fractional { y, x0, cov } => {
                                    cov_spans += 1;
                                    for (i, &c) in cov.iter().enumerate() {
                                        assert!(c > 0 && c < 65535);
                                        out[(oy + y) * w_us + ox + x0 + i] = c;
                                    }
                                }
                            });
                        }
                        _ => {}
                    }
                }
            }
        }
        (out, solid_spans, cov_spans)
    }

    #[test]
    fn fractional_rect_exact_analytical_coverage_and_sparse_runs() {
        // [10.25, 10.25]..[15.75, 15.75]
        // Corner pixels (10,10), (15,10), (10,15), (15,15): 0.75 * 0.75 = 0.5625
        // Edge pixels: 0.75; Interior pixels [11..=14]x[11..=14]: 1.0
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(10.25, 10.25, 15.75, 15.75));
        let path = pb.finish();

        let (mask, solid_spans, cov_spans) =
            render_full_mask(&path, 32, 32, FillRule::NonZero, AntialiasMode::Analytic);

        let expected_corner = acc_to_cov_u16((0.5625 * 131_072.0) as i32, FillRule::NonZero);
        let expected_edge = acc_to_cov_u16((0.75 * 131_072.0) as i32, FillRule::NonZero);

        for py in 0..32usize {
            for px in 0..32usize {
                let got = mask[py * 32 + px];
                let want = if !(10..=15).contains(&px) || !(10..=15).contains(&py) {
                    0
                } else if (px == 10 || px == 15) && (py == 10 || py == 15) {
                    expected_corner
                } else if px == 10 || px == 15 || py == 10 || py == 15 {
                    expected_edge
                } else {
                    65535
                };
                assert_eq!(got, want, "mismatch at ({px},{py})");
            }
        }
        // 4 interior rows -> 4 solid spans; 2 edge rows (top/bottom) + 4 rows * 2 side edges = 10 cov spans
        assert_eq!(solid_spans, 4);
        assert_eq!(cov_spans, 10);

        // Verify analytical sum of pixel coverages matches exact area (5.5 * 5.5 = 30.25)
        let sum_area: f64 = mask.iter().map(|&c| (c as f64) / 65535.0).sum();
        assert!((sum_area - 30.25).abs() < 1e-4, "sum_area={sum_area}");
    }

    #[test]
    fn crisp_edges_mode_binary_zero_fringe_and_watertight_seam() {
        // Slanted triangle + fractional rect spanning a tile boundary x = 32
        let mut pb = PathBuilder::new();
        pb.move_to(5.3, 4.7)
            .line_to(58.2, 19.4)
            .line_to(22.8, 55.9)
            .close();
        let tri = pb.finish();

        let (mask, solid_spans, cov_spans) =
            render_full_mask(&tri, 64, 64, FillRule::NonZero, AntialiasMode::None);
        assert!(solid_spans > 0);
        // Zero fractional edge spans in crisp-edges mode!
        assert_eq!(cov_spans, 0);
        for &c in &mask {
            assert!(c == 0 || c == 65535, "fringe pixel {c}");
            let u8c = cov_u16_to_u8(c);
            assert!(u8c == 0 || u8c == 255);
        }

        // Two adjacent rects sharing x = 32.5 across tile boundary must be 100% watertight (no gap/overlap)
        let mut left = PathBuilder::new();
        left.push_rect(Rect::new(10.5, 10.5, 32.5, 50.5));
        let mut right = PathBuilder::new();
        right.push_rect(Rect::new(32.5, 10.5, 54.5, 50.5));
        let (ml, _, _) =
            render_full_mask(&left.finish(), 64, 64, FillRule::NonZero, AntialiasMode::None);
        let (mr, _, _) =
            render_full_mask(&right.finish(), 64, 64, FillRule::NonZero, AntialiasMode::None);

        for py in 10..50 {
            for px in 10..54 {
                let a = ml[py * 64 + px] > 0;
                let b = mr[py * 64 + px] > 0;
                assert!(a ^ b, "gap or overlap at ({px},{py}): left={a}, right={b}");
            }
        }
    }

    #[test]
    fn triangle_area_conservation_axis_aligned_and_rotated() {
        let triangles = [
            [(4.0f32, 6.0f32), (48.0, 6.0), (4.0, 52.0)],
            [(10.25, 8.75), (75.5, 19.25), (27.75, 81.5)],
            [(1.5, 30.0), (92.25, 5.5), (61.75, 89.25)],
            [(32.0, 4.0), (60.0, 32.0), (4.0, 60.0)],
        ];

        for tri in triangles {
            let mut pb = PathBuilder::new();
            pb.move_to(tri[0].0, tri[0].1)
                .line_to(tri[1].0, tri[1].1)
                .line_to(tri[2].0, tri[2].1)
                .close();
            let path = pb.finish();

            let (mask, _, _) =
                render_full_mask(&path, 96, 96, FillRule::NonZero, AntialiasMode::Analytic);
            let raster_area: f64 = mask.iter().map(|&c| (c as f64) / 65535.0).sum();

            let ax = (tri[1].0 - tri[0].0) as f64;
            let ay = (tri[1].1 - tri[0].1) as f64;
            let bx = (tri[2].0 - tri[0].0) as f64;
            let by = (tri[2].1 - tri[0].1) as f64;
            let exact_area = 0.5 * (ax * by - ay * bx).abs();

            // Tolerance accounts for 24.8 subpixel quantization along slanted edges across ~100px perimeter
            let rel_err = (raster_area - exact_area).abs() / exact_area;
            assert!(
                rel_err < 1e-4,
                "area mismatch: raster={raster_area}, exact={exact_area}, rel_err={rel_err}"
            );
        }
    }

    #[test]
    fn winding_overflow_resilience_20000_overlapping_polygons() {
        // 20,000 concentric rects (20,000 * 131,072 > i32::MAX): must never panic in debug mode
        let mut pb = PathBuilder::new();
        for _ in 0..20_000 {
            pb.push_rect(Rect::new(4.0, 4.0, 60.0, 60.0));
        }
        let path = pb.finish();

        // NonZero: 20,000 != 0 (and not a multiple of 32,768) -> fully covered interior
        let (nz, _, _) =
            render_full_mask(&path, 64, 64, FillRule::NonZero, AntialiasMode::Analytic);
        assert_eq!(nz[32 * 64 + 32], 65535);
        assert_eq!(nz[0], 0);

        // EvenOdd: 20,000 is even -> completely hollow interior (0 coverage)
        let (eo, _, _) =
            render_full_mask(&path, 64, 64, FillRule::EvenOdd, AntialiasMode::Analytic);
        assert_eq!(eo[32 * 64 + 32], 0);
        assert_eq!(eo[0], 0);

        // Crisp-edges mode with 20,000 layers: 0 panics and exact parity
        let (eo_crisp, _, _) =
            render_full_mask(&path, 64, 64, FillRule::EvenOdd, AntialiasMode::None);
        assert_eq!(eo_crisp[32 * 64 + 32], 0);
    }

    #[test]
    fn tile_mask_u16_matches_sparse_spans_bit_for_bit() {
        let mut pb = PathBuilder::new();
        pb.push_circle(16.3, 15.7, 14.2);
        let path = pb.finish();

        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();
        let mut scratch = TileRasterScratch::new();
        binner.reset(32, 32);
        let range = flatten_path_for_fill(&path, Affine2D::IDENTITY, 32, 32, DEFAULT_TOLERANCE, &mut arena);
        binner.bin_path(range, FillRule::NonZero, 0, 0, &mut arena);

        let cmd = binner.tile_cmds(0, &arena).next().unwrap();
        let params = TileFillParams::from_cmd(cmd, 32, 32);
        let mut mask_direct = [0u16; TILE_AREA];
        scratch.rasterize_tile_mask_u16(params, &arena, &mut mask_direct);

        let mut mask_spans = [0u16; TILE_AREA];
        scratch.rasterize_tile_spans(params, &arena, |span| match span {
            CoverageSpan::Solid { y, x0, x1 } => {
                for px in x0..x1 {
                    mask_spans[y * TILE_SIZE + px] = 65535;
                }
            }
            CoverageSpan::Fractional { y, x0, cov } => {
                for (i, &c) in cov.iter().enumerate() {
                    mask_spans[y * TILE_SIZE + x0 + i] = c;
                }
            }
        });
        assert_eq!(mask_direct, mask_spans);
    }

    #[test]
    fn crisp_edges_matches_point_in_fill_on_odd_viewport_and_evenodd_star() {
        use crate::geom::query::point_in_fill;
        use crate::math::affine::Point;

        let mut spb = PathBuilder::new();
        let cx = 36.25f32;
        let cy = 27.75f32;
        let r = 24.0f32;
        let mut pts = [Point::ZERO; 5];
        for (i, pt) in pts.iter_mut().enumerate() {
            let angle = -core::f32::consts::FRAC_PI_2
                + (i as f32) * 2.0 * core::f32::consts::PI / 5.0;
            let (s, c) = crate::math::float::sincos_f32(angle);
            *pt = Point::new(cx + r * c, cy + r * s);
        }
        spb.move_to(pts[0].x, pts[0].y)
            .line_to(pts[2].x, pts[2].y)
            .line_to(pts[4].x, pts[4].y)
            .line_to(pts[1].x, pts[1].y)
            .line_to(pts[3].x, pts[3].y)
            .close();
        let star = spb.finish();

        for rule in [FillRule::NonZero, FillRule::EvenOdd] {
            let (mask, _, cov_spans) =
                render_full_mask(&star, 75, 53, rule, AntialiasMode::None);
            assert_eq!(cov_spans, 0);
            let mut mismatches = 0usize;
            for py in 0..53usize {
                for px in 0..75usize {
                    let raster_in = mask[py * 75 + px] == 65535;
                    let query_in = point_in_fill(
                        &star,
                        Point::new(px as f32 + 0.5, py as f32 + 0.5),
                        rule,
                        DEFAULT_TOLERANCE,
                    );
                    if raster_in != query_in {
                        mismatches += 1;
                    }
                }
            }
            // Exact pixel-center agreement across 75x53 (allow at most 1-2 boundary ties from 24.8 rounding)
            assert!(mismatches <= 2, "rule={rule:?} mismatches={mismatches}");
        }
    }
}
