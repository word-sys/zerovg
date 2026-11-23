#![forbid(unsafe_code)]

use alloc::vec::Vec;
use core::ops::Range;

use crate::geom::FillRule;
use crate::math::arena::{
    FrameArena, NO_CHUNK, TILE_SIZE, TileCmd, TileCmdKind, TileSeg, TileSegmentChunk,
};
use crate::math::fixed::mul_div_i64;

pub const TILE_SHIFT_PX: u32 = 5;
pub const TILE_SHIFT_FX: u32 = 13;
pub const TILE_FX: i32 = 1 << TILE_SHIFT_FX;
pub const TILE_FX_MASK: i32 = TILE_FX - 1;
pub const NO_CMD: u32 = u32::MAX;

pub const TILE_FLAG_EVEN_ODD: u8 = 1 << 0;
pub const TILE_FLAG_OPAQUE: u8 = 1 << 1;
pub const TILE_FLAG_CRISP_EDGES: u8 = 1 << 2;

#[derive(Debug, Default)]
pub struct TileBinner {
    width: u32,
    height: u32,
    tiles_x: usize,
    tiles_y: usize,
    tile_cmd_heads: Vec<u32>,
    tile_cmd_tails: Vec<u32>,
    tile_cmd_next: Vec<u32>,
    path_heads: Vec<u32>,
    backdrop_delta: Vec<i32>,
}

impl TileBinner {
    pub fn new() -> Self {
        Self::default()
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.width
    }

    #[inline]
    pub fn height(&self) -> u32 {
        self.height
    }

    #[inline]
    pub fn tiles_x(&self) -> usize {
        self.tiles_x
    }

    #[inline]
    pub fn tiles_y(&self) -> usize {
        self.tiles_y
    }

    #[inline]
    pub fn tile_count(&self) -> usize {
        self.tiles_x * self.tiles_y
    }

    #[inline]
    pub fn tile_index(&self, tx: usize, ty: usize) -> usize {
        ty * self.tiles_x + tx
    }

    pub fn reset(&mut self, width: u32, height: u32) {
        self.width = width;
        self.height = height;
        self.tiles_x = (width as usize).div_ceil(TILE_SIZE);
        self.tiles_y = (height as usize).div_ceil(TILE_SIZE);
        let n = self.tiles_x * self.tiles_y;

        self.tile_cmd_heads.clear();
        self.tile_cmd_heads.resize(n, NO_CMD);
        self.tile_cmd_tails.clear();
        self.tile_cmd_tails.resize(n, NO_CMD);
        self.tile_cmd_next.clear();

        self.path_heads.clear();
        self.path_heads.resize(n, NO_CHUNK);
        self.backdrop_delta.clear();
        self.backdrop_delta.resize(n, 0);
    }

    #[inline]
    pub fn push_tile_cmd(&mut self, tid: usize, cmd: TileCmd, arena: &mut FrameArena) {
        let idx = arena.tile_cmds.len() as u32;
        arena.tile_cmds.push(cmd);
        self.tile_cmd_next.push(NO_CMD);

        let tail = self.tile_cmd_tails[tid];
        if tail == NO_CMD {
            self.tile_cmd_heads[tid] = idx;
        } else {
            self.tile_cmd_next[tail as usize] = idx;
        }
        self.tile_cmd_tails[tid] = idx;
    }

    pub fn push_cmd_to_all_tiles(&mut self, cmd: TileCmd, arena: &mut FrameArena) {
        let n = self.tile_count();
        for tid in 0..n {
            self.push_tile_cmd(tid, cmd, arena);
        }
    }

    pub fn push_cmd_to_tile_rect(
        &mut self,
        tx_range: Range<usize>,
        ty_range: Range<usize>,
        cmd: TileCmd,
        arena: &mut FrameArena,
    ) {
        let tx0 = tx_range.start.min(self.tiles_x);
        let tx1 = tx_range.end.min(self.tiles_x);
        let ty0 = ty_range.start.min(self.tiles_y);
        let ty1 = ty_range.end.min(self.tiles_y);
        for ty in ty0..ty1 {
            let row = ty * self.tiles_x;
            for tx in tx0..tx1 {
                self.push_tile_cmd(row + tx, cmd, arena);
            }
        }
    }

    #[inline]
    pub fn tile_has_cmds(&self, tid: usize) -> bool {
        self.tile_cmd_heads[tid] != NO_CMD
    }

    #[inline]
    pub fn tile_cmds<'a>(&'a self, tid: usize, arena: &'a FrameArena) -> TileCmdIter<'a> {
        TileCmdIter {
            cmds: &arena.tile_cmds,
            next: &self.tile_cmd_next,
            cur: self.tile_cmd_heads[tid],
        }
    }

    #[inline]
    pub fn iter_tile_segs(head_chunk: u32, arena: &FrameArena) -> TileSegIter<'_> {
        TileSegIter {
            chunks: &arena.tile_chunks,
            cur_chunk: head_chunk,
            seg_idx: 0,
        }
    }

    // Bins a range of clipped 24.8 EdgeSegs into 32x32 tiles with exact Stokes' backdrop winding.
    pub fn bin_path(
        &mut self,
        edge_range: Range<usize>,
        fill_rule: FillRule,
        cmd_index: u32,
        flags: u8,
        arena: &mut FrameArena,
    ) {
        if edge_range.is_empty() || self.tiles_x == 0 || self.tiles_y == 0 {
            return;
        }
        let tiles_x = self.tiles_x;
        let tiles_y = self.tiles_y;
        let max_x_fx = (tiles_x as i32) << TILE_SHIFT_FX;
        let max_y_fx = (tiles_y as i32) << TILE_SHIFT_FX;

        let mut min_ty = tiles_y;
        let mut max_ty = 0usize;

        for edge_idx in edge_range {
            let e = arena.edges[edge_idx];
            if e.x0 < 0 || e.x1 < 0 || e.y0 < 0 || e.y1 < 0 {
                continue;
            }

            if e.y0 == e.y1 {
                if e.x0 == e.x1 || e.y0 >= max_y_fx {
                    continue;
                }
                let ly = (e.y0 & TILE_FX_MASK) as i16;
                if ly == 0 {
                    continue;
                }
                let ty = (e.y0 >> TILE_SHIFT_FX) as usize;
                let (rx0, rx1, x_dir) = if e.x0 < e.x1 {
                    (e.x0, e.x1, 1i32)
                } else {
                    (e.x1, e.x0, -1i32)
                };
                let tx_start = ((rx0 >> TILE_SHIFT_FX) as usize + 1).max(1);
                let tx_end = ((rx1 >> TILE_SHIFT_FX) as usize).min(tiles_x - 1);
                if tx_start <= tx_end {
                    min_ty = min_ty.min(ty);
                    max_ty = max_ty.max(ty);
                    let comp = compensating_left_seg(ly, x_dir);
                    let row_base = ty * tiles_x;
                    for tx in tx_start..=tx_end {
                        arena.push_tile_seg(&mut self.path_heads[row_base + tx], comp);
                    }
                }
                continue;
            }

            let (sx0, sy0, sx1, sy1, dir) = if e.y0 < e.y1 {
                (e.x0, e.y0, e.x1, e.y1, 1i32)
            } else {
                (e.x1, e.y1, e.x0, e.y0, -1i32)
            };

            let ty_start = (sy0 >> TILE_SHIFT_FX) as usize;
            if ty_start >= tiles_y {
                continue;
            }
            let ty_end = (((sy1 - 1) >> TILE_SHIFT_FX) as usize).min(tiles_y - 1);
            min_ty = min_ty.min(ty_start);
            max_ty = max_ty.max(ty_end);

            let dx = sx1 - sx0;
            let dy = sy1 - sy0;
            let x_dir = if e.x1 > e.x0 { 1i32 } else { -1i32 };

            for ty in ty_start..=ty_end {
                let row_y0 = (ty as i32) << TILE_SHIFT_FX;
                let row_y1 = row_y0 + TILE_FX;
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

                let row_base = ty * tiles_x;

                if x_top == x_bot {
                    if x_top >= max_x_fx {
                        continue;
                    }
                    let tx = (x_top >> TILE_SHIFT_FX) as usize;
                    let lx = (x_top - ((tx as i32) << TILE_SHIFT_FX)) as i16;
                    let ly0 = (y_top - row_y0) as i16;
                    let ly1 = (y_bot - row_y0) as i16;
                    let seg = if dir > 0 {
                        TileSeg {
                            x0: lx,
                            y0: ly0,
                            x1: lx,
                            y1: ly1,
                        }
                    } else {
                        TileSeg {
                            x0: lx,
                            y0: ly1,
                            x1: lx,
                            y1: ly0,
                        }
                    };
                    arena.push_tile_seg(&mut self.path_heads[row_base + tx], seg);
                    if ly0 == 0 && tx + 1 < tiles_x {
                        let btid = row_base + tx + 1;
                        self.backdrop_delta[btid] = self.backdrop_delta[btid].wrapping_add(dir);
                    }
                    continue;
                }

                let (rx0, ry0, rx1, ry1) = if x_top < x_bot {
                    (x_top, y_top, x_bot, y_bot)
                } else {
                    (x_bot, y_bot, x_top, y_top)
                };
                if rx0 >= max_x_fx {
                    continue;
                }

                let tx_min = (rx0 >> TILE_SHIFT_FX) as usize;
                let tx_max = (((rx1 - 1) >> TILE_SHIFT_FX) as usize).min(tiles_x - 1);
                let rdx = rx1 - rx0;
                let rdy = ry1 - ry0;
                let mut prev_y = ry0;

                for tx in tx_min..=tx_max {
                    let col_x0 = (tx as i32) << TILE_SHIFT_FX;
                    let col_x1 = col_x0 + TILE_FX;
                    let seg_x0 = rx0.max(col_x0);
                    let seg_x1 = rx1.min(col_x1);
                    let seg_y0 = prev_y;
                    let seg_y1 = if seg_x1 == rx1 {
                        ry1
                    } else {
                        ry0 + mul_div_i64(rdy, seg_x1 - rx0, rdx)
                    };
                    prev_y = seg_y1;

                    if seg_y0 != seg_y1 {
                        let lx0 = (seg_x0 - col_x0) as i16;
                        let lx1 = (seg_x1 - col_x0) as i16;
                        let ly0 = (seg_y0 - row_y0) as i16;
                        let ly1 = (seg_y1 - row_y0) as i16;
                        let seg = if x_dir > 0 {
                            TileSeg {
                                x0: lx0,
                                y0: ly0,
                                x1: lx1,
                                y1: ly1,
                            }
                        } else {
                            TileSeg {
                                x0: lx1,
                                y0: ly1,
                                x1: lx0,
                                y1: ly0,
                            }
                        };
                        arena.push_tile_seg(&mut self.path_heads[row_base + tx], seg);
                        if (ly0 == 0 || ly1 == 0) && tx + 1 < tiles_x {
                            let btid = row_base + tx + 1;
                            self.backdrop_delta[btid] = self.backdrop_delta[btid].wrapping_add(dir);
                        }
                    }

                    // Compensating left-wall vertical edge for tile (tx + 1, ty)
                    if seg_x1 == col_x1 && tx + 1 < tiles_x {
                        let ly_cross = (seg_y1 - row_y0) as i16;
                        if ly_cross > 0 && ly_cross < TILE_FX as i16 {
                            let comp = compensating_left_seg(ly_cross, x_dir);
                            arena.push_tile_seg(&mut self.path_heads[row_base + tx + 1], comp);
                        }
                    }
                }
            }
        }

        if min_ty > max_ty {
            return;
        }

        let cmd_flags = match fill_rule {
            FillRule::NonZero => flags & !TILE_FLAG_EVEN_ODD,
            FillRule::EvenOdd => flags | TILE_FLAG_EVEN_ODD,
        };

        for ty in min_ty..=max_ty {
            let row_base = ty * tiles_x;
            let mut backdrop = 0i32;
            for tx in 0..tiles_x {
                let tid = row_base + tx;
                backdrop = backdrop.wrapping_add(self.backdrop_delta[tid]);
                self.backdrop_delta[tid] = 0;

                let head = self.path_heads[tid];
                self.path_heads[tid] = NO_CHUNK;

                if head != NO_CHUNK {
                    self.push_tile_cmd(
                        tid,
                        TileCmd {
                            kind: TileCmdKind::PathFill,
                            flags: cmd_flags,
                            aux: 0,
                            cmd_index,
                            head_chunk: head,
                            backdrop,
                        },
                        arena,
                    );
                } else {
                    let covered = match fill_rule {
                        FillRule::NonZero => backdrop != 0,
                        FillRule::EvenOdd => (backdrop & 1) != 0,
                    };
                    if covered {
                        self.push_tile_cmd(
                            tid,
                            TileCmd {
                                kind: TileCmdKind::SolidTile,
                                flags: cmd_flags,
                                aux: 0,
                                cmd_index,
                                head_chunk: NO_CHUNK,
                                backdrop,
                            },
                            arena,
                        );
                    }
                }
            }
        }
    }
}

#[inline]
fn compensating_left_seg(ly: i16, x_dir: i32) -> TileSeg {
    if x_dir < 0 {
        TileSeg {
            x0: 0,
            y0: ly,
            x1: 0,
            y1: TILE_FX as i16,
        }
    } else {
        TileSeg {
            x0: 0,
            y0: TILE_FX as i16,
            x1: 0,
            y1: ly,
        }
    }
}

pub struct TileCmdIter<'a> {
    cmds: &'a [TileCmd],
    next: &'a [u32],
    cur: u32,
}

impl Iterator for TileCmdIter<'_> {
    type Item = TileCmd;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        if self.cur == NO_CMD {
            return None;
        }
        let idx = self.cur as usize;
        self.cur = self.next[idx];
        Some(self.cmds[idx])
    }
}

pub struct TileSegIter<'a> {
    chunks: &'a [TileSegmentChunk],
    cur_chunk: u32,
    seg_idx: usize,
}

impl Iterator for TileSegIter<'_> {
    type Item = TileSeg;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while self.cur_chunk != NO_CHUNK {
            let chunk = &self.chunks[self.cur_chunk as usize];
            if self.seg_idx < chunk.len as usize {
                let seg = chunk.segs[self.seg_idx];
                self.seg_idx += 1;
                return Some(seg);
            }
            self.cur_chunk = chunk.next;
            self.seg_idx = 0;
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;
    use crate::geom::flatten::{DEFAULT_TOLERANCE, flatten_path_for_fill};
    use crate::geom::PathBuilder;
    use crate::math::affine::{Affine2D, Point, Rect};
    use crate::math::arena::EdgeSeg;

    // Accumulates one 24.8 segment into a differential grid `diff` of dimensions `grid_h x (grid_w + 1)`.
    fn accumulate_seg_into_grid(
        diff: &mut [i32],
        grid_w: usize,
        grid_h: usize,
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
    ) {
        if y0 == y1 {
            return;
        }
        let stride = grid_w + 1;
        let (sx0, sy0, sx1, sy1, dir) = if y0 < y1 {
            (x0, y0, x1, y1, 1i32)
        } else {
            (x1, y1, x0, y0, -1i32)
        };
        let py_start = (sy0 >> 8).max(0) as usize;
        if py_start >= grid_h {
            return;
        }
        let py_end = (((sy1 - 1) >> 8) as usize).min(grid_h - 1);
        let dx = sx1 - sx0;
        let dy = sy1 - sy0;

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

            let row_off = py * stride;
            if x_top == x_bot {
                let px = (x_top >> 8) as usize;
                if px < grid_w {
                    let fx = x_top & 255;
                    let sdy = (y_bot - y_top) * dir;
                    let area_2x = sdy * (512 - (fx + fx));
                    diff[row_off + px] = diff[row_off + px].wrapping_add(area_2x);
                    diff[row_off + px + 1] =
                        diff[row_off + px + 1].wrapping_add((sdy << 9) - area_2x);
                }
                continue;
            }

            let (rx0, ry0, rx1, ry1) = if x_top < x_bot {
                (x_top, y_top, x_bot, y_bot)
            } else {
                (x_bot, y_bot, x_top, y_top)
            };
            let px_min = (rx0 >> 8).max(0) as usize;
            if px_min >= grid_w {
                continue;
            }
            let px_max = (((rx1 - 1) >> 8) as usize).min(grid_w - 1);
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
                    diff[row_off + px] = diff[row_off + px].wrapping_add(area_2x);
                    diff[row_off + px + 1] =
                        diff[row_off + px + 1].wrapping_add((sdy << 9) - area_2x);
                }
            }
        }
    }

    fn acc_to_cov_u16(acc: i32, fill_rule: FillRule) -> u16 {
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

    // Brute-force full-frame analytic reference rasterizer (no tiling).
    fn rasterize_global_ref(
        edges: &[EdgeSeg],
        w: usize,
        h: usize,
        fill_rule: FillRule,
    ) -> Vec<u16> {
        let mut diff = vec![0i32; h * (w + 1)];
        for e in edges {
            accumulate_seg_into_grid(&mut diff, w, h, e.x0, e.y0, e.x1, e.y1);
        }
        let mut out = vec![0u16; w * h];
        for py in 0..h {
            let row = py * (w + 1);
            let mut acc = 0i32;
            for px in 0..w {
                acc = acc.wrapping_add(diff[row + px]);
                out[py * w + px] = acc_to_cov_u16(acc, fill_rule);
            }
        }
        out
    }

    // Evaluates TileBinner's per-tile commands (PathFill + SolidTile) into a full-frame coverage buffer.
    fn rasterize_tiled(
        binner: &TileBinner,
        arena: &FrameArena,
        w: usize,
        h: usize,
        fill_rule: FillRule,
    ) -> Vec<u16> {
        let mut out = vec![0u16; w * h];
        let mut diff = [0i32; TILE_SIZE * (TILE_SIZE + 1)];

        for ty in 0..binner.tiles_y() {
            for tx in 0..binner.tiles_x() {
                let tid = binner.tile_index(tx, ty);
                for cmd in binner.tile_cmds(tid, arena) {
                    let ox = tx * TILE_SIZE;
                    let oy = ty * TILE_SIZE;
                    let bw = (w - ox).min(TILE_SIZE);
                    let bh = (h - oy).min(TILE_SIZE);

                    match cmd.kind {
                        TileCmdKind::SolidTile => {
                            for py in 0..bh {
                                for px in 0..bw {
                                    out[(oy + py) * w + ox + px] = 65535;
                                }
                            }
                        }
                        TileCmdKind::PathFill => {
                            diff.fill(0);
                            for seg in TileBinner::iter_tile_segs(cmd.head_chunk, arena) {
                                accumulate_seg_into_grid(
                                    &mut diff,
                                    TILE_SIZE,
                                    TILE_SIZE,
                                    seg.x0 as i32,
                                    seg.y0 as i32,
                                    seg.x1 as i32,
                                    seg.y1 as i32,
                                );
                            }
                            let init_acc = cmd.backdrop.wrapping_shl(17);
                            for py in 0..bh {
                                let row = py * (TILE_SIZE + 1);
                                let mut acc = init_acc;
                                for px in 0..bw {
                                    acc = acc.wrapping_add(diff[row + px]);
                                    out[(oy + py) * w + ox + px] = acc_to_cov_u16(acc, fill_rule);
                                }
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
        out
    }

    fn check_tiled_vs_global(
        path: &crate::geom::Path,
        transform: Affine2D,
        size: (u32, u32),
        fill_rule: FillRule,
        require_strict_zero: bool,
        binner: &mut TileBinner,
        arena: &mut FrameArena,
    ) {
        let (w, h) = size;
        arena.reset();
        binner.reset(w, h);
        let range = flatten_path_for_fill(path, transform, w, h, DEFAULT_TOLERANCE, arena);
        binner.bin_path(range.clone(), fill_rule, 0, 0, arena);

        let global = rasterize_global_ref(&arena.edges[range], w as usize, h as usize, fill_rule);
        let tiled = rasterize_tiled(binner, arena, w as usize, h as usize, fill_rule);

        let w_us = w as usize;
        let h_us = h as usize;
        let mut max_diff = 0u32;
        let mut worst_xy = (0usize, 0usize, 0u16, 0u16);
        for py in 0..h_us {
            for px in 0..w_us {
                let idx = py * w_us + px;
                let g = global[idx];
                let t = tiled[idx];
                let d = g.abs_diff(t) as u32;
                if d > max_diff {
                    max_diff = d;
                    worst_xy = (px, py, g, t);
                }
                if d > 1024 {
                    panic!(
                        "tile seam/winding mismatch at ({px},{py}): global={g}, tiled={t}, diff={d}, rule={fill_rule:?}"
                    );
                }
                // Any pixel strictly in interior or exterior (same solid/empty value as its 4-neighbors)
                // must be 100% bit-exact (0 winding leaks across tile seams).
                if (g == 0 || g == 65535) && px > 0 && px + 1 < w_us && py > 0 && py + 1 < h_us {
                    let interior = global[idx - 1] == g
                        && global[idx + 1] == g
                        && global[idx - w_us] == g
                        && global[idx + w_us] == g;
                    if interior {
                        assert_eq!(
                            t, g,
                            "interior/exterior seam leak at ({px},{py}): global={g}, tiled={t}, rule={fill_rule:?}"
                        );
                    }
                }
            }
        }
        assert!(
            max_diff <= 600,
            "excessive diff={max_diff} at ({},{}) global={} tiled={} rule={fill_rule:?}",
            worst_xy.0,
            worst_xy.1,
            worst_xy.2,
            worst_xy.3
        );
        if require_strict_zero {
            assert_eq!(
                max_diff, 0,
                "expected bit-exact agreement, worst at ({},{}) global={} tiled={}",
                worst_xy.0, worst_xy.1, worst_xy.2, worst_xy.3
            );
        }
    }

    #[test]
    fn exact_rectangles_solid_tiles_and_tile_boundaries() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        // 1. Fractional rect spanning 3x2 tiles (exercises horizontal edge Stokes' compensation)
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(10.25, 5.3, 80.75, 50.7));
        let rect_path = pb.finish();
        check_tiled_vs_global(
            &rect_path,
            Affine2D::IDENTITY,
            (128, 96),
            FillRule::NonZero,
            true,
            &mut binner,
            &mut arena,
        );

        // 2. Large rect verifying interior tiles become SolidTile (head_chunk == NO_CHUNK)
        let mut big = PathBuilder::new();
        big.push_rect(Rect::new(5.0, 5.0, 120.0, 90.0));
        let big_path = big.finish();
        check_tiled_vs_global(
            &big_path,
            Affine2D::IDENTITY,
            (128, 96),
            FillRule::NonZero,
            true,
            &mut binner,
            &mut arena,
        );
        // Tiles (1, 1) and (2, 1) are strictly interior -> must be SolidTile!
        for tx in [1, 2] {
            let cmds: Vec<TileCmd> = binner
                .tile_cmds(binner.tile_index(tx, 1), &arena)
                .collect();
            assert_eq!(cmds.len(), 1);
            assert_eq!(cmds[0].kind, TileCmdKind::SolidTile);
            assert_eq!(cmds[0].head_chunk, NO_CHUNK);
        }

        // 3. Rectangles aligned directly on tile walls x = 32, 64 and y = 32, 64
        for rect in [
            Rect::new(0.0, 32.0, 64.0, 64.0),
            Rect::new(32.0, 0.0, 64.0, 32.0),
            Rect::new(32.0, 32.0, 96.0, 64.0),
            Rect::new(10.0, 32.0, 50.0, 50.0),
            Rect::new(10.0, 10.0, 50.0, 32.0),
        ] {
            let mut rpb = PathBuilder::new();
            rpb.push_rect(rect);
            let p = rpb.finish();
            check_tiled_vs_global(
                &p,
                Affine2D::IDENTITY,
                (128, 96),
                FillRule::NonZero,
                true,
                &mut binner,
                &mut arena,
            );
        }
    }

    #[test]
    fn exact_diagonals_through_tile_corners_and_killer_subpixel_edges() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        // Triangles whose hypotenuse passes directly through (32, 32) in all 4 orientations
        let triangles = [
            [(0.0, 0.0), (64.0, 64.0), (0.0, 64.0)],
            [(0.0, 0.0), (64.0, 64.0), (64.0, 0.0)],
            [(64.0, 0.0), (0.0, 64.0), (64.0, 64.0)],
            [(64.0, 0.0), (0.0, 64.0), (0.0, 0.0)],
            // Reverse winding of each
            [(0.0, 64.0), (64.0, 64.0), (0.0, 0.0)],
            [(64.0, 0.0), (64.0, 64.0), (0.0, 0.0)],
            [(64.0, 64.0), (0.0, 64.0), (64.0, 0.0)],
            [(0.0, 0.0), (0.0, 64.0), (64.0, 0.0)],
        ];
        for tri in triangles {
            let mut pb = PathBuilder::new();
            pb.move_to(tri[0].0, tri[0].1)
                .line_to(tri[1].0, tri[1].1)
                .line_to(tri[2].0, tri[2].1)
                .close();
            let p = pb.finish();
            check_tiled_vs_global(
                &p,
                Affine2D::IDENTITY,
                (96, 96),
                FillRule::NonZero,
                true,
                &mut binner,
                &mut arena,
            );
        }

        // Direct 24.8 killer subpixel triangles around tile corner (8192, 8192)
        // including (8189, 8191) -> (8194, 8193) where independent x_cross/y_cross roundings disagree
        let killer_polys: [&[(i32, i32)]; 4] = [
            &[(8189, 8191), (8194, 8193), (8189, 8193)],
            &[(8189, 8191), (8194, 8193), (8194, 8191)],
            &[(8190, 8191), (8195, 8193), (8190, 8193)],
            &[(8188, 8191), (8193, 8192), (8193, 8191)],
        ];
        for poly in killer_polys {
            arena.reset();
            binner.reset(96, 96);
            for i in 0..poly.len() {
                let (x0, y0) = poly[i];
                let (x1, y1) = poly[(i + 1) % poly.len()];
                arena.edges.push(EdgeSeg { x0, y0, x1, y1 });
            }
            let range = 0..arena.edges.len();
            binner.bin_path(range.clone(), FillRule::NonZero, 0, 0, &mut arena);
            let global = rasterize_global_ref(&arena.edges[range], 96, 96, FillRule::NonZero);
            let tiled = rasterize_tiled(&binner, &arena, 96, 96, FillRule::NonZero);
            for py in 0..96 {
                for px in 0..96 {
                    let g = global[py * 96 + px];
                    let t = tiled[py * 96 + px];
                    assert!(
                        g.abs_diff(t) <= 256,
                        "killer poly mismatch at ({px},{py}): global={g} tiled={t}"
                    );
                }
            }
            // Specifically verify tiles right of (1, 1) have 0 residual winding leak!
            assert_eq!(tiled[32 * 96 + 64], 0);
            assert_eq!(tiled[31 * 96 + 64], 0);
            assert_eq!(tiled[33 * 96 + 64], 0);
        }
    }

    #[test]
    fn offscreen_huge_coords_and_non_multiple_viewport() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        // Huge polygon extending to x = -1_000_000, y = -50_000, y = +50_000 on odd viewport 100x77
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(-1_000_000.0, -50_000.0, 50.25, 50_000.0));
        let p = pb.finish();
        check_tiled_vs_global(
            &p,
            Affine2D::IDENTITY,
            (100, 77),
            FillRule::NonZero,
            true,
            &mut binner,
            &mut arena,
        );

        // Huge polygon extending to x = -1_000_000 and x = +1_000_000 covering [0, 100) x [10.5, 65.5)
        let mut pb2 = PathBuilder::new();
        pb2.push_rect(Rect::new(-1_000_000.0, 10.5, 1_000_000.0, 65.5));
        let p2 = pb2.finish();
        check_tiled_vs_global(
            &p2,
            Affine2D::IDENTITY,
            (100, 77),
            FillRule::NonZero,
            true,
            &mut binner,
            &mut arena,
        );

        // Circle partially off-screen on left and bottom
        let mut cpb = PathBuilder::new();
        cpb.push_circle(-40.0, 45.0, 110.0);
        let circle = cpb.finish();
        check_tiled_vs_global(
            &circle,
            Affine2D::IDENTITY,
            (100, 77),
            FillRule::NonZero,
            false,
            &mut binner,
            &mut arena,
        );
    }

    #[test]
    fn subpixel_sweep_256_offsets_star_and_circle() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        // Self-intersecting 5-point star centered near tile corner (32, 32)
        let mut spb = PathBuilder::new();
        let cx = 32.0f32;
        let cy = 32.0f32;
        let r = 28.0f32;
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

        // Sweep every single subpixel offset 0/256 .. 255/256 across tile seams
        for step in 0..256 {
            let offset = (step as f32) / 256.0;
            let xform = Affine2D::translate(offset, offset);
            let rule = if step & 1 == 0 {
                FillRule::NonZero
            } else {
                FillRule::EvenOdd
            };
            check_tiled_vs_global(&star, xform, (96, 96), rule, false, &mut binner, &mut arena);
        }
    }

    #[test]
    fn randomized_200_complex_polygons_match_global_reference() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        fn next_u64(seed: &mut u64) -> u64 {
            *seed ^= *seed >> 12;
            *seed ^= *seed << 25;
            *seed ^= *seed >> 27;
            seed.wrapping_mul(0x2545_f491_4f6c_dd1d)
        }
        fn next_f32(seed: &mut u64, lo: f32, hi: f32) -> f32 {
            let u = ((next_u64(seed) >> 40) as f32) / ((1u32 << 24) as f32);
            lo + u * (hi - lo)
        }

        let mut seed = 0x9e37_79b9_7f4a_7c15u64;

        for case in 0..200 {
            let mut pb = PathBuilder::new();
            let subpaths = 1 + (next_u64(&mut seed) % 3) as usize;
            for _ in 0..subpaths {
                let verts = 3 + (next_u64(&mut seed) % 10) as usize;
                let x0 = next_f32(&mut seed, -30.0, 150.0);
                let y0 = next_f32(&mut seed, -30.0, 120.0);
                pb.move_to(x0, y0);
                for _ in 1..verts {
                    let x = next_f32(&mut seed, -30.0, 150.0);
                    let y = next_f32(&mut seed, -30.0, 120.0);
                    // Occasionally snap coordinates to exact tile walls to torture boundary predicates
                    let x = match next_u64(&mut seed) % 12 {
                        0 => 0.0,
                        1 => 32.0,
                        2 => 64.0,
                        3 => 96.0,
                        _ => x,
                    };
                    let y = match next_u64(&mut seed) % 12 {
                        0 => 0.0,
                        1 => 32.0,
                        2 => 64.0,
                        _ => y,
                    };
                    pb.line_to(x, y);
                }
                pb.close();
            }
            let path = pb.finish();
            let rule = if case & 1 == 0 {
                FillRule::NonZero
            } else {
                FillRule::EvenOdd
            };
            check_tiled_vs_global(
                &path,
                Affine2D::IDENTITY,
                (128, 96),
                rule,
                false,
                &mut binner,
                &mut arena,
            );
        }
    }

    #[test]
    fn even_odd_double_overlap_omits_hollow_solid_tile() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();

        // Two identical large rects -> interior winding = 2 -> EvenOdd must be hollow (0 commands on tile (1, 1))
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(5.0, 5.0, 120.0, 90.0));
        pb.push_rect(Rect::new(5.0, 5.0, 120.0, 90.0));
        let p = pb.finish();

        arena.reset();
        binner.reset(128, 96);
        let range = flatten_path_for_fill(&p, Affine2D::IDENTITY, 128, 96, DEFAULT_TOLERANCE, &mut arena);
        binner.bin_path(range, FillRule::EvenOdd, 0, 0, &mut arena);

        let tid = binner.tile_index(1, 1);
        assert!(!binner.tile_has_cmds(tid));
    }

    #[test]
    fn multi_path_fifo_order_and_curved_high_winding() {
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();
        binner.reset(128, 96);

        binner.push_cmd_to_all_tiles(
            TileCmd {
                kind: TileCmdKind::Clear,
                flags: TILE_FLAG_OPAQUE,
                aux: 0,
                cmd_index: 0,
                head_chunk: NO_CHUNK,
                backdrop: 0,
            },
            &mut arena,
        );

        let mut pb1 = PathBuilder::new();
        pb1.push_rect(Rect::new(5.0, 5.0, 120.0, 90.0));
        let p1 = pb1.finish();
        let r1 = flatten_path_for_fill(&p1, Affine2D::IDENTITY, 128, 96, DEFAULT_TOLERANCE, &mut arena);
        binner.bin_path(r1, FillRule::NonZero, 1, TILE_FLAG_OPAQUE, &mut arena);

        let mut pb2 = PathBuilder::new();
        pb2.push_circle(64.0, 48.0, 35.0);
        let p2 = pb2.finish();
        let r2 = flatten_path_for_fill(&p2, Affine2D::IDENTITY, 128, 96, DEFAULT_TOLERANCE, &mut arena);
        binner.bin_path(r2, FillRule::EvenOdd, 2, 0, &mut arena);

        // Tile (1, 1) must receive Clear (0), SolidTile (1), PathFill (2) in exact FIFO draw order
        let cmds: Vec<TileCmd> = binner.tile_cmds(binner.tile_index(1, 1), &arena).collect();
        assert_eq!(cmds.len(), 3);
        assert_eq!(cmds[0].kind, TileCmdKind::Clear);
        assert_eq!(cmds[0].cmd_index, 0);
        assert_eq!(cmds[1].kind, TileCmdKind::SolidTile);
        assert_eq!(cmds[1].cmd_index, 1);
        assert_eq!(cmds[1].flags, TILE_FLAG_OPAQUE);
        assert_eq!(cmds[2].kind, TileCmdKind::PathFill);
        assert_eq!(cmds[2].cmd_index, 2);
        assert_eq!(cmds[2].flags, TILE_FLAG_EVEN_ODD);

        // Curved shapes + high winding depth (200 concentric rects) must agree with global ref with 0 panics
        let mut rrect = PathBuilder::new();
        rrect.push_rounded_rect(Rect::new(10.5, 12.25, 115.75, 83.5), 18.0, 14.0);
        let rr_path = rrect.finish();
        check_tiled_vs_global(
            &rr_path,
            Affine2D::IDENTITY,
            (128, 96),
            FillRule::NonZero,
            false,
            &mut binner,
            &mut arena,
        );

        let mut deep = PathBuilder::new();
        for i in 0..200 {
            let inset = (i as f32) * 0.1;
            deep.push_rect(Rect::new(4.0 + inset, 4.0 + inset, 124.0 - inset, 92.0 - inset));
        }
        let deep_path = deep.finish();
        check_tiled_vs_global(
            &deep_path,
            Affine2D::IDENTITY,
            (128, 96),
            FillRule::NonZero,
            true,
            &mut binner,
            &mut arena,
        );
        check_tiled_vs_global(
            &deep_path,
            Affine2D::IDENTITY,
            (128, 96),
            FillRule::EvenOdd,
            true,
            &mut binner,
            &mut arena,
        );
    }
}
