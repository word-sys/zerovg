#![forbid(unsafe_code)]

use alloc::vec::Vec;

use crate::error::{Result, ZvgError};
use crate::geom::bounds::path_tight_bounds;
use crate::geom::flatten::{DEFAULT_TOLERANCE, flatten_path_for_fill, flatten_path_segments};
use crate::geom::stroke::{LineCap, LineJoin, StrokeStyle, expand_stroke_into};
use crate::geom::{FillRule, Path, PathBuilder};
use crate::math::affine::{Affine2D, Point, Rect, Vec2};
use crate::math::arena::{FrameArena, NO_CHUNK, TILE_SIZE, TileCmd, TileCmdKind};
use crate::math::color::{Color, LinearRgba16, LinearRgbaF32};
use crate::math::dither::DitherMode;
use crate::math::float::clamp_finite_f32;
use crate::raster::simd::{
    TileBuffer16, TileBufferF32, fill_cov_span_src_over_f32, fill_cov_span_src_over_u16,
    fill_solid_span_src_over_f32, fill_solid_span_src_over_u16, fill_solid_tile_src_over_f32,
    fill_solid_tile_src_over_u16, load_tile_f32, load_tile_u16, store_tile_f32, store_tile_u16,
    surface_uses_f32_pipeline,
};
use crate::raster::{AntialiasMode, CoverageSpan, TileFillParams, TileRasterScratch};
use crate::surface::{Surface, SurfaceMut};
use crate::tile::{TILE_FLAG_CRISP_EDGES, TILE_FLAG_OPAQUE, TileBinner};

#[derive(Debug, Clone, Copy)]
struct ContextState {
    transform: Affine2D,
    paint: Color,
    fill_rule: FillRule,
    antialias: AntialiasMode,
    dither_mode: DitherMode,
    tolerance: f32,
    line_width: f32,
    line_cap: LineCap,
    line_join: LineJoin,
    miter_limit: f32,
    non_scaling_stroke: bool,
}

impl Default for ContextState {
    fn default() -> Self {
        Self {
            transform: Affine2D::IDENTITY,
            paint: Color::BLACK,
            fill_rule: FillRule::NonZero,
            antialias: AntialiasMode::Analytic,
            dither_mode: DitherMode::BlueNoise,
            tolerance: DEFAULT_TOLERANCE,
            line_width: 1.0,
            line_cap: LineCap::Butt,
            line_join: LineJoin::Miter,
            miter_limit: 4.0,
            non_scaling_stroke: false,
        }
    }
}

pub struct Context<'a> {
    surface: SurfaceMut<'a>,
    state: ContextState,
    state_stack: Vec<ContextState>,
    current_path: PathBuilder,
    scratch_path: Path,
    stroke_builder: PathBuilder,
    arena: FrameArena,
    binner: TileBinner,
    scratch: TileRasterScratch,
    tile_buf16: TileBuffer16,
    tile_buf_f32: TileBufferF32,
    draw_colors16: Vec<LinearRgba16>,
    draw_colors_f32: Vec<LinearRgbaF32>,
    pending_clear: Option<Color>,
    use_f32: bool,
}

impl<'a> Context<'a> {
    pub fn new(surface: &'a mut Surface) -> Self {
        Self::from_surface_mut(surface.as_mut())
    }

    pub fn from_surface_mut(surface: SurfaceMut<'a>) -> Self {
        let w = surface.width();
        let h = surface.height();
        let use_f32 = surface_uses_f32_pipeline(surface.format(), surface.color_space());
        let mut binner = TileBinner::new();
        binner.reset(w, h);
        Self {
            surface,
            state: ContextState::default(),
            state_stack: Vec::new(),
            current_path: PathBuilder::new(),
            scratch_path: Path::empty(),
            stroke_builder: PathBuilder::new(),
            arena: FrameArena::new(),
            binner,
            scratch: TileRasterScratch::new(),
            tile_buf16: TileBuffer16::new(),
            tile_buf_f32: TileBufferF32::new(),
            draw_colors16: Vec::new(),
            draw_colors_f32: Vec::new(),
            pending_clear: None,
            use_f32,
        }
    }

    #[inline]
    pub fn width(&self) -> u32 {
        self.surface.width()
    }

    #[inline]
    pub fn height(&self) -> u32 {
        self.surface.height()
    }

    #[inline]
    pub fn save(&mut self) {
        self.state_stack.push(self.state);
    }

    #[inline]
    pub fn restore(&mut self) -> bool {
        if let Some(prev) = self.state_stack.pop() {
            let old_ctm = self.state.transform;
            self.state = prev;
            self.adapt_current_path_ctm(old_ctm, self.state.transform);
            true
        } else {
            false
        }
    }

    #[inline]
    pub fn try_restore(&mut self) -> Result<()> {
        if self.restore() {
            Ok(())
        } else {
            Err(ZvgError::InvalidArgument("context state stack underflow"))
        }
    }

    fn adapt_current_path_ctm(&mut self, old_ctm: Affine2D, new_ctm: Affine2D) {
        if self.current_path.is_empty() || old_ctm == new_ctm {
            return;
        }
        if let Some(inv_new) = new_ctm.invert() {
            self.current_path.transform_in_place(inv_new * old_ctm);
        }
    }

    #[inline]
    pub fn transform(&self) -> Affine2D {
        self.state.transform
    }

    #[inline]
    pub fn set_transform(&mut self, transform: Affine2D) {
        if !transform.is_finite() {
            return;
        }
        let old_ctm = self.state.transform;
        self.state.transform = transform;
        self.adapt_current_path_ctm(old_ctm, transform);
    }

    #[inline]
    pub fn reset_transform(&mut self) {
        self.set_transform(Affine2D::IDENTITY);
    }

    #[inline]
    pub fn concat_transform(&mut self, transform: Affine2D) {
        if !transform.is_finite() {
            return;
        }
        let new_ctm = self.state.transform.pre_concat(transform);
        self.set_transform(new_ctm);
    }

    #[inline]
    pub fn translate(&mut self, tx: f32, ty: f32) {
        self.concat_transform(Affine2D::translate(tx, ty));
    }

    #[inline]
    pub fn scale(&mut self, sx: f32, sy: f32) {
        self.concat_transform(Affine2D::scale(sx, sy));
    }

    #[inline]
    pub fn rotate(&mut self, radians: f32) {
        self.concat_transform(Affine2D::rotate(radians));
    }

    #[inline]
    pub fn set_paint(&mut self, color: Color) {
        self.state.paint = color;
    }

    #[inline]
    pub fn set_source_rgb(&mut self, r: f32, g: f32, b: f32) {
        self.state.paint = Color::from_rgb(r, g, b);
    }

    #[inline]
    pub fn set_source_rgba(&mut self, r: f32, g: f32, b: f32, a: f32) {
        self.state.paint = Color::new(r, g, b, a);
    }

    #[inline]
    pub fn paint_color(&self) -> Color {
        self.state.paint
    }

    #[inline]
    pub fn set_fill_rule(&mut self, fill_rule: FillRule) {
        self.state.fill_rule = fill_rule;
    }

    #[inline]
    pub fn fill_rule(&self) -> FillRule {
        self.state.fill_rule
    }

    #[inline]
    pub fn set_antialias(&mut self, mode: AntialiasMode) {
        self.state.antialias = mode;
    }

    #[inline]
    pub fn antialias(&self) -> AntialiasMode {
        self.state.antialias
    }

    #[inline]
    pub fn set_dither_mode(&mut self, mode: DitherMode) {
        self.state.dither_mode = mode;
    }

    #[inline]
    pub fn dither_mode(&self) -> DitherMode {
        self.state.dither_mode
    }

    #[inline]
    pub fn set_tolerance(&mut self, tolerance: f32) {
        if tolerance.is_finite() && tolerance > 0.0 {
            self.state.tolerance = tolerance.max(1e-4);
        }
    }

    #[inline]
    pub fn tolerance(&self) -> f32 {
        self.state.tolerance
    }

    #[inline]
    pub fn set_line_width(&mut self, width: f32) {
        if width.is_finite() && width >= 0.0 {
            self.state.line_width = width;
        }
    }

    #[inline]
    pub fn line_width(&self) -> f32 {
        self.state.line_width
    }

    #[inline]
    pub fn set_line_cap(&mut self, cap: LineCap) {
        self.state.line_cap = cap;
    }

    #[inline]
    pub fn line_cap(&self) -> LineCap {
        self.state.line_cap
    }

    #[inline]
    pub fn set_line_join(&mut self, join: LineJoin) {
        self.state.line_join = join;
    }

    #[inline]
    pub fn line_join(&self) -> LineJoin {
        self.state.line_join
    }

    #[inline]
    pub fn set_miter_limit(&mut self, miter_limit: f32) {
        if miter_limit.is_finite() && miter_limit >= 1.0 {
            self.state.miter_limit = miter_limit;
        }
    }

    #[inline]
    pub fn miter_limit(&self) -> f32 {
        self.state.miter_limit
    }

    #[inline]
    pub fn set_stroke_style(&mut self, style: StrokeStyle<'_>) {
        self.set_line_width(style.width);
        self.state.line_cap = style.cap;
        self.state.line_join = style.join;
        self.set_miter_limit(style.miter_limit);
        self.state.non_scaling_stroke = style.non_scaling;
    }

    #[inline]
    pub fn stroke_style(&self) -> StrokeStyle<'static> {
        StrokeStyle {
            width: self.state.line_width,
            cap: self.state.line_cap,
            join: self.state.line_join,
            miter_limit: self.state.miter_limit,
            dash_array: None,
            dash_offset: 0.0,
            non_scaling: self.state.non_scaling_stroke,
        }
    }

    // Clears the entire surface; supersedes any previously queued commands.
    pub fn clear(&mut self, color: Color) {
        let w = self.surface.width();
        let h = self.surface.height();
        self.arena.reset();
        self.binner.reset(w, h);
        self.draw_colors16.clear();
        self.draw_colors_f32.clear();
        self.pending_clear = Some(color);
    }

    // Modern immutable path fill API.
    pub fn fill(&mut self, path: &Path, fill_rule: FillRule) {
        self.fill_internal(path, self.state.transform, fill_rule, self.state.paint);
    }

    pub fn fill_with_paint(&mut self, path: &Path, fill_rule: FillRule, color: Color) {
        self.fill_internal(path, self.state.transform, fill_rule, color);
    }

    pub fn stroke(&mut self, path: &Path, style: &StrokeStyle<'_>) {
        if path.is_empty() || !style.width.is_finite() || style.width <= 0.0 {
            return;
        }
        expand_stroke_into(
            path,
            self.state.transform,
            style,
            self.state.tolerance,
            &mut self.stroke_builder,
        );
        if self.stroke_builder.finish_into(&mut self.scratch_path).is_ok() {
            let color = self.state.paint;
            self.fill_scratch_path(Affine2D::IDENTITY, FillRule::NonZero, color);
        }
    }

    // --- Cairo-Compatible Stateful Current-Path API ---

    #[inline]
    pub fn new_path(&mut self) {
        self.current_path.clear();
    }

    #[inline]
    pub fn new_sub_path(&mut self) {
        self.current_path.new_sub_path();
    }

    #[inline]
    pub fn move_to(&mut self, x: f32, y: f32) {
        self.current_path.move_to(x, y);
    }

    #[inline]
    pub fn rel_move_to(&mut self, dx: f32, dy: f32) {
        self.current_path.rel_move_to(dx, dy);
    }

    #[inline]
    pub fn line_to(&mut self, x: f32, y: f32) {
        self.current_path.line_to(x, y);
    }

    #[inline]
    pub fn rel_line_to(&mut self, dx: f32, dy: f32) {
        self.current_path.rel_line_to(dx, dy);
    }

    #[inline]
    pub fn quad_to(&mut self, cx: f32, cy: f32, x: f32, y: f32) {
        self.current_path.quad_to(cx, cy, x, y);
    }

    #[inline]
    pub fn rel_quad_to(&mut self, dcx: f32, dcy: f32, dx: f32, dy: f32) {
        self.current_path.rel_quad_to(dcx, dcy, dx, dy);
    }

    #[inline]
    pub fn curve_to(&mut self, c1x: f32, c1y: f32, c2x: f32, c2y: f32, x: f32, y: f32) {
        self.current_path.cubic_to(c1x, c1y, c2x, c2y, x, y);
    }

    #[inline]
    pub fn rel_curve_to(
        &mut self,
        dc1x: f32,
        dc1y: f32,
        dc2x: f32,
        dc2y: f32,
        dx: f32,
        dy: f32,
    ) {
        self.current_path
            .rel_cubic_to(dc1x, dc1y, dc2x, dc2y, dx, dy);
    }

    #[inline]
    pub fn arc(&mut self, cx: f32, cy: f32, r: f32, a1: f32, a2: f32) {
        self.current_path.arc_center(cx, cy, r, a1, a2, false);
    }

    #[inline]
    pub fn arc_negative(&mut self, cx: f32, cy: f32, r: f32, a1: f32, a2: f32) {
        self.current_path.arc_center(cx, cy, r, a1, a2, true);
    }

    #[inline]
    pub fn rectangle(&mut self, x: f32, y: f32, width: f32, height: f32) {
        self.current_path
            .push_rect(Rect::from_xywh(x, y, width, height));
    }

    #[inline]
    pub fn close_path(&mut self) {
        self.current_path.close();
    }

    #[inline]
    pub fn current_point(&self) -> Option<Point> {
        self.current_path.current_point()
    }

    pub fn fill_preserve(&mut self) {
        if self.current_path.is_empty() {
            return;
        }
        if self
            .current_path
            .clone_into_path(&mut self.scratch_path)
            .is_ok()
        {
            let transform = self.state.transform;
            let fill_rule = self.state.fill_rule;
            let color = self.state.paint;
            self.fill_scratch_path(transform, fill_rule, color);
        }
    }

    pub fn fill_current(&mut self) {
        if self.current_path.is_empty() {
            return;
        }
        if self
            .current_path
            .finish_into(&mut self.scratch_path)
            .is_ok()
        {
            let transform = self.state.transform;
            let fill_rule = self.state.fill_rule;
            let color = self.state.paint;
            self.fill_scratch_path(transform, fill_rule, color);
        }
    }

    pub fn stroke_preserve(&mut self) {
        if self.current_path.is_empty() {
            return;
        }
        if let Ok(path) = self.current_path.try_finish() {
            let style = self.stroke_style();
            self.stroke(&path, &style);
        }
    }

    pub fn stroke_current(&mut self) {
        self.stroke_preserve();
        self.current_path.clear();
    }

    // Paints the entire viewport with the active paint using SrcOver.
    pub fn paint(&mut self) {
        self.paint_with_alpha(1.0);
    }

    pub fn paint_with_alpha(&mut self, alpha: f32) {
        let a = clamp_finite_f32(alpha, 0.0, 1.0);
        if a <= 0.0 {
            return;
        }
        let color = self.state.paint.with_alpha(clamp_finite_f32(self.state.paint.a, 0.0, 1.0) * a);
        let Some((cmd_index, is_opaque)) = self.push_draw_color(color) else {
            return;
        };
        if is_opaque {
            self.clear(color);
            return;
        }
        self.binner.push_cmd_to_all_tiles(
            TileCmd {
                kind: TileCmdKind::SolidTile,
                flags: 0,
                aux: 0,
                cmd_index,
                head_chunk: NO_CHUNK,
                backdrop: 1,
            },
            &mut self.arena,
        );
    }

    pub fn path_extents(&self) -> Rect {
        if self.current_path.is_empty() {
            return Rect::EMPTY;
        }
        match self.current_path.try_finish() {
            Ok(p) => path_tight_bounds(&p),
            Err(_) => Rect::EMPTY,
        }
    }

    #[inline]
    pub fn user_to_device(&self, p: Point) -> Point {
        self.state.transform.transform_point(p)
    }

    #[inline]
    pub fn user_to_device_distance(&self, v: Vec2) -> Vec2 {
        self.state.transform.transform_vec(v)
    }

    #[inline]
    pub fn device_to_user(&self, p: Point) -> Option<Point> {
        self.state.transform.invert().map(|inv| inv.transform_point(p))
    }

    #[inline]
    pub fn device_to_user_distance(&self, v: Vec2) -> Option<Vec2> {
        self.state.transform.invert().map(|inv| inv.transform_vec(v))
    }

    pub fn copy_path(&self) -> Path {
        self.current_path.try_finish().unwrap_or_default()
    }

    pub fn copy_path_flat(&self) -> Path {
        let Ok(src) = self.current_path.try_finish() else {
            return Path::empty();
        };
        if !src.has_curves() {
            return src;
        }
        let mut pb = PathBuilder::new();
        let mut prev_end: Option<Point> = None;
        flatten_path_segments(&src, Affine2D::IDENTITY, self.state.tolerance, false, &mut |a, b| {
            if prev_end != Some(a) {
                pb.move_to(a.x, a.y);
            }
            pb.line_to(b.x, b.y);
            prev_end = Some(b);
        });
        pb.finish()
    }

    fn push_draw_color(&mut self, color: Color) -> Option<(u32, bool)> {
        if self.use_f32 {
            let lin = color.to_linear_f32();
            if lin.a <= 0.0 && lin.r == 0.0 && lin.g == 0.0 && lin.b == 0.0 {
                return None;
            }
            let idx = self.draw_colors_f32.len() as u32;
            self.draw_colors_f32.push(lin);
            Some((idx, lin.a >= 1.0))
        } else {
            let lin = color.to_linear_rgba16();
            if lin.a == 0 {
                return None;
            }
            let idx = self.draw_colors16.len() as u32;
            self.draw_colors16.push(lin);
            Some((idx, lin.a == 65535))
        }
    }

    fn fill_internal(
        &mut self,
        path: &Path,
        transform: Affine2D,
        fill_rule: FillRule,
        color: Color,
    ) {
        if path.is_empty() {
            return;
        }
        let w = self.surface.width();
        let h = self.surface.height();
        let range = flatten_path_for_fill(
            path,
            transform,
            w,
            h,
            self.state.tolerance,
            &mut self.arena,
        );
        if range.is_empty() {
            return;
        }
        let Some((cmd_index, is_opaque)) = self.push_draw_color(color) else {
            return;
        };
        let mut flags = 0u8;
        if is_opaque {
            flags |= TILE_FLAG_OPAQUE;
        }
        if self.state.antialias == AntialiasMode::None {
            flags |= TILE_FLAG_CRISP_EDGES;
        }
        self.binner
            .bin_path(range, fill_rule, cmd_index, flags, &mut self.arena);
    }

    fn fill_scratch_path(&mut self, transform: Affine2D, fill_rule: FillRule, color: Color) {
        if self.scratch_path.is_empty() {
            return;
        }
        let w = self.surface.width();
        let h = self.surface.height();
        let range = flatten_path_for_fill(
            &self.scratch_path,
            transform,
            w,
            h,
            self.state.tolerance,
            &mut self.arena,
        );
        if range.is_empty() {
            return;
        }
        let Some((cmd_index, is_opaque)) = self.push_draw_color(color) else {
            return;
        };
        let mut flags = 0u8;
        if is_opaque {
            flags |= TILE_FLAG_OPAQUE;
        }
        if self.state.antialias == AntialiasMode::None {
            flags |= TILE_FLAG_CRISP_EDGES;
        }
        self.binner
            .bin_path(range, fill_rule, cmd_index, flags, &mut self.arena);
    }

    // Composites all queued tile commands in a single L1-resident pass per tile.
    pub fn flush(&mut self) {
        let has_draws = if self.use_f32 {
            !self.draw_colors_f32.is_empty()
        } else {
            !self.draw_colors16.is_empty()
        };
        if self.pending_clear.is_none() && !has_draws {
            return;
        }

        let w = self.surface.width();
        let h = self.surface.height();
        let dither = self.state.dither_mode;

        if !has_draws && dither == DitherMode::None {
            if let Some(bg) = self.pending_clear.take() {
                self.surface.clear(bg);
            }
            self.arena.reset();
            self.binner.reset(w, h);
            return;
        }

        let tiles_x = self.binner.tiles_x();
        let tiles_y = self.binner.tiles_y();

        if self.use_f32 {
            let clear_lin = self.pending_clear.map(Color::to_linear_f32);
            for ty in 0..tiles_y {
                for tx in 0..tiles_x {
                    let tid = self.binner.tile_index(tx, ty);
                    let has_cmds = self.binner.tile_has_cmds(tid);
                    if !has_cmds && clear_lin.is_none() {
                        continue;
                    }
                    let tw = ((w as usize) - tx * TILE_SIZE).min(TILE_SIZE);
                    let th = ((h as usize) - ty * TILE_SIZE).min(TILE_SIZE);

                    if let Some(bg) = clear_lin {
                        self.tile_buf_f32.clear_rect(tw, th, bg);
                    } else {
                        let first_opaque_solid = self
                            .binner
                            .tile_cmds(tid, &self.arena)
                            .next()
                            .is_some_and(|c| {
                                c.kind == TileCmdKind::SolidTile
                                    && (c.flags & TILE_FLAG_OPAQUE) != 0
                            });
                        if !first_opaque_solid {
                            load_tile_f32(self.surface.as_ref(), tx, ty, &mut self.tile_buf_f32);
                        }
                    }

                    for cmd in self.binner.tile_cmds(tid, &self.arena) {
                        let src = self.draw_colors_f32[cmd.cmd_index as usize];
                        match cmd.kind {
                            TileCmdKind::Clear => self.tile_buf_f32.clear_rect(tw, th, src),
                            TileCmdKind::SolidTile => {
                                fill_solid_tile_src_over_f32(&mut self.tile_buf_f32, tw, th, src);
                            }
                            TileCmdKind::PathFill => {
                                let params = TileFillParams::from_cmd(cmd, tw, th);
                                let tile_buf = &mut self.tile_buf_f32;
                                self.scratch.rasterize_tile_spans(
                                    params,
                                    &self.arena,
                                    |span| match span {
                                        CoverageSpan::Solid { y, x0, x1 } => {
                                            fill_solid_span_src_over_f32(
                                                &mut tile_buf.row_mut(y)[x0..x1],
                                                src,
                                            );
                                        }
                                        CoverageSpan::Fractional { y, x0, cov } => {
                                            fill_cov_span_src_over_f32(
                                                &mut tile_buf.row_mut(y)[x0..x0 + cov.len()],
                                                cov,
                                                src,
                                            );
                                        }
                                    },
                                );
                            }
                            _ => {}
                        }
                    }
                    store_tile_f32(&self.tile_buf_f32, &mut self.surface, tx, ty, dither);
                }
            }
        } else {
            let clear_lin = self.pending_clear.map(Color::to_linear_rgba16);
            for ty in 0..tiles_y {
                for tx in 0..tiles_x {
                    let tid = self.binner.tile_index(tx, ty);
                    let has_cmds = self.binner.tile_has_cmds(tid);
                    if !has_cmds && clear_lin.is_none() {
                        continue;
                    }
                    let tw = ((w as usize) - tx * TILE_SIZE).min(TILE_SIZE);
                    let th = ((h as usize) - ty * TILE_SIZE).min(TILE_SIZE);

                    if let Some(bg) = clear_lin {
                        self.tile_buf16.clear_rect(tw, th, bg);
                    } else {
                        let first_opaque_solid = self
                            .binner
                            .tile_cmds(tid, &self.arena)
                            .next()
                            .is_some_and(|c| {
                                c.kind == TileCmdKind::SolidTile
                                    && (c.flags & TILE_FLAG_OPAQUE) != 0
                            });
                        if !first_opaque_solid {
                            load_tile_u16(self.surface.as_ref(), tx, ty, &mut self.tile_buf16);
                        }
                    }

                    for cmd in self.binner.tile_cmds(tid, &self.arena) {
                        let src = self.draw_colors16[cmd.cmd_index as usize];
                        match cmd.kind {
                            TileCmdKind::Clear => self.tile_buf16.clear_rect(tw, th, src),
                            TileCmdKind::SolidTile => {
                                fill_solid_tile_src_over_u16(&mut self.tile_buf16, tw, th, src);
                            }
                            TileCmdKind::PathFill => {
                                let params = TileFillParams::from_cmd(cmd, tw, th);
                                let tile_buf = &mut self.tile_buf16;
                                self.scratch.rasterize_tile_spans(
                                    params,
                                    &self.arena,
                                    |span| match span {
                                        CoverageSpan::Solid { y, x0, x1 } => {
                                            fill_solid_span_src_over_u16(
                                                &mut tile_buf.row_mut(y)[x0..x1],
                                                src,
                                            );
                                        }
                                        CoverageSpan::Fractional { y, x0, cov } => {
                                            fill_cov_span_src_over_u16(
                                                &mut tile_buf.row_mut(y)[x0..x0 + cov.len()],
                                                cov,
                                                src,
                                            );
                                        }
                                    },
                                );
                            }
                            _ => {}
                        }
                    }
                    store_tile_u16(&self.tile_buf16, &mut self.surface, tx, ty, dither);
                }
            }
        }

        self.pending_clear = None;
        self.draw_colors16.clear();
        self.draw_colors_f32.clear();
        self.arena.reset();
        self.binner.reset(w, h);
    }

    #[inline]
    pub fn finish(mut self) {
        self.flush();
    }
}

impl Drop for Context<'_> {
    fn drop(&mut self) {
        self.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::float::abs_f32;

    #[test]
    fn cairo_stateful_api_matches_path_builder_bit_for_bit() {
        let mut s_cairo = Surface::new_rgba8(128, 128).unwrap();
        {
            let mut ctx = Context::new(&mut s_cairo);
            ctx.set_dither_mode(DitherMode::BlueNoise);
            ctx.clear(Color::from_hex(0x101820));

            ctx.save();
            ctx.translate(10.0, 12.0);
            ctx.scale(1.25, 1.25);

            ctx.set_paint(Color::from_hex(0xFF6600));
            ctx.move_to(20.0, 20.0);
            ctx.rel_line_to(40.0, 0.0);
            ctx.rel_line_to(-10.0, 35.0);
            ctx.close_path();
            ctx.fill_preserve();

            ctx.set_paint(Color::new(0.2, 0.7, 1.0, 0.5));
            ctx.fill_current();

            ctx.arc(48.0, 52.0, 22.0, 0.0, core::f32::consts::PI * 1.5);
            ctx.line_to(48.0, 52.0);
            ctx.close_path();
            ctx.set_paint(Color::from_hex(0x33CC66));
            ctx.fill_current();
            ctx.restore();
        }

        let mut s_modern = Surface::new_rgba8(128, 128).unwrap();
        {
            let mut ctx = Context::new(&mut s_modern);
            ctx.set_dither_mode(DitherMode::BlueNoise);
            ctx.clear(Color::from_hex(0x101820));

            ctx.save();
            ctx.translate(10.0, 12.0);
            ctx.scale(1.25, 1.25);

            let mut pb1 = PathBuilder::new();
            pb1.move_to(20.0, 20.0)
                .rel_line_to(40.0, 0.0)
                .rel_line_to(-10.0, 35.0)
                .close();
            let p1 = pb1.finish();
            ctx.fill_with_paint(&p1, FillRule::NonZero, Color::from_hex(0xFF6600));
            ctx.fill_with_paint(&p1, FillRule::NonZero, Color::new(0.2, 0.7, 1.0, 0.5));

            let mut pb2 = PathBuilder::new();
            pb2.arc_center(48.0, 52.0, 22.0, 0.0, core::f32::consts::PI * 1.5, false)
                .line_to(48.0, 52.0)
                .close();
            let p2 = pb2.finish();
            ctx.fill_with_paint(&p2, FillRule::NonZero, Color::from_hex(0x33CC66));
            ctx.restore();
        }

        assert_eq!(s_cairo.data(), s_modern.data());
    }

    #[test]
    fn save_restore_extents_transforms_and_copy_path_flat() {
        let mut surf = Surface::new_rgba8(64, 64).unwrap();
        let mut ctx = Context::new(&mut surf);

        ctx.save();
        ctx.translate(10.0, 20.0);
        ctx.scale(2.0, 4.0);
        let dev = ctx.user_to_device(Point::new(5.0, 3.0));
        assert_eq!(dev, Point::new(20.0, 32.0));
        let usr = ctx.device_to_user(dev).unwrap();
        assert!(abs_f32(usr.x - 5.0) < 1e-5 && abs_f32(usr.y - 3.0) < 1e-5);
        assert!(ctx.restore());
        assert!(!ctx.restore());
        assert_eq!(ctx.transform(), Affine2D::IDENTITY);

        ctx.move_to(0.0, 0.0);
        ctx.curve_to(0.0, 20.0, 40.0, 20.0, 40.0, 0.0);
        let ext = ctx.path_extents();
        assert!(abs_f32(ext.min_x) < 1e-4 && abs_f32(ext.max_x - 40.0) < 1e-4);
        assert!(abs_f32(ext.min_y) < 1e-4 && abs_f32(ext.max_y - 15.0) < 1e-3);

        let copied = ctx.copy_path();
        assert!(copied.has_curves());
        let flat = ctx.copy_path_flat();
        assert!(!flat.has_curves());
        assert!(flat.points().len() > 4);
    }

    #[test]
    fn paint_with_alpha_and_basic_stroke() {
        let mut surf = Surface::new_rgba8(64, 64).unwrap();
        {
            let mut ctx = Context::new(&mut surf);
            ctx.set_dither_mode(DitherMode::None);
            ctx.clear(Color::BLACK);
            ctx.set_paint(Color::WHITE);
            ctx.paint_with_alpha(0.5);

            ctx.set_paint(Color::RED);
            ctx.set_line_width(4.0);
            ctx.move_to(8.0, 32.0);
            ctx.line_to(56.0, 32.0);
            ctx.stroke_current();
        }
        // Background pixel (2, 2): 50% linear white over black -> ~188 sRGB
        let bg = &surf.row(2).unwrap()[2 * 4..2 * 4 + 4];
        assert!(bg[0] >= 186 && bg[0] <= 190, "bg={bg:?}");
        // Stroke center pixel (32, 32): pure red
        let st = &surf.row(32).unwrap()[32 * 4..32 * 4 + 4];
        assert_eq!(st, &[255, 0, 0, 255]);
    }
}
