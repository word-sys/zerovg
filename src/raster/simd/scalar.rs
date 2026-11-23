#![forbid(unsafe_code)]

use crate::math::arena::TILE_SIZE;
use crate::math::color::{
    ColorSpace, LinearRgba16, LinearRgbaF32, linear_srgb_to_display_p3, linear_u16_to_srgb_8_8,
    mul_u16, u8_to_u16_linear, u16_to_u8_linear, unpremultiply_u16,
};
use crate::math::dither::{
    DitherMode, dither_threshold, encode_linear_rgba16_srgb_dithered, quantize_srgb_8_8,
};
use crate::math::float::{clamp_finite_f32, round_ties_even_f32};
use crate::surface::{
    PixelFormat, SurfaceMut, SurfaceRef, decode_pixel_to_linear_f32, decode_pixel_to_linear16,
    encode_linear_f32_to_pixel, encode_linear16_to_pixel,
};

use super::{TileBuffer16, TileBufferF32};

#[inline]
fn tile_bounds(width: u32, height: u32, tx: usize, ty: usize) -> Option<(usize, usize, usize, usize)> {
    let ox = tx * TILE_SIZE;
    let oy = ty * TILE_SIZE;
    let w = width as usize;
    let h = height as usize;
    if ox >= w || oy >= h {
        return None;
    }
    Some((ox, oy, (w - ox).min(TILE_SIZE), (h - oy).min(TILE_SIZE)))
}

pub fn load_tile_u16(
    surface: SurfaceRef<'_>,
    tx: usize,
    ty: usize,
    tile_buf: &mut TileBuffer16,
) {
    let Some((ox, oy, tw, th)) = tile_bounds(surface.width(), surface.height(), tx, ty) else {
        return;
    };
    let format = surface.format();
    let color_space = surface.color_space();
    let stride = surface.stride();
    let bpp = format.bytes_per_pixel();
    let data = surface.data();
    let is_p3 = matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3);

    for py in 0..th {
        let src_row_off = (oy + py) * stride + ox * bpp;
        let src_row = &data[src_row_off..src_row_off + tw * bpp];
        let dst_row = &mut tile_buf.row_mut(py)[..tw];

        if is_p3 || matches!(format, PixelFormat::RgbaF32Linear) {
            for (px_bytes, dst) in src_row.chunks_exact(bpp).zip(dst_row.iter_mut()) {
                *dst = decode_pixel_to_linear16(px_bytes, format, color_space);
            }
            continue;
        }

        match format {
            PixelFormat::Rgba8Srgb => {
                for (px, dst) in src_row.chunks_exact(4).zip(dst_row.iter_mut()) {
                    *dst = LinearRgba16::from_srgb_u8(px[0], px[1], px[2], px[3]);
                }
            }
            PixelFormat::Bgra8Srgb => {
                for (px, dst) in src_row.chunks_exact(4).zip(dst_row.iter_mut()) {
                    *dst = LinearRgba16::from_srgb_u8(px[2], px[1], px[0], px[3]);
                }
            }
            PixelFormat::Rgba8Linear => {
                for (px, dst) in src_row.chunks_exact(4).zip(dst_row.iter_mut()) {
                    let a = u8_to_u16_linear(px[3]);
                    *dst = LinearRgba16::new(
                        mul_u16(u8_to_u16_linear(px[0]), a),
                        mul_u16(u8_to_u16_linear(px[1]), a),
                        mul_u16(u8_to_u16_linear(px[2]), a),
                        a,
                    );
                }
            }
            PixelFormat::Rgba16Linear => {
                for (px, dst) in src_row.chunks_exact(8).zip(dst_row.iter_mut()) {
                    *dst = LinearRgba16::new(
                        u16::from_le_bytes([px[0], px[1]]),
                        u16::from_le_bytes([px[2], px[3]]),
                        u16::from_le_bytes([px[4], px[5]]),
                        u16::from_le_bytes([px[6], px[7]]),
                    );
                }
            }
            PixelFormat::RgbaF32Linear => unreachable!(),
        }
    }
}

pub fn load_tile_f32(
    surface: SurfaceRef<'_>,
    tx: usize,
    ty: usize,
    tile_buf: &mut TileBufferF32,
) {
    let Some((ox, oy, tw, th)) = tile_bounds(surface.width(), surface.height(), tx, ty) else {
        return;
    };
    let format = surface.format();
    let color_space = surface.color_space();
    let stride = surface.stride();
    let bpp = format.bytes_per_pixel();
    let data = surface.data();

    for py in 0..th {
        let src_row_off = (oy + py) * stride + ox * bpp;
        let src_row = &data[src_row_off..src_row_off + tw * bpp];
        let dst_row = &mut tile_buf.row_mut(py)[..tw];
        for (px_bytes, dst) in src_row.chunks_exact(bpp).zip(dst_row.iter_mut()) {
            *dst = decode_pixel_to_linear_f32(px_bytes, format, color_space);
        }
    }
}

#[inline]
pub fn fill_solid_span_src_over_u16(dst: &mut [LinearRgba16], src: LinearRgba16) {
    if src.a == 0 {
        return;
    }
    if src.a == 65535 {
        dst.fill(src);
        return;
    }
    let inv_a = 65535 - src.a;
    for px in dst.iter_mut() {
        px.r = src.r.saturating_add(mul_u16(px.r, inv_a));
        px.g = src.g.saturating_add(mul_u16(px.g, inv_a));
        px.b = src.b.saturating_add(mul_u16(px.b, inv_a));
        px.a = src.a.saturating_add(mul_u16(px.a, inv_a));
    }
}

#[inline]
pub fn fill_solid_tile_src_over_u16(
    tile_buf: &mut TileBuffer16,
    tw: usize,
    th: usize,
    src: LinearRgba16,
) {
    let w = tw.min(TILE_SIZE);
    let h = th.min(TILE_SIZE);
    if w == TILE_SIZE {
        fill_solid_span_src_over_u16(&mut tile_buf.pixels[..h * TILE_SIZE], src);
    } else {
        for py in 0..h {
            fill_solid_span_src_over_u16(&mut tile_buf.row_mut(py)[..w], src);
        }
    }
}

#[inline]
pub fn fill_cov_span_src_over_u16(dst: &mut [LinearRgba16], cov: &[u16], src: LinearRgba16) {
    if src.a == 0 {
        return;
    }
    let len = dst.len().min(cov.len());
    if src.a == 65535 {
        for (px, &c) in dst[..len].iter_mut().zip(cov[..len].iter()) {
            if c == 0 {
                continue;
            }
            if c == 65535 {
                *px = src;
                continue;
            }
            let inv_c = 65535 - c;
            px.r = mul_u16(src.r, c).saturating_add(mul_u16(px.r, inv_c));
            px.g = mul_u16(src.g, c).saturating_add(mul_u16(px.g, inv_c));
            px.b = mul_u16(src.b, c).saturating_add(mul_u16(px.b, inv_c));
            px.a = c.saturating_add(mul_u16(px.a, inv_c));
        }
    } else {
        for (px, &c) in dst[..len].iter_mut().zip(cov[..len].iter()) {
            if c == 0 {
                continue;
            }
            let s = src.scale_coverage(c);
            *px = s.src_over(*px);
        }
    }
}

#[inline]
pub fn fill_solid_span_src_over_f32(dst: &mut [LinearRgbaF32], src: LinearRgbaF32) {
    if src.a <= 0.0 && src.r == 0.0 && src.g == 0.0 && src.b == 0.0 {
        return;
    }
    if src.a >= 1.0 {
        dst.fill(src);
        return;
    }
    let inv_a = 1.0 - src.a;
    for px in dst.iter_mut() {
        px.r = src.r + px.r * inv_a;
        px.g = src.g + px.g * inv_a;
        px.b = src.b + px.b * inv_a;
        px.a = src.a + px.a * inv_a;
    }
}

#[inline]
pub fn fill_solid_tile_src_over_f32(
    tile_buf: &mut TileBufferF32,
    tw: usize,
    th: usize,
    src: LinearRgbaF32,
) {
    let w = tw.min(TILE_SIZE);
    let h = th.min(TILE_SIZE);
    if w == TILE_SIZE {
        fill_solid_span_src_over_f32(&mut tile_buf.pixels[..h * TILE_SIZE], src);
    } else {
        for py in 0..h {
            fill_solid_span_src_over_f32(&mut tile_buf.row_mut(py)[..w], src);
        }
    }
}

#[inline]
pub fn fill_cov_span_src_over_f32(dst: &mut [LinearRgbaF32], cov: &[u16], src: LinearRgbaF32) {
    if src.a <= 0.0 && src.r == 0.0 && src.g == 0.0 && src.b == 0.0 {
        return;
    }
    const INV65535: f32 = 1.0 / 65535.0;
    let len = dst.len().min(cov.len());
    for (px, &c) in dst[..len].iter_mut().zip(cov[..len].iter()) {
        if c == 0 {
            continue;
        }
        if c == 65535 {
            *px = src.src_over(*px);
            continue;
        }
        let cf = (c as f32) * INV65535;
        let s = LinearRgbaF32::new(src.r * cf, src.g * cf, src.b * cf, src.a * cf);
        *px = s.src_over(*px);
    }
}

pub fn store_tile_u16(
    tile_buf: &TileBuffer16,
    surface: &mut SurfaceMut<'_>,
    tx: usize,
    ty: usize,
    dither_mode: DitherMode,
) {
    let Some((ox, oy, tw, th)) = tile_bounds(surface.width(), surface.height(), tx, ty) else {
        return;
    };
    let format = surface.format();
    let color_space = surface.color_space();
    let stride = surface.stride();
    let bpp = format.bytes_per_pixel();
    let data = surface.data_mut();
    let is_p3 = matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3);

    for py in 0..th {
        let gy = (oy + py) as u32;
        let dst_row_off = (oy + py) * stride + ox * bpp;
        let dst_row = &mut data[dst_row_off..dst_row_off + tw * bpp];
        let src_row = &tile_buf.row(py)[..tw];

        if is_p3 || matches!(format, PixelFormat::RgbaF32Linear) {
            for (px, dst_bytes) in src_row.iter().zip(dst_row.chunks_exact_mut(bpp)) {
                encode_linear16_to_pixel(*px, dst_bytes, format, color_space);
            }
            continue;
        }

        match format {
            PixelFormat::Rgba8Srgb => {
                for (x, (&px, dst)) in src_row
                    .iter()
                    .zip(dst_row.chunks_exact_mut(4))
                    .enumerate()
                {
                    let s = encode_linear_rgba16_srgb_dithered(
                        px,
                        dither_mode,
                        (ox + x) as u32,
                        gy,
                    );
                    dst.copy_from_slice(&s);
                }
            }
            PixelFormat::Bgra8Srgb => {
                for (x, (&px, dst)) in src_row
                    .iter()
                    .zip(dst_row.chunks_exact_mut(4))
                    .enumerate()
                {
                    let s = encode_linear_rgba16_srgb_dithered(
                        px,
                        dither_mode,
                        (ox + x) as u32,
                        gy,
                    );
                    dst[0] = s[2];
                    dst[1] = s[1];
                    dst[2] = s[0];
                    dst[3] = s[3];
                }
            }
            PixelFormat::Rgba8Linear => {
                for (x, (&px, dst)) in src_row
                    .iter()
                    .zip(dst_row.chunks_exact_mut(4))
                    .enumerate()
                {
                    if px.a == 0 {
                        dst.copy_from_slice(&[0, 0, 0, 0]);
                    } else if dither_mode == DitherMode::None {
                        if px.a == 65535 {
                            dst[0] = u16_to_u8_linear(px.r);
                            dst[1] = u16_to_u8_linear(px.g);
                            dst[2] = u16_to_u8_linear(px.b);
                            dst[3] = 255;
                        } else {
                            dst[0] = u16_to_u8_linear(unpremultiply_u16(px.r, px.a));
                            dst[1] = u16_to_u8_linear(unpremultiply_u16(px.g, px.a));
                            dst[2] = u16_to_u8_linear(unpremultiply_u16(px.b, px.a));
                            dst[3] = u16_to_u8_linear(px.a);
                        }
                    } else {
                        let t = dither_threshold(dither_mode, (ox + x) as u32, gy);
                        let (ur, ug, ub, ua) = if px.a == 65535 {
                            (px.r, px.g, px.b, 255u8)
                        } else {
                            (
                                unpremultiply_u16(px.r, px.a),
                                unpremultiply_u16(px.g, px.a),
                                unpremultiply_u16(px.b, px.a),
                                u16_to_u8_linear(px.a),
                            )
                        };
                        let r88 = (((ur as u32) * 65280 + 32767) / 65535) as u16;
                        let g88 = (((ug as u32) * 65280 + 32767) / 65535) as u16;
                        let b88 = (((ub as u32) * 65280 + 32767) / 65535) as u16;
                        dst[0] = quantize_srgb_8_8(r88, t);
                        dst[1] = quantize_srgb_8_8(g88, t);
                        dst[2] = quantize_srgb_8_8(b88, t);
                        dst[3] = ua;
                    }
                }
            }
            PixelFormat::Rgba16Linear => {
                for (&px, dst) in src_row.iter().zip(dst_row.chunks_exact_mut(8)) {
                    dst[0..2].copy_from_slice(&px.r.to_le_bytes());
                    dst[2..4].copy_from_slice(&px.g.to_le_bytes());
                    dst[4..6].copy_from_slice(&px.b.to_le_bytes());
                    dst[6..8].copy_from_slice(&px.a.to_le_bytes());
                }
            }
            PixelFormat::RgbaF32Linear => unreachable!(),
        }
    }
}

pub fn store_tile_f32(
    tile_buf: &TileBufferF32,
    surface: &mut SurfaceMut<'_>,
    tx: usize,
    ty: usize,
    dither_mode: DitherMode,
) {
    let Some((ox, oy, tw, th)) = tile_bounds(surface.width(), surface.height(), tx, ty) else {
        return;
    };
    let format = surface.format();
    let color_space = surface.color_space();
    let stride = surface.stride();
    let bpp = format.bytes_per_pixel();
    let data = surface.data_mut();
    let is_p3 = matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3);

    for py in 0..th {
        let gy = (oy + py) as u32;
        let dst_row_off = (oy + py) * stride + ox * bpp;
        let dst_row = &mut data[dst_row_off..dst_row_off + tw * bpp];
        let src_row = &tile_buf.row(py)[..tw];

        match format {
            PixelFormat::Rgba8Srgb | PixelFormat::Bgra8Srgb => {
                for (x, (&px, dst)) in src_row
                    .iter()
                    .zip(dst_row.chunks_exact_mut(4))
                    .enumerate()
                {
                    if px.a <= 0.0 {
                        dst.copy_from_slice(&[0, 0, 0, 0]);
                        continue;
                    }
                    let (r, g, b) = if is_p3 {
                        linear_srgb_to_display_p3(px.r, px.g, px.b)
                    } else {
                        (px.r, px.g, px.b)
                    };
                    let inv_a = 1.0 / px.a;
                    let ur = round_ties_even_f32(clamp_finite_f32(r * inv_a, 0.0, 1.0) * 65535.0)
                        as u16;
                    let ug = round_ties_even_f32(clamp_finite_f32(g * inv_a, 0.0, 1.0) * 65535.0)
                        as u16;
                    let ub = round_ties_even_f32(clamp_finite_f32(b * inv_a, 0.0, 1.0) * 65535.0)
                        as u16;
                    let ua = round_ties_even_f32(clamp_finite_f32(px.a, 0.0, 1.0) * 255.0) as u8;
                    let t = dither_threshold(dither_mode, (ox + x) as u32, gy);
                    let sr = quantize_srgb_8_8(linear_u16_to_srgb_8_8(ur), t);
                    let sg = quantize_srgb_8_8(linear_u16_to_srgb_8_8(ug), t);
                    let sb = quantize_srgb_8_8(linear_u16_to_srgb_8_8(ub), t);
                    if matches!(format, PixelFormat::Rgba8Srgb) {
                        dst[0] = sr;
                        dst[1] = sg;
                        dst[2] = sb;
                        dst[3] = ua;
                    } else {
                        dst[0] = sb;
                        dst[1] = sg;
                        dst[2] = sr;
                        dst[3] = ua;
                    }
                }
            }
            PixelFormat::Rgba8Linear => {
                for (&px, dst) in src_row.iter().zip(dst_row.chunks_exact_mut(4)) {
                    let (r, g, b) = if is_p3 {
                        linear_srgb_to_display_p3(px.r, px.g, px.b)
                    } else {
                        (px.r, px.g, px.b)
                    };
                    if px.a <= 0.0 {
                        dst.copy_from_slice(&[0, 0, 0, 0]);
                    } else {
                        let inv_a = 1.0 / px.a;
                        dst[0] = round_ties_even_f32(clamp_finite_f32(r * inv_a, 0.0, 1.0) * 255.0)
                            as u8;
                        dst[1] = round_ties_even_f32(clamp_finite_f32(g * inv_a, 0.0, 1.0) * 255.0)
                            as u8;
                        dst[2] = round_ties_even_f32(clamp_finite_f32(b * inv_a, 0.0, 1.0) * 255.0)
                            as u8;
                        dst[3] = round_ties_even_f32(clamp_finite_f32(px.a, 0.0, 1.0) * 255.0)
                            as u8;
                    }
                }
            }
            PixelFormat::Rgba16Linear | PixelFormat::RgbaF32Linear => {
                for (&px, dst) in src_row.iter().zip(dst_row.chunks_exact_mut(bpp)) {
                    encode_linear_f32_to_pixel(px, dst, format, color_space);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raster::surface_uses_f32_pipeline;
    use crate::geom::flatten::{DEFAULT_TOLERANCE, flatten_path_for_fill};
    use crate::geom::{FillRule, Path, PathBuilder};
    use crate::math::affine::{Affine2D, Rect};
    use crate::math::arena::{FrameArena, NO_CHUNK, TileCmd, TileCmdKind};
    use crate::math::color::Color;
    use crate::math::float::abs_f32;
    use crate::raster::{AntialiasMode, CoverageSpan, TileFillParams, TileRasterScratch};
    use crate::surface::Surface;
    use crate::tile::TileBinner;

    fn render_commands_to_surface(
        surface: &mut Surface,
        clear_color: Option<Color>,
        draws: &[(&Path, Color, FillRule, AntialiasMode)],
        dither_mode: DitherMode,
    ) {
        let w = surface.width();
        let h = surface.height();
        let mut arena = FrameArena::new();
        let mut binner = TileBinner::new();
        let mut scratch = TileRasterScratch::new();
        binner.reset(w, h);

        let mut palette = alloc::vec::Vec::new();
        if let Some(c) = clear_color {
            let cmd_index = palette.len() as u32;
            palette.push(c);
            binner.push_cmd_to_all_tiles(
                TileCmd {
                    kind: TileCmdKind::Clear,
                    flags: 0,
                    aux: 0,
                    cmd_index,
                    head_chunk: NO_CHUNK,
                    backdrop: 0,
                },
                &mut arena,
            );
        }

        for &(path, color, rule, aa) in draws {
            let cmd_index = palette.len() as u32;
            palette.push(color);
            let flags = if aa == AntialiasMode::None {
                crate::tile::TILE_FLAG_CRISP_EDGES
            } else {
                0
            };
            let range =
                flatten_path_for_fill(path, Affine2D::IDENTITY, w, h, DEFAULT_TOLERANCE, &mut arena);
            binner.bin_path(range, rule, cmd_index, flags, &mut arena);
        }

        let use_f32 = surface_uses_f32_pipeline(surface.format(), surface.color_space());
        let mut surf_mut = surface.as_mut();

        if use_f32 {
            let mut tile_buf = TileBufferF32::new();
            for ty in 0..binner.tiles_y() {
                for tx in 0..binner.tiles_x() {
                    let tid = binner.tile_index(tx, ty);
                    if !binner.tile_has_cmds(tid) {
                        continue;
                    }
                    let tw = ((w as usize) - tx * TILE_SIZE).min(TILE_SIZE);
                    let th = ((h as usize) - ty * TILE_SIZE).min(TILE_SIZE);
                    if clear_color.is_none() {
                        load_tile_f32(surf_mut.as_ref(), tx, ty, &mut tile_buf);
                    }
                    for cmd in binner.tile_cmds(tid, &arena) {
                        let src = palette[cmd.cmd_index as usize].to_linear_f32();
                        match cmd.kind {
                            TileCmdKind::Clear => tile_buf.clear_rect(tw, th, src),
                            TileCmdKind::SolidTile => {
                                fill_solid_tile_src_over_f32(&mut tile_buf, tw, th, src);
                            }
                            TileCmdKind::PathFill => {
                                let params = TileFillParams::from_cmd(cmd, tw, th);
                                scratch.rasterize_tile_spans(params, &arena, |span| match span {
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
                                });
                            }
                            _ => {}
                        }
                    }
                    store_tile_f32(&tile_buf, &mut surf_mut, tx, ty, dither_mode);
                }
            }
        } else {
            let mut tile_buf = TileBuffer16::new();
            for ty in 0..binner.tiles_y() {
                for tx in 0..binner.tiles_x() {
                    let tid = binner.tile_index(tx, ty);
                    if !binner.tile_has_cmds(tid) {
                        continue;
                    }
                    let tw = ((w as usize) - tx * TILE_SIZE).min(TILE_SIZE);
                    let th = ((h as usize) - ty * TILE_SIZE).min(TILE_SIZE);
                    if clear_color.is_none() {
                        load_tile_u16(surf_mut.as_ref(), tx, ty, &mut tile_buf);
                    }
                    for cmd in binner.tile_cmds(tid, &arena) {
                        let src = palette[cmd.cmd_index as usize].to_linear_rgba16();
                        match cmd.kind {
                            TileCmdKind::Clear => tile_buf.clear_rect(tw, th, src),
                            TileCmdKind::SolidTile => {
                                fill_solid_tile_src_over_u16(&mut tile_buf, tw, th, src);
                            }
                            TileCmdKind::PathFill => {
                                let params = TileFillParams::from_cmd(cmd, tw, th);
                                scratch.rasterize_tile_spans(params, &arena, |span| match span {
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
                                });
                            }
                            _ => {}
                        }
                    }
                    store_tile_u16(&tile_buf, &mut surf_mut, tx, ty, dither_mode);
                }
            }
        }
    }

    #[test]
    fn red_over_green_antialiased_edge_has_bright_linear_yellow_seam() {
        // Fill background with pure green (0, 255, 0), then draw red rect [0, 0]..[16.5, 32]
        // At column x = 16 (50% coverage), linear-light blend produces (0.5, 0.5, 0) in linear RGB
        // which maps to ~188 sRGB (bright clean yellow), NOT gamma-darkened (128, 128, 0)!
        let mut surface = Surface::new_rgba8(32, 32).unwrap();
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(0.0, 0.0, 16.5, 32.0));
        let red_half = pb.finish();

        render_commands_to_surface(
            &mut surface,
            Some(Color::GREEN),
            &[(&red_half, Color::RED, FillRule::NonZero, AntialiasMode::Analytic)],
            DitherMode::None,
        );

        let row = surface.row(16).unwrap();
        // x = 10: pure red
        assert_eq!(&row[10 * 4..10 * 4 + 4], &[255, 0, 0, 255]);
        // x = 20: pure green
        assert_eq!(&row[20 * 4..20 * 4 + 4], &[0, 255, 0, 255]);
        // x = 16: 50% red over green in linear light -> (~188, ~188, 0, 255)
        let seam = &row[16 * 4..16 * 4 + 4];
        assert!(
            seam[0] >= 186 && seam[0] <= 190 && seam[1] >= 186 && seam[1] <= 190,
            "expected bright linear yellow (~188, ~188, 0, 255), got {seam:?}"
        );
        assert_eq!(seam[2], 0);
        assert_eq!(seam[3], 255);
    }

    #[test]
    fn optical_balance_white_on_black_vs_black_on_white_thin_strokes() {
        // Thin 0.5px horizontal bar y in [10.25, 10.75] (exact 50% linear coverage on row 10):
        // White bar on black background vs Black bar on white background must have identical
        // linear energy (L_white_on_black + L_black_on_white == 1.0 in linear luminance!).
        let mut bar_pb = PathBuilder::new();
        bar_pb.push_rect(Rect::new(4.0, 10.25, 28.0, 10.75));
        let bar = bar_pb.finish();

        let mut surf_w_on_b = Surface::new_rgba8(32, 32).unwrap();
        render_commands_to_surface(
            &mut surf_w_on_b,
            Some(Color::BLACK),
            &[(&bar, Color::WHITE, FillRule::NonZero, AntialiasMode::Analytic)],
            DitherMode::None,
        );

        let mut surf_b_on_w = Surface::new_rgba8(32, 32).unwrap();
        render_commands_to_surface(
            &mut surf_b_on_w,
            Some(Color::WHITE),
            &[(&bar, Color::BLACK, FillRule::NonZero, AntialiasMode::Analytic)],
            DitherMode::None,
        );

        let px_wb = surf_w_on_b.pixel_at_f32(16, 10).unwrap();
        let px_bw = surf_b_on_w.pixel_at_f32(16, 10).unwrap();
        assert!(
            abs_f32(px_wb.r - 0.5) < 0.015 && abs_f32(px_bw.r - 0.5) < 0.015,
            "wb={} bw={}",
            px_wb.r,
            px_bw.r
        );
        assert!(abs_f32((px_wb.r + px_bw.r) - 1.0) < 0.01);
    }

    #[test]
    fn display_p3_wide_gamut_and_hdr_f32_preservation() {
        // 1. Display P3 pure P3 red (1, 0, 0) round-trip through linear sRGB working space
        let mut p3_surf =
            Surface::try_new(32, 32, PixelFormat::Rgba8Srgb, ColorSpace::DisplayP3).unwrap();
        let mut pb = PathBuilder::new();
        pb.push_rect(Rect::new(2.0, 2.0, 30.0, 30.0));
        let rect = pb.finish();

        let p3_red = Color::from_display_p3(1.0, 0.0, 0.0, 1.0);
        render_commands_to_surface(
            &mut p3_surf,
            Some(Color::BLACK),
            &[(&rect, p3_red, FillRule::NonZero, AntialiasMode::Analytic)],
            DitherMode::None,
        );
        let row = p3_surf.row(16).unwrap();
        assert_eq!(&row[16 * 4..16 * 4 + 4], &[255, 0, 0, 255]);

        // Reload tile from Display P3 surface and verify working-space linear sRGB has R > 1.22 (super-sRGB primary)
        let mut f32_buf = TileBufferF32::new();
        load_tile_f32(p3_surf.as_ref(), 0, 0, &mut f32_buf);
        let loaded = f32_buf.row(16)[16];
        assert!(loaded.r > 1.22 && loaded.g < -0.04, "loaded={loaded:?}");

        // 2. HDR RgbaF32Linear surface preserving emissive channels > 1.0 (e.g. R = 4.5, G = 2.25)
        let mut hdr_surf =
            Surface::try_new(32, 32, PixelFormat::RgbaF32Linear, ColorSpace::LinearSrgb).unwrap();
        let hdr_color = Color::from_linear_rgba(4.5, 2.25, 0.125, 1.0);
        render_commands_to_surface(
            &mut hdr_surf,
            Some(Color::BLACK),
            &[(&rect, hdr_color, FillRule::NonZero, AntialiasMode::Analytic)],
            DitherMode::None,
        );
        let hdr_px = hdr_surf.pixel_at_f32(16, 16).unwrap();
        assert!(abs_f32(hdr_px.r - 4.5) < 1e-4, "hdr_px={hdr_px:?}");
        assert!(abs_f32(hdr_px.g - 2.25) < 1e-4, "hdr_px={hdr_px:?}");
        assert!(abs_f32(hdr_px.b - 0.125) < 1e-4, "hdr_px={hdr_px:?}");
        assert!(abs_f32(hdr_px.a - 1.0) < 1e-5);
    }

    #[test]
    fn non_tile_aligned_framebuffers_and_sub_surface_padding_safety() {
        // Test non-tile-aligned sizes required by PLAN.md: 1x1, 31x33, 137x91, 1920x1080
        for (w, h) in [(1u32, 1u32), (31, 33), (137, 91), (1920, 1080)] {
            let mut surf = Surface::new_rgba8(w, h).unwrap();
            let mut pb = PathBuilder::new();
            pb.push_rect(Rect::new(0.0, 0.0, w as f32, h as f32));
            let full = pb.finish();

            render_commands_to_surface(
                &mut surf,
                Some(Color::BLACK),
                &[(&full, Color::RED, FillRule::NonZero, AntialiasMode::Analytic)],
                DitherMode::BlueNoise,
            );

            assert_eq!(&surf.row(0).unwrap()[0..4], &[255, 0, 0, 255]);
            let last_row = surf.row(h - 1).unwrap();
            let last_off = ((w - 1) as usize) * 4;
            assert_eq!(&last_row[last_off..last_off + 4], &[255, 0, 0, 255]);
        }

        // Verify padded stride borrowed SurfaceMut (31x33 inside a 40x36 canary buffer)
        // never overwrites a single sentinel byte outside the 31x33 region!
        let stride = 40 * 4;
        let mut raw = alloc::vec![0xAAu8; stride * 36];
        {
            let mut borrowed = SurfaceMut::from_slice(
                &mut raw,
                31,
                33,
                stride,
                PixelFormat::Bgra8Srgb,
                ColorSpace::Srgb,
            )
            .unwrap();
            let mut tile_buf = TileBuffer16::new();
            tile_buf.clear(Color::RED.to_linear_rgba16());
            for ty in 0..2 {
                for tx in 0..2 {
                    store_tile_u16(&tile_buf, &mut borrowed, tx, ty, DitherMode::None);
                }
            }
        }
        // Check pixels inside 31x33 are BGRA red [0, 0, 255, 255] and padding bytes remain 0xAA
        for y in 0..36usize {
            for x in 0..40usize {
                let off = y * stride + x * 4;
                let px = &raw[off..off + 4];
                if x < 31 && y < 33 {
                    assert_eq!(px, &[0, 0, 255, 255], "at ({x},{y})");
                } else {
                    assert_eq!(px, &[0xAA, 0xAA, 0xAA, 0xAA], "canary overwritten at ({x},{y})");
                }
            }
        }
    }

    #[test]
    fn load_blend_store_across_all_pixel_formats() {
        for (format, cs) in [
            (PixelFormat::Rgba8Srgb, ColorSpace::Srgb),
            (PixelFormat::Bgra8Srgb, ColorSpace::Srgb),
            (PixelFormat::Rgba8Linear, ColorSpace::LinearSrgb),
            (PixelFormat::Rgba16Linear, ColorSpace::LinearSrgb),
            (PixelFormat::RgbaF32Linear, ColorSpace::LinearSrgb),
        ] {
            let mut surf = Surface::try_new(32, 32, format, cs).unwrap();
            surf.clear(Color::BLUE);

            let mut pb = PathBuilder::new();
            pb.push_rect(Rect::new(4.0, 4.0, 28.0, 28.0));
            let rect = pb.finish();

            // Render without clearing so load_tile_u16 / load_tile_f32 loads existing BLUE background
            render_commands_to_surface(
                &mut surf,
                None,
                &[(
                    &rect,
                    Color::RED.with_alpha(0.5),
                    FillRule::NonZero,
                    AntialiasMode::Analytic,
                )],
                DitherMode::None,
            );

            let out_bg = surf.pixel_at_f32(1, 1).unwrap();
            assert!(out_bg.b > 0.99 && out_bg.r < 0.01);

            let out_blend = surf.pixel_at_f32(16, 16).unwrap();
            assert!(
                abs_f32(out_blend.r - 0.5) < 0.02 && abs_f32(out_blend.b - 0.5) < 0.02,
                "format={format:?} blend={out_blend:?}"
            );
        }
    }
}
