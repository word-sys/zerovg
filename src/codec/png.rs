#![forbid(unsafe_code)]

use alloc::vec;
use alloc::vec::Vec;

use crate::codec::deflate::{CompressionLevel, Crc32, zlib_compress, zlib_decompress};
use crate::error::{Result, ZvgError};
use crate::math::color::ColorSpace;
use crate::surface::{PixelFormat, Surface, SurfaceRef, decode_pixel_to_linear16};

const PNG_SIG: [u8; 8] = [137, 80, 78, 71, 13, 10, 26, 10];

const COLOR_GRAY: u8 = 0;
const COLOR_RGB: u8 = 2;
const COLOR_INDEXED: u8 = 3;
const COLOR_GRAY_ALPHA: u8 = 4;
const COLOR_RGBA: u8 = 6;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PngEncodeOptions {
    pub compression: CompressionLevel,
    pub strip_opaque_alpha: bool,
}

impl Default for PngEncodeOptions {
    fn default() -> Self {
        Self {
            compression: CompressionLevel::Balanced,
            strip_opaque_alpha: false,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PngDecodeLimits {
    pub max_width: u32,
    pub max_height: u32,
    pub max_pixels: usize,
    pub max_decompressed_bytes: usize,
}

impl Default for PngDecodeLimits {
    fn default() -> Self {
        Self {
            max_width: 16_384,
            max_height: 16_384,
            max_pixels: 64_000_000,
            max_decompressed_bytes: 256 * 1024 * 1024,
        }
    }
}

#[inline]
fn paeth(a: u8, b: u8, c: u8) -> u8 {
    let a = a as i32;
    let b = b as i32;
    let c = c as i32;
    let p = a + b - c;
    let pa = (p - a).abs();
    let pb = (p - b).abs();
    let pc = (p - c).abs();
    if pa <= pb && pa <= pc {
        a as u8
    } else if pb <= pc {
        b as u8
    } else {
        c as u8
    }
}

fn write_chunk(out: &mut Vec<u8>, chunk_type: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    out.extend_from_slice(chunk_type);
    out.extend_from_slice(data);
    let mut crc = Crc32::new();
    crc.update(chunk_type);
    crc.update(data);
    out.extend_from_slice(&crc.finalize().to_be_bytes());
}

fn filter_row_adaptive(cur: &[u8], prev: &[u8], bpp: usize, out: &mut Vec<u8>, scratch: &mut [Vec<u8>; 5]) {
    let len = cur.len();
    for buf in scratch.iter_mut() {
        buf.clear();
        buf.resize(len, 0);
    }

    // 0: None
    scratch[0].copy_from_slice(cur);
    // 1: Sub
    for i in 0..len {
        let a = if i >= bpp { cur[i - bpp] } else { 0 };
        scratch[1][i] = cur[i].wrapping_sub(a);
    }
    // 2: Up
    for i in 0..len {
        scratch[2][i] = cur[i].wrapping_sub(prev[i]);
    }
    // 3: Average
    for i in 0..len {
        let a = if i >= bpp { cur[i - bpp] as u16 } else { 0 };
        let b = prev[i] as u16;
        scratch[3][i] = cur[i].wrapping_sub(((a + b) >> 1) as u8);
    }
    // 4: Paeth
    for i in 0..len {
        let a = if i >= bpp { cur[i - bpp] } else { 0 };
        let b = prev[i];
        let c = if i >= bpp { prev[i - bpp] } else { 0 };
        scratch[4][i] = cur[i].wrapping_sub(paeth(a, b, c));
    }

    let mut best_filter = 0usize;
    let mut best_score = u64::MAX;
    for (f, cand) in scratch.iter().enumerate() {
        let mut score = 0u64;
        for &byte in cand {
            score += (byte as i8 as i32).unsigned_abs() as u64;
        }
        if score < best_score {
            best_score = score;
            best_filter = f;
        }
    }

    out.push(best_filter as u8);
    out.extend_from_slice(&scratch[best_filter]);
}

pub fn encode_png_raw(
    pixels: &[u8],
    width: u32,
    height: u32,
    color_type: u8,
    compression: CompressionLevel,
) -> Result<Vec<u8>> {
    if width == 0 || height == 0 {
        return Err(ZvgError::InvalidArgument("png dimensions must be non-zero"));
    }
    let bpp: usize = match color_type {
        COLOR_GRAY => 1,
        COLOR_RGB => 3,
        COLOR_GRAY_ALPHA => 2,
        COLOR_RGBA => 4,
        _ => return Err(ZvgError::InvalidArgument("unsupported png encode color type")),
    };
    let row_bytes = (width as usize)
        .checked_mul(bpp)
        .ok_or(ZvgError::AllocationLimit)?;
    let expected = row_bytes
        .checked_mul(height as usize)
        .ok_or(ZvgError::AllocationLimit)?;
    if pixels.len() != expected {
        return Err(ZvgError::InvalidArgument("png pixel buffer length mismatch"));
    }

    let mut filtered = Vec::with_capacity((row_bytes + 1) * (height as usize));
    let mut prev_row = vec![0u8; row_bytes];
    let mut scratch: [Vec<u8>; 5] = [
        Vec::with_capacity(row_bytes),
        Vec::with_capacity(row_bytes),
        Vec::with_capacity(row_bytes),
        Vec::with_capacity(row_bytes),
        Vec::with_capacity(row_bytes),
    ];

    for y in 0..height as usize {
        let cur = &pixels[y * row_bytes..(y + 1) * row_bytes];
        filter_row_adaptive(cur, &prev_row, bpp, &mut filtered, &mut scratch);
        prev_row.copy_from_slice(cur);
    }

    let compressed = zlib_compress(&filtered, compression);

    let mut out = Vec::with_capacity(64 + compressed.len());
    out.extend_from_slice(&PNG_SIG);

    let mut ihdr = [0u8; 13];
    ihdr[0..4].copy_from_slice(&width.to_be_bytes());
    ihdr[4..8].copy_from_slice(&height.to_be_bytes());
    ihdr[8] = 8;
    ihdr[9] = color_type;
    ihdr[10] = 0;
    ihdr[11] = 0;
    ihdr[12] = 0;
    write_chunk(&mut out, b"IHDR", &ihdr);
    write_chunk(&mut out, b"sRGB", &[0]);
    write_chunk(&mut out, b"IDAT", &compressed);
    write_chunk(&mut out, b"IEND", &[]);
    Ok(out)
}

pub fn encode_png_with_options(surface: SurfaceRef<'_>, options: PngEncodeOptions) -> Result<Vec<u8>> {
    let w = surface.width();
    let h = surface.height();
    let fmt = surface.format();
    let cs = surface.color_space();
    let bpp_in = fmt.bytes_per_pixel();

    let mut all_opaque = options.strip_opaque_alpha;
    let mut rgba = Vec::with_capacity((w as usize) * (h as usize) * 4);
    for y in 0..h {
        let row = surface.row(y).ok_or(ZvgError::OutOfBounds)?;
        if fmt == PixelFormat::Rgba8Srgb && cs == ColorSpace::Srgb {
            if all_opaque {
                for px in row.chunks_exact(4) {
                    if px[3] != 255 {
                        all_opaque = false;
                        break;
                    }
                }
            }
            rgba.extend_from_slice(row);
        } else {
            for px in row.chunks_exact(bpp_in) {
                let s = decode_pixel_to_linear16(px, fmt, cs).to_srgb_u8();
                if s[3] != 255 {
                    all_opaque = false;
                }
                rgba.extend_from_slice(&s);
            }
        }
    }

    if all_opaque {
        let mut rgb = Vec::with_capacity((w as usize) * (h as usize) * 3);
        for px in rgba.chunks_exact(4) {
            rgb.extend_from_slice(&px[..3]);
        }
        encode_png_raw(&rgb, w, h, COLOR_RGB, options.compression)
    } else {
        encode_png_raw(&rgba, w, h, COLOR_RGBA, options.compression)
    }
}

pub fn encode_png(surface: SurfaceRef<'_>) -> Result<Vec<u8>> {
    encode_png_with_options(surface, PngEncodeOptions::default())
}

// ---------- PNG Decoder ----------

fn channels_for_color_type(ct: u8) -> Result<usize> {
    match ct {
        COLOR_GRAY => Ok(1),
        COLOR_RGB => Ok(3),
        COLOR_INDEXED => Ok(1),
        COLOR_GRAY_ALPHA => Ok(2),
        COLOR_RGBA => Ok(4),
        _ => Err(ZvgError::ParseError("invalid png color type")),
    }
}

fn validate_bit_depth(ct: u8, bd: u8) -> Result<()> {
    let ok = match ct {
        COLOR_GRAY => matches!(bd, 1 | 2 | 4 | 8 | 16),
        COLOR_RGB | COLOR_GRAY_ALPHA | COLOR_RGBA => matches!(bd, 8 | 16),
        COLOR_INDEXED => matches!(bd, 1 | 2 | 4 | 8),
        _ => false,
    };
    if !ok {
        return Err(ZvgError::ParseError("invalid png color type / bit depth"));
    }
    Ok(())
}

fn unfilter_row(filter: u8, raw: &[u8], prev: &[u8], bpp: usize, out: &mut [u8]) -> Result<()> {
    let len = raw.len();
    match filter {
        0 => out.copy_from_slice(raw),
        1 => {
            for i in 0..len {
                let a = if i >= bpp { out[i - bpp] } else { 0 };
                out[i] = raw[i].wrapping_add(a);
            }
        }
        2 => {
            for i in 0..len {
                out[i] = raw[i].wrapping_add(prev[i]);
            }
        }
        3 => {
            for i in 0..len {
                let a = if i >= bpp { out[i - bpp] as u16 } else { 0 };
                let b = prev[i] as u16;
                out[i] = raw[i].wrapping_add(((a + b) >> 1) as u8);
            }
        }
        4 => {
            for i in 0..len {
                let a = if i >= bpp { out[i - bpp] } else { 0 };
                let b = prev[i];
                let c = if i >= bpp { prev[i - bpp] } else { 0 };
                out[i] = raw[i].wrapping_add(paeth(a, b, c));
            }
        }
        _ => return Err(ZvgError::ParseError("invalid png filter type")),
    }
    Ok(())
}

#[inline]
fn scale_subbyte_gray(val: u8, bit_depth: u8) -> u8 {
    match bit_depth {
        1 => val * 255,
        2 => val * 85,
        4 => val * 17,
        _ => val,
    }
}

#[inline]
fn u16_be_to_u8(hi: u8, lo: u8) -> u8 {
    let v = u16::from_be_bytes([hi, lo]) as u32;
    ((v * 255 + 32768) / 65535) as u8
}

struct DecodeCtx<'a> {
    color_type: u8,
    bit_depth: u8,
    palette: Option<&'a [u8]>,
    trns: Option<&'a [u8]>,
}

fn unpack_scanline_pixel(ctx: &DecodeCtx<'_>, row: &[u8], x: usize) -> Result<[u8; 4]> {
    let bd = ctx.bit_depth;
    match ctx.color_type {
        COLOR_GRAY => {
            if bd < 8 {
                let ppb = (8 / bd) as usize;
                let byte = row[x / ppb];
                let shift = 8 - bd - ((x % ppb) as u8) * bd;
                let mask = (1u8 << bd) - 1;
                let raw_v = (byte >> shift) & mask;
                let alpha = if let Some(t) = ctx.trns {
                    if t.len() >= 2 && u16::from_be_bytes([t[0], t[1]]) == raw_v as u16 {
                        0
                    } else {
                        255
                    }
                } else {
                    255
                };
                let g = scale_subbyte_gray(raw_v, bd);
                Ok([g, g, g, alpha])
            } else if bd == 8 {
                let g = row[x];
                let alpha = if let Some(t) = ctx.trns {
                    if t.len() >= 2 && u16::from_be_bytes([t[0], t[1]]) == g as u16 {
                        0
                    } else {
                        255
                    }
                } else {
                    255
                };
                Ok([g, g, g, alpha])
            } else {
                let hi = row[x * 2];
                let lo = row[x * 2 + 1];
                let raw16 = u16::from_be_bytes([hi, lo]);
                let alpha = if let Some(t) = ctx.trns {
                    if t.len() >= 2 && u16::from_be_bytes([t[0], t[1]]) == raw16 {
                        0
                    } else {
                        255
                    }
                } else {
                    255
                };
                let g = u16_be_to_u8(hi, lo);
                Ok([g, g, g, alpha])
            }
        }
        COLOR_RGB => {
            if bd == 8 {
                let r = row[x * 3];
                let g = row[x * 3 + 1];
                let b = row[x * 3 + 2];
                let alpha = if let Some(t) = ctx.trns {
                    if t.len() >= 6
                        && u16::from_be_bytes([t[0], t[1]]) == r as u16
                        && u16::from_be_bytes([t[2], t[3]]) == g as u16
                        && u16::from_be_bytes([t[4], t[5]]) == b as u16
                    {
                        0
                    } else {
                        255
                    }
                } else {
                    255
                };
                Ok([r, g, b, alpha])
            } else {
                let o = x * 6;
                let r16 = u16::from_be_bytes([row[o], row[o + 1]]);
                let g16 = u16::from_be_bytes([row[o + 2], row[o + 3]]);
                let b16 = u16::from_be_bytes([row[o + 4], row[o + 5]]);
                let alpha = if let Some(t) = ctx.trns {
                    if t.len() >= 6
                        && u16::from_be_bytes([t[0], t[1]]) == r16
                        && u16::from_be_bytes([t[2], t[3]]) == g16
                        && u16::from_be_bytes([t[4], t[5]]) == b16
                    {
                        0
                    } else {
                        255
                    }
                } else {
                    255
                };
                Ok([
                    u16_be_to_u8(row[o], row[o + 1]),
                    u16_be_to_u8(row[o + 2], row[o + 3]),
                    u16_be_to_u8(row[o + 4], row[o + 5]),
                    alpha,
                ])
            }
        }
        COLOR_INDEXED => {
            let pal = ctx
                .palette
                .ok_or(ZvgError::ParseError("indexed png missing PLTE"))?;
            let idx = if bd < 8 {
                let ppb = (8 / bd) as usize;
                let byte = row[x / ppb];
                let shift = 8 - bd - ((x % ppb) as u8) * bd;
                let mask = (1u8 << bd) - 1;
                ((byte >> shift) & mask) as usize
            } else {
                row[x] as usize
            };
            let po = idx * 3;
            if po + 3 > pal.len() {
                return Err(ZvgError::ParseError("png palette index out of range"));
            }
            let alpha = ctx.trns.and_then(|t| t.get(idx).copied()).unwrap_or(255);
            Ok([pal[po], pal[po + 1], pal[po + 2], alpha])
        }
        COLOR_GRAY_ALPHA => {
            if bd == 8 {
                let g = row[x * 2];
                let a = row[x * 2 + 1];
                Ok([g, g, g, a])
            } else {
                let o = x * 4;
                let g = u16_be_to_u8(row[o], row[o + 1]);
                let a = u16_be_to_u8(row[o + 2], row[o + 3]);
                Ok([g, g, g, a])
            }
        }
        COLOR_RGBA => {
            if bd == 8 {
                let o = x * 4;
                Ok([row[o], row[o + 1], row[o + 2], row[o + 3]])
            } else {
                let o = x * 8;
                Ok([
                    u16_be_to_u8(row[o], row[o + 1]),
                    u16_be_to_u8(row[o + 2], row[o + 3]),
                    u16_be_to_u8(row[o + 4], row[o + 5]),
                    u16_be_to_u8(row[o + 6], row[o + 7]),
                ])
            }
        }
        _ => Err(ZvgError::ParseError("invalid png color type")),
    }
}

pub fn decode_png_with_limits(data: &[u8], limits: PngDecodeLimits) -> Result<Surface> {
    if data.len() < 8 || data[..8] != PNG_SIG {
        return Err(ZvgError::ParseError("invalid png signature"));
    }

    let mut pos = 8usize;
    let mut width = 0u32;
    let mut height = 0u32;
    let mut bit_depth = 0u8;
    let mut color_type = 0u8;
    let mut interlace = 0u8;
    let mut seen_ihdr = false;
    let mut seen_iend = false;

    let mut palette: Option<&[u8]> = None;
    let mut trns: Option<&[u8]> = None;
    let mut idat = Vec::new();

    while pos < data.len() {
        if pos + 12 > data.len() {
            return Err(ZvgError::ParseError("truncated png chunk header"));
        }
        let len = u32::from_be_bytes([data[pos], data[pos + 1], data[pos + 2], data[pos + 3]]) as usize;
        let ctype = &data[pos + 4..pos + 8];
        let payload_start = pos + 8;
        let payload_end = payload_start
            .checked_add(len)
            .ok_or(ZvgError::ParseError("png chunk length overflow"))?;
        let crc_end = payload_end
            .checked_add(4)
            .ok_or(ZvgError::ParseError("png chunk crc overflow"))?;
        if crc_end > data.len() {
            return Err(ZvgError::ParseError("truncated png chunk payload"));
        }
        let payload = &data[payload_start..payload_end];
        let expected_crc = u32::from_be_bytes([
            data[payload_end],
            data[payload_end + 1],
            data[payload_end + 2],
            data[payload_end + 3],
        ]);
        let mut crc = Crc32::new();
        crc.update(ctype);
        crc.update(payload);
        if crc.finalize() != expected_crc {
            return Err(ZvgError::ParseError("png chunk crc mismatch"));
        }
        pos = crc_end;

        if !seen_ihdr {
            if ctype != b"IHDR" || len != 13 {
                return Err(ZvgError::ParseError("png missing or malformed IHDR"));
            }
            width = u32::from_be_bytes([payload[0], payload[1], payload[2], payload[3]]);
            height = u32::from_be_bytes([payload[4], payload[5], payload[6], payload[7]]);
            bit_depth = payload[8];
            color_type = payload[9];
            let compression = payload[10];
            let filter_method = payload[11];
            interlace = payload[12];

            if width == 0 || height == 0 {
                return Err(ZvgError::ParseError("png zero dimensions"));
            }
            if width > limits.max_width || height > limits.max_height {
                return Err(ZvgError::AllocationLimit);
            }
            let pixels = (width as usize)
                .checked_mul(height as usize)
                .ok_or(ZvgError::AllocationLimit)?;
            if pixels > limits.max_pixels {
                return Err(ZvgError::AllocationLimit);
            }
            validate_bit_depth(color_type, bit_depth)?;
            if compression != 0 || filter_method != 0 || interlace > 1 {
                return Err(ZvgError::ParseError("unsupported png header parameters"));
            }
            seen_ihdr = true;
            continue;
        }

        match ctype {
            b"IHDR" => return Err(ZvgError::ParseError("duplicate png IHDR")),
            b"PLTE" => {
                if len == 0 || len % 3 != 0 || len > 768 {
                    return Err(ZvgError::ParseError("invalid png PLTE chunk"));
                }
                palette = Some(payload);
            }
            b"tRNS" => {
                trns = Some(payload);
            }
            b"IDAT" => {
                if idat.len().saturating_add(len) > limits.max_decompressed_bytes {
                    return Err(ZvgError::AllocationLimit);
                }
                idat.extend_from_slice(payload);
            }
            b"IEND" => {
                if len != 0 {
                    return Err(ZvgError::ParseError("non-empty png IEND"));
                }
                seen_iend = true;
                break;
            }
            _ => {
                // Unknown critical chunk (first letter uppercase) must be rejected per RFC 2083.
                if (ctype[0] & 0x20) == 0 {
                    return Err(ZvgError::UnsupportedFeature("unknown critical png chunk"));
                }
            }
        }
    }

    if !seen_ihdr || !seen_iend || idat.is_empty() {
        return Err(ZvgError::ParseError("incomplete png stream"));
    }
    if color_type == COLOR_INDEXED && palette.is_none() {
        return Err(ZvgError::ParseError("indexed png missing PLTE"));
    }

    let channels = channels_for_color_type(color_type)?;
    let bits_per_pixel = channels * (bit_depth as usize);
    let filter_bpp = bits_per_pixel.div_ceil(8).max(1);

    let decompressed = zlib_decompress(&idat, limits.max_decompressed_bytes)?;
    let mut surface = Surface::try_new(width, height, PixelFormat::Rgba8Srgb, ColorSpace::Srgb)?;
    let ctx = DecodeCtx {
        color_type,
        bit_depth,
        palette,
        trns,
    };

    if interlace == 0 {
        let row_bytes = (width as usize)
            .checked_mul(bits_per_pixel)
            .ok_or(ZvgError::AllocationLimit)?
            .div_ceil(8);
        let expected = (row_bytes + 1)
            .checked_mul(height as usize)
            .ok_or(ZvgError::AllocationLimit)?;
        if decompressed.len() < expected {
            return Err(ZvgError::ParseError("truncated png scanline data"));
        }

        let mut prev = vec![0u8; row_bytes];
        let mut cur = vec![0u8; row_bytes];
        let stride = surface.stride();
        let dst = surface.data_mut();

        for y in 0..height as usize {
            let off = y * (row_bytes + 1);
            let filter = decompressed[off];
            unfilter_row(
                filter,
                &decompressed[off + 1..off + 1 + row_bytes],
                &prev,
                filter_bpp,
                &mut cur,
            )?;
            let row_dst = &mut dst[y * stride..y * stride + (width as usize) * 4];
            for x in 0..width as usize {
                let px = unpack_scanline_pixel(&ctx, &cur, x)?;
                row_dst[x * 4..x * 4 + 4].copy_from_slice(&px);
            }
            prev.copy_from_slice(&cur);
        }
    } else {
        // Adam7 7-pass interlacing
        const X_START: [usize; 7] = [0, 4, 0, 2, 0, 1, 0];
        const Y_START: [usize; 7] = [0, 0, 4, 0, 2, 0, 1];
        const X_STEP: [usize; 7] = [8, 8, 4, 4, 2, 2, 1];
        const Y_STEP: [usize; 7] = [8, 8, 8, 4, 4, 2, 2];

        let w = width as usize;
        let h = height as usize;
        let stride = surface.stride();
        let dst = surface.data_mut();
        let mut off = 0usize;

        for pass in 0..7 {
            if w <= X_START[pass] || h <= Y_START[pass] {
                continue;
            }
            let pw = (w - X_START[pass]).div_ceil(X_STEP[pass]);
            let ph = (h - Y_START[pass]).div_ceil(Y_STEP[pass]);
            let row_bytes = (pw * bits_per_pixel).div_ceil(8);
            let pass_bytes = (row_bytes + 1)
                .checked_mul(ph)
                .ok_or(ZvgError::AllocationLimit)?;
            if off + pass_bytes > decompressed.len() {
                return Err(ZvgError::ParseError("truncated adam7 png data"));
            }

            let mut prev = vec![0u8; row_bytes];
            let mut cur = vec![0u8; row_bytes];
            for py in 0..ph {
                let filter = decompressed[off];
                unfilter_row(
                    filter,
                    &decompressed[off + 1..off + 1 + row_bytes],
                    &prev,
                    filter_bpp,
                    &mut cur,
                )?;
                off += 1 + row_bytes;
                let dy = Y_START[pass] + py * Y_STEP[pass];
                for px_i in 0..pw {
                    let dx = X_START[pass] + px_i * X_STEP[pass];
                    let px = unpack_scanline_pixel(&ctx, &cur, px_i)?;
                    let doff = dy * stride + dx * 4;
                    dst[doff..doff + 4].copy_from_slice(&px);
                }
                prev.copy_from_slice(&cur);
            }
        }
    }

    Ok(surface)
}

pub fn decode_png(data: &[u8]) -> Result<Surface> {
    decode_png_with_limits(data, PngDecodeLimits::default())
}

impl Surface {
    pub fn encode_png(&self) -> Result<Vec<u8>> {
        encode_png(self.as_ref())
    }

    pub fn from_png_bytes(data: &[u8]) -> Result<Self> {
        decode_png(data)
    }

    #[cfg(feature = "std")]
    pub fn save_png<P: AsRef<std::path::Path>>(&self, path: P) -> Result<()> {
        let bytes = self.encode_png()?;
        std::fs::write(path, bytes)?;
        Ok(())
    }

    #[cfg(feature = "std")]
    pub fn load_png<P: AsRef<std::path::Path>>(path: P) -> Result<Self> {
        let bytes = std::fs::read(path)?;
        Self::from_png_bytes(&bytes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct SyntheticPng<'a> {
        w: u32,
        h: u32,
        bit_depth: u8,
        color_type: u8,
        interlace: u8,
        plte: Option<&'a [u8]>,
        trns: Option<&'a [u8]>,
        scanlines: &'a [u8],
        split_idat: bool,
    }

    fn build_synthetic_png(spec: SyntheticPng<'_>) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&PNG_SIG);
        let mut ihdr = [0u8; 13];
        ihdr[0..4].copy_from_slice(&spec.w.to_be_bytes());
        ihdr[4..8].copy_from_slice(&spec.h.to_be_bytes());
        ihdr[8] = spec.bit_depth;
        ihdr[9] = spec.color_type;
        ihdr[12] = spec.interlace;
        write_chunk(&mut out, b"IHDR", &ihdr);
        if let Some(p) = spec.plte {
            write_chunk(&mut out, b"PLTE", p);
        }
        if let Some(t) = spec.trns {
            write_chunk(&mut out, b"tRNS", t);
        }
        let z = zlib_compress(spec.scanlines, CompressionLevel::Balanced);
        if spec.split_idat && z.len() > 6 {
            let mid = z.len() / 2;
            write_chunk(&mut out, b"IDAT", &z[..mid]);
            write_chunk(&mut out, b"IDAT", &z[mid..]);
        } else {
            write_chunk(&mut out, b"IDAT", &z);
        }
        write_chunk(&mut out, b"IEND", &[]);
        out
    }

    #[test]
    fn png_rgba_and_rgb_roundtrip_exact() {
        let mut surf = Surface::new_rgba8(64, 48).unwrap();
        let stride = surf.stride();
        let data = surf.data_mut();
        for y in 0..48usize {
            for x in 0..64usize {
                let o = y * stride + x * 4;
                data[o] = (x * 4) as u8;
                data[o + 1] = (y * 5) as u8;
                data[o + 2] = ((x ^ y) * 3) as u8;
                data[o + 3] = if (x + y) % 7 == 0 { 128 } else { 255 };
            }
        }

        let png = surf.encode_png().unwrap();
        let decoded = Surface::from_png_bytes(&png).unwrap();
        assert_eq!(decoded.width(), 64);
        assert_eq!(decoded.height(), 48);
        assert_eq!(decoded.data(), surf.data());

        // Opaque RGB auto-strip mode
        let mut opaque = Surface::new_rgba8(32, 24).unwrap();
        for px in opaque.data_mut().chunks_exact_mut(4) {
            px.copy_from_slice(&[12, 180, 240, 255]);
        }
        let opts = PngEncodeOptions {
            compression: CompressionLevel::Balanced,
            strip_opaque_alpha: true,
        };
        let png_rgb = encode_png_with_options(opaque.as_ref(), opts).unwrap();
        // Verify IHDR color type is RGB (2)
        assert_eq!(png_rgb[25], COLOR_RGB);
        let dec_rgb = decode_png(&png_rgb).unwrap();
        assert_eq!(dec_rgb.data(), opaque.data());
    }

    #[test]
    fn png_all_color_types_bit_depths_and_trns() {
        // 1. 1-bit Grayscale (4x1: 1, 0, 1, 0 -> 0b1010_0000 = 0xA0)
        let g1 = build_synthetic_png(SyntheticPng {
            w: 4,
            h: 1,
            bit_depth: 1,
            color_type: COLOR_GRAY,
            interlace: 0,
            plte: None,
            trns: None,
            scanlines: &[0, 0xA0],
            split_idat: false,
        });
        let s = decode_png(&g1).unwrap();
        assert_eq!(
            s.data(),
            &[255, 255, 255, 255, 0, 0, 0, 255, 255, 255, 255, 255, 0, 0, 0, 255]
        );

        // 2. 4-bit Indexed + PLTE + tRNS + split multi-IDAT (2x1: idx 0, idx 1 -> 0x01)
        let plte = [255, 0, 0, 0, 255, 0];
        let trns = [128u8, 255];
        let idx4 = build_synthetic_png(SyntheticPng {
            w: 2,
            h: 1,
            bit_depth: 4,
            color_type: COLOR_INDEXED,
            interlace: 0,
            plte: Some(&plte),
            trns: Some(&trns),
            scanlines: &[0, 0x01],
            split_idat: true,
        });
        let s = decode_png(&idx4).unwrap();
        assert_eq!(s.data(), &[255, 0, 0, 128, 0, 255, 0, 255]);

        // 3. 16-bit RGBA (2x1)
        let mut raw16 = vec![0u8]; // filter None
        raw16.extend_from_slice(&0xFFFFu16.to_be_bytes());
        raw16.extend_from_slice(&0x0000u16.to_be_bytes());
        raw16.extend_from_slice(&0x8080u16.to_be_bytes());
        raw16.extend_from_slice(&0xFFFFu16.to_be_bytes());
        raw16.extend_from_slice(&0x0000u16.to_be_bytes());
        raw16.extend_from_slice(&0xFFFFu16.to_be_bytes());
        raw16.extend_from_slice(&0x0000u16.to_be_bytes());
        raw16.extend_from_slice(&0x8080u16.to_be_bytes());
        let rgba16 = build_synthetic_png(SyntheticPng {
            w: 2,
            h: 1,
            bit_depth: 16,
            color_type: COLOR_RGBA,
            interlace: 0,
            plte: None,
            trns: None,
            scanlines: &raw16,
            split_idat: false,
        });
        let s = decode_png(&rgba16).unwrap();
        assert_eq!(s.data(), &[255, 0, 128, 255, 0, 255, 0, 128]);

        // 4. 8-bit RGB with tRNS color-key transparency
        let rgb_trns = build_synthetic_png(SyntheticPng {
            w: 2,
            h: 1,
            bit_depth: 8,
            color_type: COLOR_RGB,
            interlace: 0,
            plte: None,
            trns: Some(&[0, 10, 0, 20, 0, 30]),
            scanlines: &[0, 10, 20, 30, 40, 50, 60],
            split_idat: false,
        });
        let s = decode_png(&rgb_trns).unwrap();
        assert_eq!(s.data(), &[10, 20, 30, 0, 40, 50, 60, 255]);
    }

    #[test]
    fn png_adam7_interlaced_decode() {
        // 5x3 RGBA8 Adam7 image with unique pixel value per coordinate
        let w = 5usize;
        let h = 3usize;
        let mut expected = vec![0u8; w * h * 4];
        for y in 0..h {
            for x in 0..w {
                let o = (y * w + x) * 4;
                expected[o] = (x * 40 + 10) as u8;
                expected[o + 1] = (y * 70 + 20) as u8;
                expected[o + 2] = ((x + y) * 30) as u8;
                expected[o + 3] = 255;
            }
        }

        const X_START: [usize; 7] = [0, 4, 0, 2, 0, 1, 0];
        const Y_START: [usize; 7] = [0, 0, 4, 0, 2, 0, 1];
        const X_STEP: [usize; 7] = [8, 8, 4, 4, 2, 2, 1];
        const Y_STEP: [usize; 7] = [8, 8, 8, 4, 4, 2, 2];

        let mut scanlines = Vec::new();
        for pass in 0..7 {
            if w <= X_START[pass] || h <= Y_START[pass] {
                continue;
            }
            let pw = (w - X_START[pass]).div_ceil(X_STEP[pass]);
            let ph = (h - Y_START[pass]).div_ceil(Y_STEP[pass]);
            for py in 0..ph {
                scanlines.push(0); // filter None
                let dy = Y_START[pass] + py * Y_STEP[pass];
                for px in 0..pw {
                    let dx = X_START[pass] + px * X_STEP[pass];
                    let o = (dy * w + dx) * 4;
                    scanlines.extend_from_slice(&expected[o..o + 4]);
                }
            }
        }

        let png = build_synthetic_png(SyntheticPng {
            w: w as u32,
            h: h as u32,
            bit_depth: 8,
            color_type: COLOR_RGBA,
            interlace: 1,
            plte: None,
            trns: None,
            scanlines: &scanlines,
            split_idat: true,
        });
        let dec = decode_png(&png).unwrap();
        assert_eq!(dec.data(), &expected);
    }

    #[test]
    fn png_malformed_and_crc_errors_rejected() {
        assert!(decode_png(b"not a png").is_err());

        let surf = Surface::new_rgba8(8, 8).unwrap();
        let valid = surf.encode_png().unwrap();

        // Corrupt a single byte in IHDR CRC
        let mut bad_crc = valid.clone();
        bad_crc[29] ^= 0x80;
        assert!(decode_png(&bad_crc).is_err());

        // Truncated streams never panic
        for cut in 0..valid.len() {
            assert!(decode_png(&valid[..cut]).is_err());
        }

        // Limits enforcement
        let tight = PngDecodeLimits {
            max_width: 4,
            max_height: 4,
            ..PngDecodeLimits::default()
        };
        assert_eq!(
            decode_png_with_limits(&valid, tight).unwrap_err(),
            ZvgError::AllocationLimit
        );
    }
}
