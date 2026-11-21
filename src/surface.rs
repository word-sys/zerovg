#![forbid(unsafe_code)]

use alloc::vec;
use alloc::vec::Vec;

use crate::error::{Result, ZvgError};
use crate::math::affine::IRect;
use crate::math::color::{
    Color, ColorSpace, LinearRgba16, LinearRgbaF32, linear_display_p3_to_srgb,
    linear_srgb_to_display_p3, linear_to_srgb_f32, mul_u16, srgb_to_linear_f32, u8_to_u16_linear,
    u16_to_u8_linear, unpremultiply_u16,
};
use crate::math::float::{clamp_finite_f32, round_ties_even_f32};

pub const MAX_SURFACE_DIM: u32 = 32_768;
pub const MAX_SURFACE_BYTES: usize = 512 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub enum PixelFormat {
    #[default]
    Rgba8Srgb,
    Bgra8Srgb,
    Rgba8Linear,
    Rgba16Linear,
    RgbaF32Linear,
}

impl PixelFormat {
    #[inline]
    pub const fn bytes_per_pixel(self) -> usize {
        match self {
            Self::Rgba8Srgb | Self::Bgra8Srgb | Self::Rgba8Linear => 4,
            Self::Rgba16Linear => 8,
            Self::RgbaF32Linear => 16,
        }
    }

    #[inline]
    pub const fn default_color_space(self) -> ColorSpace {
        match self {
            Self::Rgba8Srgb | Self::Bgra8Srgb => ColorSpace::Srgb,
            Self::Rgba8Linear | Self::Rgba16Linear | Self::RgbaF32Linear => ColorSpace::LinearSrgb,
        }
    }

    #[inline]
    pub const fn is_hdr_f32(self) -> bool {
        matches!(self, Self::RgbaF32Linear)
    }
}

fn validate_dims(width: u32, height: u32) -> Result<()> {
    if width == 0 || height == 0 {
        return Err(ZvgError::InvalidArgument("surface dimensions must be non-zero"));
    }
    if width > MAX_SURFACE_DIM || height > MAX_SURFACE_DIM {
        return Err(ZvgError::AllocationLimit);
    }
    Ok(())
}

fn required_borrowed_len(width: u32, height: u32, stride: usize, bpp: usize) -> Result<usize> {
    validate_dims(width, height)?;
    let min_stride = (width as usize)
        .checked_mul(bpp)
        .ok_or(ZvgError::AllocationLimit)?;
    if stride < min_stride {
        return Err(ZvgError::InvalidArgument("stride smaller than row width"));
    }
    let last_row_offset = ((height - 1) as usize)
        .checked_mul(stride)
        .ok_or(ZvgError::AllocationLimit)?;
    last_row_offset
        .checked_add(min_stride)
        .ok_or(ZvgError::AllocationLimit)
}

#[inline]
pub fn decode_pixel_to_linear16(
    px: &[u8],
    format: PixelFormat,
    color_space: ColorSpace,
) -> LinearRgba16 {
    let is_p3 = matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3);
    if is_p3 {
        return decode_pixel_to_linear_f32(px, format, color_space).to_linear_rgba16();
    }
    match format {
        PixelFormat::Rgba8Srgb => LinearRgba16::from_srgb_u8(px[0], px[1], px[2], px[3]),
        PixelFormat::Bgra8Srgb => LinearRgba16::from_srgb_u8(px[2], px[1], px[0], px[3]),
        PixelFormat::Rgba8Linear => {
            let a = u8_to_u16_linear(px[3]);
            LinearRgba16::new(
                mul_u16(u8_to_u16_linear(px[0]), a),
                mul_u16(u8_to_u16_linear(px[1]), a),
                mul_u16(u8_to_u16_linear(px[2]), a),
                a,
            )
        }
        PixelFormat::Rgba16Linear => LinearRgba16::new(
            u16::from_le_bytes([px[0], px[1]]),
            u16::from_le_bytes([px[2], px[3]]),
            u16::from_le_bytes([px[4], px[5]]),
            u16::from_le_bytes([px[6], px[7]]),
        ),
        PixelFormat::RgbaF32Linear => {
            decode_pixel_to_linear_f32(px, format, color_space).to_linear_rgba16()
        }
    }
}

pub fn decode_pixel_to_linear_f32(
    px: &[u8],
    format: PixelFormat,
    color_space: ColorSpace,
) -> LinearRgbaF32 {
    const INV255: f32 = 1.0 / 255.0;
    const INV65535: f32 = 1.0 / 65535.0;
    let (mut r, mut g, mut b, a) = match format {
        PixelFormat::Rgba8Srgb => {
            let a = (px[3] as f32) * INV255;
            let r = srgb_to_linear_f32((px[0] as f32) * INV255) * a;
            let g = srgb_to_linear_f32((px[1] as f32) * INV255) * a;
            let b = srgb_to_linear_f32((px[2] as f32) * INV255) * a;
            (r, g, b, a)
        }
        PixelFormat::Bgra8Srgb => {
            let a = (px[3] as f32) * INV255;
            let r = srgb_to_linear_f32((px[2] as f32) * INV255) * a;
            let g = srgb_to_linear_f32((px[1] as f32) * INV255) * a;
            let b = srgb_to_linear_f32((px[0] as f32) * INV255) * a;
            (r, g, b, a)
        }
        PixelFormat::Rgba8Linear => {
            let a = (px[3] as f32) * INV255;
            (
                (px[0] as f32) * INV255 * a,
                (px[1] as f32) * INV255 * a,
                (px[2] as f32) * INV255 * a,
                a,
            )
        }
        PixelFormat::Rgba16Linear => (
            (u16::from_le_bytes([px[0], px[1]]) as f32) * INV65535,
            (u16::from_le_bytes([px[2], px[3]]) as f32) * INV65535,
            (u16::from_le_bytes([px[4], px[5]]) as f32) * INV65535,
            (u16::from_le_bytes([px[6], px[7]]) as f32) * INV65535,
        ),
        PixelFormat::RgbaF32Linear => (
            f32::from_le_bytes([px[0], px[1], px[2], px[3]]),
            f32::from_le_bytes([px[4], px[5], px[6], px[7]]),
            f32::from_le_bytes([px[8], px[9], px[10], px[11]]),
            f32::from_le_bytes([px[12], px[13], px[14], px[15]]),
        ),
    };
    if matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3) {
        (r, g, b) = linear_display_p3_to_srgb(r, g, b);
    }
    LinearRgbaF32::new(r, g, b, a)
}

#[inline]
pub fn encode_linear16_to_pixel(
    c: LinearRgba16,
    dst: &mut [u8],
    format: PixelFormat,
    color_space: ColorSpace,
) {
    let is_p3 = matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3);
    if is_p3 || matches!(format, PixelFormat::RgbaF32Linear) {
        encode_linear_f32_to_pixel(
            LinearRgbaF32::from_linear_rgba16(c),
            dst,
            format,
            color_space,
        );
        return;
    }
    match format {
        PixelFormat::Rgba8Srgb => {
            let s = c.to_srgb_u8();
            dst[..4].copy_from_slice(&s);
        }
        PixelFormat::Bgra8Srgb => {
            let s = c.to_srgb_u8();
            dst[0] = s[2];
            dst[1] = s[1];
            dst[2] = s[0];
            dst[3] = s[3];
        }
        PixelFormat::Rgba8Linear => {
            if c.a == 0 {
                dst[..4].copy_from_slice(&[0, 0, 0, 0]);
            } else if c.a == 65535 {
                dst[0] = u16_to_u8_linear(c.r);
                dst[1] = u16_to_u8_linear(c.g);
                dst[2] = u16_to_u8_linear(c.b);
                dst[3] = 255;
            } else {
                dst[0] = u16_to_u8_linear(unpremultiply_u16(c.r, c.a));
                dst[1] = u16_to_u8_linear(unpremultiply_u16(c.g, c.a));
                dst[2] = u16_to_u8_linear(unpremultiply_u16(c.b, c.a));
                dst[3] = u16_to_u8_linear(c.a);
            }
        }
        PixelFormat::Rgba16Linear => {
            dst[0..2].copy_from_slice(&c.r.to_le_bytes());
            dst[2..4].copy_from_slice(&c.g.to_le_bytes());
            dst[4..6].copy_from_slice(&c.b.to_le_bytes());
            dst[6..8].copy_from_slice(&c.a.to_le_bytes());
        }
        PixelFormat::RgbaF32Linear => unreachable!(),
    }
}

pub fn encode_linear_f32_to_pixel(
    c: LinearRgbaF32,
    dst: &mut [u8],
    format: PixelFormat,
    color_space: ColorSpace,
) {
    let (r, g, b) = if matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3) {
        linear_srgb_to_display_p3(c.r, c.g, c.b)
    } else {
        (c.r, c.g, c.b)
    };
    let a = c.a;

    match format {
        PixelFormat::Rgba8Srgb | PixelFormat::Bgra8Srgb => {
            let (ur, ug, ub, ua) = if a <= 0.0 {
                (0u8, 0u8, 0u8, 0u8)
            } else {
                let inv_a = 1.0 / a;
                let sr = linear_to_srgb_f32(clamp_finite_f32(r * inv_a, 0.0, 1.0));
                let sg = linear_to_srgb_f32(clamp_finite_f32(g * inv_a, 0.0, 1.0));
                let sb = linear_to_srgb_f32(clamp_finite_f32(b * inv_a, 0.0, 1.0));
                (
                    round_ties_even_f32(sr * 255.0) as u8,
                    round_ties_even_f32(sg * 255.0) as u8,
                    round_ties_even_f32(sb * 255.0) as u8,
                    round_ties_even_f32(clamp_finite_f32(a, 0.0, 1.0) * 255.0) as u8,
                )
            };
            if matches!(format, PixelFormat::Rgba8Srgb) {
                dst[0] = ur;
                dst[1] = ug;
                dst[2] = ub;
                dst[3] = ua;
            } else {
                dst[0] = ub;
                dst[1] = ug;
                dst[2] = ur;
                dst[3] = ua;
            }
        }
        PixelFormat::Rgba8Linear => {
            if a <= 0.0 {
                dst[..4].copy_from_slice(&[0, 0, 0, 0]);
            } else {
                let inv_a = 1.0 / a;
                dst[0] = round_ties_even_f32(clamp_finite_f32(r * inv_a, 0.0, 1.0) * 255.0) as u8;
                dst[1] = round_ties_even_f32(clamp_finite_f32(g * inv_a, 0.0, 1.0) * 255.0) as u8;
                dst[2] = round_ties_even_f32(clamp_finite_f32(b * inv_a, 0.0, 1.0) * 255.0) as u8;
                dst[3] = round_ties_even_f32(clamp_finite_f32(a, 0.0, 1.0) * 255.0) as u8;
            }
        }
        PixelFormat::Rgba16Linear => {
            let q = LinearRgbaF32::new(r, g, b, a).to_linear_rgba16();
            dst[0..2].copy_from_slice(&q.r.to_le_bytes());
            dst[2..4].copy_from_slice(&q.g.to_le_bytes());
            dst[4..6].copy_from_slice(&q.b.to_le_bytes());
            dst[6..8].copy_from_slice(&q.a.to_le_bytes());
        }
        PixelFormat::RgbaF32Linear => {
            dst[0..4].copy_from_slice(&r.to_le_bytes());
            dst[4..8].copy_from_slice(&g.to_le_bytes());
            dst[8..12].copy_from_slice(&b.to_le_bytes());
            dst[12..16].copy_from_slice(&a.to_le_bytes());
        }
    }
}

#[derive(Debug, Clone)]
pub struct Surface {
    data: Vec<u8>,
    width: u32,
    height: u32,
    stride: usize,
    format: PixelFormat,
    color_space: ColorSpace,
    generation: u64,
}

impl Surface {
    pub fn try_new(
        width: u32,
        height: u32,
        format: PixelFormat,
        color_space: ColorSpace,
    ) -> Result<Self> {
        validate_dims(width, height)?;
        let bpp = format.bytes_per_pixel();
        let stride = (width as usize)
            .checked_mul(bpp)
            .ok_or(ZvgError::AllocationLimit)?;
        let total = (height as usize)
            .checked_mul(stride)
            .ok_or(ZvgError::AllocationLimit)?;
        if total > MAX_SURFACE_BYTES {
            return Err(ZvgError::AllocationLimit);
        }
        Ok(Self {
            data: vec![0u8; total],
            width,
            height,
            stride,
            format,
            color_space,
            generation: 1,
        })
    }

    pub fn new(width: u32, height: u32, color_space: ColorSpace) -> Result<Self> {
        let format = match color_space {
            ColorSpace::Srgb | ColorSpace::DisplayP3 => PixelFormat::Rgba8Srgb,
            ColorSpace::LinearSrgb | ColorSpace::LinearDisplayP3 => PixelFormat::Rgba8Linear,
        };
        Self::try_new(width, height, format, color_space)
    }

    pub fn new_rgba8(width: u32, height: u32) -> Result<Self> {
        Self::try_new(width, height, PixelFormat::Rgba8Srgb, ColorSpace::Srgb)
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
    pub fn stride(&self) -> usize {
        self.stride
    }

    #[inline]
    pub fn format(&self) -> PixelFormat {
        self.format
    }

    #[inline]
    pub fn color_space(&self) -> ColorSpace {
        self.color_space
    }

    #[inline]
    pub fn generation(&self) -> u64 {
        self.generation
    }

    #[inline]
    pub fn bump_generation(&mut self) {
        self.generation = self.generation.wrapping_add(1);
    }

    #[inline]
    pub fn data(&self) -> &[u8] {
        &self.data
    }

    #[inline]
    pub fn data_mut(&mut self) -> &mut [u8] {
        self.bump_generation();
        &mut self.data
    }

    #[inline]
    pub fn as_ref(&self) -> SurfaceRef<'_> {
        SurfaceRef {
            data: &self.data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        }
    }

    #[inline]
    pub fn as_mut(&mut self) -> SurfaceMut<'_> {
        self.bump_generation();
        SurfaceMut {
            data: &mut self.data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        }
    }

    #[inline]
    pub fn row(&self, y: u32) -> Option<&[u8]> {
        self.as_ref().row(y)
    }

    #[inline]
    pub fn row_mut(&mut self, y: u32) -> Option<&mut [u8]> {
        if y >= self.height {
            return None;
        }
        self.bump_generation();
        let row_bytes = (self.width as usize) * self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride;
        Some(&mut self.data[off..off + row_bytes])
    }

    #[inline]
    pub fn clear(&mut self, color: Color) {
        self.as_mut().clear(color);
    }

    #[inline]
    pub fn pixel_at(&self, x: u32, y: u32) -> Option<LinearRgba16> {
        self.as_ref().pixel_at(x, y)
    }

    #[inline]
    pub fn pixel_at_f32(&self, x: u32, y: u32) -> Option<LinearRgbaF32> {
        self.as_ref().pixel_at_f32(x, y)
    }

    #[inline]
    pub fn set_pixel(&mut self, x: u32, y: u32, color: Color) -> bool {
        self.as_mut().set_pixel(x, y, color)
    }

    pub fn sub_surface(&self, rect: IRect) -> Result<SurfaceRef<'_>> {
        self.as_ref().sub_surface(rect)
    }

    pub fn sub_surface_mut(&mut self, rect: IRect) -> Result<SurfaceMut<'_>> {
        self.bump_generation();
        SurfaceMut {
            data: &mut self.data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        }
        .into_sub_surface(rect)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SurfaceRef<'a> {
    data: &'a [u8],
    width: u32,
    height: u32,
    stride: usize,
    format: PixelFormat,
    color_space: ColorSpace,
}

impl<'a> SurfaceRef<'a> {
    pub fn from_slice(
        data: &'a [u8],
        width: u32,
        height: u32,
        stride: usize,
        format: PixelFormat,
        color_space: ColorSpace,
    ) -> Result<Self> {
        let req = required_borrowed_len(width, height, stride, format.bytes_per_pixel())?;
        if data.len() < req {
            return Err(ZvgError::OutOfBounds);
        }
        Ok(Self {
            data,
            width,
            height,
            stride,
            format,
            color_space,
        })
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
    pub fn stride(&self) -> usize {
        self.stride
    }

    #[inline]
    pub fn format(&self) -> PixelFormat {
        self.format
    }

    #[inline]
    pub fn color_space(&self) -> ColorSpace {
        self.color_space
    }

    #[inline]
    pub fn data(&self) -> &'a [u8] {
        self.data
    }

    #[inline]
    pub fn row(&self, y: u32) -> Option<&'a [u8]> {
        if y >= self.height {
            return None;
        }
        let row_bytes = (self.width as usize) * self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride;
        Some(&self.data[off..off + row_bytes])
    }

    #[inline]
    pub fn pixel_at(&self, x: u32, y: u32) -> Option<LinearRgba16> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let bpp = self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride + (x as usize) * bpp;
        Some(decode_pixel_to_linear16(
            &self.data[off..off + bpp],
            self.format,
            self.color_space,
        ))
    }

    #[inline]
    pub fn pixel_at_f32(&self, x: u32, y: u32) -> Option<LinearRgbaF32> {
        if x >= self.width || y >= self.height {
            return None;
        }
        let bpp = self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride + (x as usize) * bpp;
        Some(decode_pixel_to_linear_f32(
            &self.data[off..off + bpp],
            self.format,
            self.color_space,
        ))
    }

    pub fn sub_surface(&self, rect: IRect) -> Result<SurfaceRef<'a>> {
        if rect.is_empty()
            || rect.min_x < 0
            || rect.min_y < 0
            || (rect.max_x as u32) > self.width
            || (rect.max_y as u32) > self.height
        {
            return Err(ZvgError::OutOfBounds);
        }
        let bpp = self.format.bytes_per_pixel();
        let start = (rect.min_y as usize) * self.stride + (rect.min_x as usize) * bpp;
        let sub_w = rect.width() as u32;
        let sub_h = rect.height() as u32;
        let req = required_borrowed_len(sub_w, sub_h, self.stride, bpp)?;
        Ok(SurfaceRef {
            data: &self.data[start..start + req],
            width: sub_w,
            height: sub_h,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        })
    }
}

#[derive(Debug)]
pub struct SurfaceMut<'a> {
    data: &'a mut [u8],
    width: u32,
    height: u32,
    stride: usize,
    format: PixelFormat,
    color_space: ColorSpace,
}

impl<'a> SurfaceMut<'a> {
    pub fn from_slice(
        data: &'a mut [u8],
        width: u32,
        height: u32,
        stride: usize,
        format: PixelFormat,
        color_space: ColorSpace,
    ) -> Result<Self> {
        let req = required_borrowed_len(width, height, stride, format.bytes_per_pixel())?;
        if data.len() < req {
            return Err(ZvgError::OutOfBounds);
        }
        Ok(Self {
            data,
            width,
            height,
            stride,
            format,
            color_space,
        })
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
    pub fn stride(&self) -> usize {
        self.stride
    }

    #[inline]
    pub fn format(&self) -> PixelFormat {
        self.format
    }

    #[inline]
    pub fn color_space(&self) -> ColorSpace {
        self.color_space
    }

    #[inline]
    pub fn data(&self) -> &[u8] {
        self.data
    }

    #[inline]
    pub fn data_mut(&mut self) -> &mut [u8] {
        self.data
    }

    #[inline]
    pub fn as_ref(&self) -> SurfaceRef<'_> {
        SurfaceRef {
            data: self.data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        }
    }

    #[inline]
    pub fn row(&self, y: u32) -> Option<&[u8]> {
        self.as_ref().row(y)
    }

    #[inline]
    pub fn row_mut(&mut self, y: u32) -> Option<&mut [u8]> {
        if y >= self.height {
            return None;
        }
        let row_bytes = (self.width as usize) * self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride;
        Some(&mut self.data[off..off + row_bytes])
    }

    pub fn clear(&mut self, color: Color) {
        let bpp = self.format.bytes_per_pixel();
        let mut encoded = [0u8; 16];
        encode_linear_f32_to_pixel(
            color.to_linear_f32(),
            &mut encoded[..bpp],
            self.format,
            self.color_space,
        );
        let row_bytes = (self.width as usize) * bpp;
        for y in 0..self.height {
            let off = (y as usize) * self.stride;
            let row = &mut self.data[off..off + row_bytes];
            for px in row.chunks_exact_mut(bpp) {
                px.copy_from_slice(&encoded[..bpp]);
            }
        }
    }

    #[inline]
    pub fn pixel_at(&self, x: u32, y: u32) -> Option<LinearRgba16> {
        self.as_ref().pixel_at(x, y)
    }

    #[inline]
    pub fn pixel_at_f32(&self, x: u32, y: u32) -> Option<LinearRgbaF32> {
        self.as_ref().pixel_at_f32(x, y)
    }

    #[inline]
    pub fn set_pixel(&mut self, x: u32, y: u32, color: Color) -> bool {
        if x >= self.width || y >= self.height {
            return false;
        }
        let bpp = self.format.bytes_per_pixel();
        let off = (y as usize) * self.stride + (x as usize) * bpp;
        encode_linear_f32_to_pixel(
            color.to_linear_f32(),
            &mut self.data[off..off + bpp],
            self.format,
            self.color_space,
        );
        true
    }

    pub fn into_sub_surface(self, rect: IRect) -> Result<SurfaceMut<'a>> {
        if rect.is_empty()
            || rect.min_x < 0
            || rect.min_y < 0
            || (rect.max_x as u32) > self.width
            || (rect.max_y as u32) > self.height
        {
            return Err(ZvgError::OutOfBounds);
        }
        let bpp = self.format.bytes_per_pixel();
        let start = (rect.min_y as usize) * self.stride + (rect.min_x as usize) * bpp;
        let sub_w = rect.width() as u32;
        let sub_h = rect.height() as u32;
        let req = required_borrowed_len(sub_w, sub_h, self.stride, bpp)?;
        Ok(SurfaceMut {
            data: &mut self.data[start..start + req],
            width: sub_w,
            height: sub_h,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        })
    }

    pub fn sub_surface_mut(&mut self, rect: IRect) -> Result<SurfaceMut<'_>> {
        SurfaceMut {
            data: self.data,
            width: self.width,
            height: self.height,
            stride: self.stride,
            format: self.format,
            color_space: self.color_space,
        }
        .into_sub_surface(rect)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::float::abs_f32;

    #[test]
    fn validation_and_overflow_rejection() {
        assert_eq!(
            Surface::new_rgba8(0, 10).unwrap_err(),
            ZvgError::InvalidArgument("surface dimensions must be non-zero")
        );
        assert_eq!(
            Surface::new_rgba8(10, 0).unwrap_err(),
            ZvgError::InvalidArgument("surface dimensions must be non-zero")
        );
        assert_eq!(
            Surface::new_rgba8(u32::MAX, 4).unwrap_err(),
            ZvgError::AllocationLimit
        );
        assert_eq!(
            Surface::try_new(20_000, 20_000, PixelFormat::Rgba8Srgb, ColorSpace::Srgb).unwrap_err(),
            ZvgError::AllocationLimit
        );

        let mut buf = [0u8; 32];
        assert_eq!(
            SurfaceMut::from_slice(&mut buf, 4, 2, 12, PixelFormat::Rgba8Srgb, ColorSpace::Srgb)
                .unwrap_err(),
            ZvgError::InvalidArgument("stride smaller than row width")
        );
        assert_eq!(
            SurfaceMut::from_slice(&mut buf, 4, 3, 16, PixelFormat::Rgba8Srgb, ColorSpace::Srgb)
                .unwrap_err(),
            ZvgError::OutOfBounds
        );
    }

    #[test]
    fn non_tight_stride_and_sub_surface() {
        // 4x3 RGBA8 surface with 24-byte padded stride (8 padding bytes per row).
        let mut backing = vec![0xAAu8; 24 * 3];
        {
            let mut surf = SurfaceMut::from_slice(
                &mut backing,
                4,
                3,
                24,
                PixelFormat::Rgba8Srgb,
                ColorSpace::Srgb,
            )
            .unwrap();
            surf.clear(Color::from_rgba8(10, 20, 30, 255));
            let mut sub = surf.sub_surface_mut(IRect::new(1, 1, 3, 3)).unwrap();
            sub.clear(Color::from_rgba8(200, 100, 50, 255));
        }

        // Padding bytes at end of each row must remain untouched (0xAA).
        for y in 0..3 {
            let pad = &backing[y * 24 + 16..y * 24 + 24];
            assert!(pad.iter().all(|&b| b == 0xAA));
        }

        let r = SurfaceRef::from_slice(
            &backing,
            4,
            3,
            24,
            PixelFormat::Rgba8Srgb,
            ColorSpace::Srgb,
        )
        .unwrap();
        assert_eq!(r.pixel_at(0, 0).unwrap().to_srgb_u8(), [10, 20, 30, 255]);
        assert_eq!(r.pixel_at(1, 1).unwrap().to_srgb_u8(), [200, 100, 50, 255]);
        assert_eq!(r.pixel_at(2, 2).unwrap().to_srgb_u8(), [200, 100, 50, 255]);
        assert_eq!(r.pixel_at(3, 2).unwrap().to_srgb_u8(), [10, 20, 30, 255]);
        assert!(r.pixel_at(4, 0).is_none());
    }

    #[test]
    fn rgba_bgra_hdr_and_display_p3_symmetry() {
        let color = Color::from_rgba8(220, 80, 40, 255);

        let mut rgba = Surface::try_new(2, 2, PixelFormat::Rgba8Srgb, ColorSpace::Srgb).unwrap();
        let mut bgra = Surface::try_new(2, 2, PixelFormat::Bgra8Srgb, ColorSpace::Srgb).unwrap();
        rgba.clear(color);
        bgra.clear(color);
        assert_eq!(rgba.row(0).unwrap()[0..4], [220, 80, 40, 255]);
        assert_eq!(bgra.row(0).unwrap()[0..4], [40, 80, 220, 255]);
        assert_eq!(rgba.pixel_at(0, 0), bgra.pixel_at(0, 0));

        // Linear u16 surface
        let mut s16 =
            Surface::try_new(2, 2, PixelFormat::Rgba16Linear, ColorSpace::LinearSrgb).unwrap();
        s16.clear(color);
        assert_eq!(s16.pixel_at(1, 1).unwrap().to_srgb_u8(), [220, 80, 40, 255]);

        // HDR f32 surface preserves > 1.0 channel values
        let mut hdr =
            Surface::try_new(2, 2, PixelFormat::RgbaF32Linear, ColorSpace::LinearSrgb).unwrap();
        let bright = Color::new(2.0, 0.5, 0.25, 1.0);
        hdr.set_pixel(0, 1, bright);
        let got = hdr.pixel_at_f32(0, 1).unwrap();
        let exp = bright.to_linear_f32();
        assert!(abs_f32(got.r - exp.r) < 1e-5 && got.r > 1.5);
        assert!(abs_f32(got.g - exp.g) < 1e-5);
        assert!(abs_f32(got.b - exp.b) < 1e-5);

        // Display P3 round-trip
        let mut p3 =
            Surface::try_new(2, 2, PixelFormat::Rgba8Srgb, ColorSpace::DisplayP3).unwrap();
        p3.clear(color);
        let back = p3.pixel_at(0, 0).unwrap().to_srgb_u8();
        assert!((back[0] as i16 - 220).abs() <= 1);
        assert!((back[1] as i16 - 80).abs() <= 1);
        assert!((back[2] as i16 - 40).abs() <= 1);
    }
}
