#![forbid(unsafe_code)]

pub mod scalar;

use crate::math::arena::{TILE_AREA, TILE_SIZE};
use crate::math::color::{ColorSpace, LinearRgba16, LinearRgbaF32};
use crate::surface::PixelFormat;

pub use scalar::{
    fill_cov_span_src_over_f32, fill_cov_span_src_over_u16, fill_solid_span_src_over_f32,
    fill_solid_span_src_over_u16, fill_solid_tile_src_over_f32, fill_solid_tile_src_over_u16,
    load_tile_f32, load_tile_u16, store_tile_f32, store_tile_u16,
};

#[inline]
pub const fn surface_uses_f32_pipeline(format: PixelFormat, color_space: ColorSpace) -> bool {
    matches!(format, PixelFormat::RgbaF32Linear)
        || matches!(color_space, ColorSpace::DisplayP3 | ColorSpace::LinearDisplayP3)
}

// 8 KB L1-resident 32x32 linear-light premultiplied u16 tile scratchpad.
#[repr(C, align(32))]
#[derive(Clone)]
pub struct TileBuffer16 {
    pub pixels: [LinearRgba16; TILE_AREA],
}

impl Default for TileBuffer16 {
    fn default() -> Self {
        Self::new()
    }
}

impl TileBuffer16 {
    pub const fn new() -> Self {
        Self {
            pixels: [LinearRgba16::TRANSPARENT; TILE_AREA],
        }
    }

    #[inline]
    pub fn clear(&mut self, color: LinearRgba16) {
        self.pixels.fill(color);
    }

    #[inline]
    pub fn clear_rect(&mut self, width: usize, height: usize, color: LinearRgba16) {
        let w = width.min(TILE_SIZE);
        let h = height.min(TILE_SIZE);
        if w == TILE_SIZE {
            self.pixels[..h * TILE_SIZE].fill(color);
        } else {
            for py in 0..h {
                let off = py * TILE_SIZE;
                self.pixels[off..off + w].fill(color);
            }
        }
    }

    #[inline]
    pub fn row(&self, py: usize) -> &[LinearRgba16] {
        let off = py * TILE_SIZE;
        &self.pixels[off..off + TILE_SIZE]
    }

    #[inline]
    pub fn row_mut(&mut self, py: usize) -> &mut [LinearRgba16] {
        let off = py * TILE_SIZE;
        &mut self.pixels[off..off + TILE_SIZE]
    }
}

// 16 KB L1-resident 32x32 linear-light premultiplied f32 tile scratchpad for HDR & Display P3.
#[repr(C, align(32))]
#[derive(Clone)]
pub struct TileBufferF32 {
    pub pixels: [LinearRgbaF32; TILE_AREA],
}

impl Default for TileBufferF32 {
    fn default() -> Self {
        Self::new()
    }
}

impl TileBufferF32 {
    pub const fn new() -> Self {
        Self {
            pixels: [LinearRgbaF32::TRANSPARENT; TILE_AREA],
        }
    }

    #[inline]
    pub fn clear(&mut self, color: LinearRgbaF32) {
        self.pixels.fill(color);
    }

    #[inline]
    pub fn clear_rect(&mut self, width: usize, height: usize, color: LinearRgbaF32) {
        let w = width.min(TILE_SIZE);
        let h = height.min(TILE_SIZE);
        if w == TILE_SIZE {
            self.pixels[..h * TILE_SIZE].fill(color);
        } else {
            for py in 0..h {
                let off = py * TILE_SIZE;
                self.pixels[off..off + w].fill(color);
            }
        }
    }

    #[inline]
    pub fn row(&self, py: usize) -> &[LinearRgbaF32] {
        let off = py * TILE_SIZE;
        &self.pixels[off..off + TILE_SIZE]
    }

    #[inline]
    pub fn row_mut(&mut self, py: usize) -> &mut [LinearRgbaF32] {
        let off = py * TILE_SIZE;
        &mut self.pixels[off..off + TILE_SIZE]
    }
}
