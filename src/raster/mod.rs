#![forbid(unsafe_code)]

pub mod simd;
pub mod strip;

pub use simd::{TileBuffer16, TileBufferF32, surface_uses_f32_pipeline};
pub use strip::{
    AntialiasMode, CoverageSpan, DIFF_LEN, DIFF_STRIDE, STRIP_HEIGHT, STRIPS_PER_TILE,
    TileFillParams, TileRasterScratch, acc_to_cov_u16, cov_u16_to_u8,
};
