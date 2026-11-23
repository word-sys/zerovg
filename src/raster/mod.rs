#![forbid(unsafe_code)]

pub mod strip;

pub use strip::{
    AntialiasMode, CoverageSpan, DIFF_LEN, DIFF_STRIDE, STRIP_HEIGHT, STRIPS_PER_TILE,
    TileFillParams, TileRasterScratch, acc_to_cov_u16, cov_u16_to_u8,
};
