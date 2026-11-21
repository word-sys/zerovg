#![forbid(unsafe_code)]

pub mod affine;
pub mod arena;
pub mod color;
pub mod dither;
pub mod fixed;
pub mod float;

pub use affine::{Affine2D, IRect, Point, Point64, Rect, Vec2};
pub use arena::{
    CHUNK_CAP, EdgeSeg, FrameArena, NO_CHUNK, TILE_AREA, TILE_SIZE, TileCmd, TileCmdKind, TileSeg,
    TileSegmentChunk,
};
pub use color::{Color, ColorSpace, LinearRgba16, LinearRgbaF32, Oklab, Oklch};
pub use dither::DitherMode;
pub use fixed::Fixed24_8;
