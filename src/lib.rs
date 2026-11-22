#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]
#![warn(clippy::all)]

extern crate alloc;

#[cfg(feature = "std")]
extern crate std;

pub mod codec;
pub mod error;
pub mod geom;
pub mod math;
pub mod surface;

pub use error::{Result, ZvgError};
pub use geom::{FillRule, Path, PathBuilder, PathEvent, PathVerb};
pub use math::{
    Affine2D, Color, ColorSpace, DitherMode, IRect, LinearRgba16, LinearRgbaF32, Oklab, Oklch,
    Point, Rect, Vec2,
};
pub use surface::{PixelFormat, Surface, SurfaceMut, SurfaceRef};

pub const VERSION_MAJOR: u32 = 0;
pub const VERSION_MINOR: u32 = 1;
pub const VERSION_PATCH: u32 = 0;
pub const VERSION_STR: &str = env!("CARGO_PKG_VERSION");

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_matches_cargo() {
        assert_eq!(VERSION_STR, "0.1.0");
        assert_eq!((VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH), (0, 1, 0));
    }
}
