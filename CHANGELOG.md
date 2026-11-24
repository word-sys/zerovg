# Changelog

All notable changes to **ZeroVG** (`zerovg`) are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2026-11-24

Initial `v0.1` release delivering the complete zero-dependency `#![no_std]` + `alloc` 2D vector rasterization core, linear-light color pipeline, pure-Rust image codecs, and dual modern/Cairo-stateful drawing API.

### Added

#### Core Architecture & Math
- Zero-dependency `#![no_std]` + `alloc` crate layout with feature-flag matrix and `#![forbid(unsafe_code)]` across all core modules.
- Deterministic pure-Rust IEEE-754 math library (`math::float`): exact 0-ULP `sqrt`, `<= 1 ULP` residual-Halley `cbrt`, 3-part Cody-Waite `sincos`, `atan2`, `acos`, `exp`, `exp2`, `ln`, `log2`, and `powf` (zero `libm` calls, zero FMA contraction).
- Geometric primitives (`Point`, `Point64`, `Vec2`, `Rect`, `IRect`), `Affine2D` (with `f64` inversion and exact SVD `max_scale_factor`), and `Fixed24_8` with exact round-ties-to-even `mul_div_i64`.
- Typed-slab `FrameArena` (`EdgeSeg`, `TileSeg`, 256-byte `TileSegmentChunk`, `TileCmd`, `clip_masks`) with `O(1)` `reset()` and `shrink_to_limit()` for zero steady-state frame allocations.

#### Color Science, Dithering & Surfaces
- `ColorSpace` (`Srgb`, `LinearSrgb`, `DisplayP3`, `LinearDisplayP3`), `Color` (including `from_display_p3` and `from_linear_rgba`), premultiplied `LinearRgba16`, HDR `LinearRgbaF32`, `Oklab`, `Oklch`, exact `mul_u16`, and compile-time `const fn` sRGB transfer LUTs.
- Deterministic spatial dithering (`DitherMode::{BlueNoise, InterleavedGradientNoise, None}`) using a 16×16 toroidal Ulichney Void-and-Cluster blue-noise matrix and pure-integer Q32 IGN with zero endpoint speckle.
- Owned `Surface` (with generation tracking) and zero-copy borrowed `SurfaceRef` / `SurfaceMut` (with non-tight stride and sub-surface slicing) supporting `Rgba8Srgb`, `Bgra8Srgb`, `Rgba8Linear`, `Rgba16Linear`, and `RgbaF32Linear`.

#### Pure-Rust Image Codecs
- RFC 1951 DEFLATE/INFLATE, RFC 1950 Zlib, CRC32, and Adler-32 (`codec::deflate`) with LZ77 lazy matching, Package-Merge 15-bit Huffman coding, strict Kraft validation, and zip-bomb limits.
- Pure-Rust PNG encoder & decoder (`codec-png`) with adaptive 5-filter selection, all color types (`0/2/3/4/6`), bit depths (`1/2/4/8/16`), `PLTE`, `tRNS`, Adam7 interlacing, and `PngDecodeLimits`.
- Opt-in pure-Rust JPEG decoder (`codec-jpeg`) supporting Baseline (`SOF0`/`SOF1`) and Progressive (`SOF2`), `DRI`/`RSTn`, integer Loeffler 8×8 IDCT, chroma upsampling, and `JpegDecodeLimits`.

#### Path Geometry & Flattening
- Compact SoA `Path` and fluent `PathBuilder` with SVG F.6.5/F.6.6 `arc_to`, Cairo-style `arc_center`, rounded rects, ellipses, circles, polygons, and deterministic `NaN`/`Inf` rejection.
- Exact analytical quadratic/cubic curve bounds (`B'(t) = 0` via stable Citardauq roots) and two-stage viewport culling.
- Screen-space Wang's adaptive Bézier flattening (`flatten_path_for_fill`) with exact `f64` viewport clipping and `x < 0` left-edge winding projection.
- Interactive geometry queries (`winding_number`, `point_in_fill`, damped-Newton `closest_point_on_path`, and `signed_distance_to_path`).

#### Tiled Sparse-Strip Rasterizer & Drawing Context
- Coarse `32×32` `TileBinner` with unified `24.8` row-then-column tile splitting, exact Stokes' theorem backdrop winding propagation, left-wall compensating vertical edges, `SolidTile` fast-path classification, and FIFO per-tile command streams.
- L1-resident 4-row sparse-strip coverage rasterizer (`TileRasterScratch`) supporting `FillRule::{NonZero, EvenOdd}`, `AntialiasMode::{Analytic, None}`, contiguous solid/fractional span extraction, and wrapping winding overflow resilience.
- Scalar linear-light tile compositing blitters (`TileBuffer16` `8 KB` and `TileBufferF32` `16 KB` for HDR & wide-gamut Display P3) with load/blend/dithered-store and partial edge-tile bounds safety.
- Immediate-mode and Cairo-compatible stateful `Context` v1 (`save`/`restore`, transforms, `fill`, `stroke`, `move_to`/`line_to`/`curve_to`/`arc`/`arc_negative`/`rectangle`/`fill_preserve`/`fill_current`/`paint_with_alpha`/`path_extents`/`copy_path_flat`, and deferred tile-batch `flush()`).

[Unreleased]: https://github.com/word-sys/zerovg/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/word-sys/zerovg/releases/tag/v0.1.0
