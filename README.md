# ZeroVG

[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Rust: 1.85+](https://img.shields.io/badge/rust-1.85%2B-orange.svg)](https://www.rust-lang.org)
[![Dependencies: 0](https://img.shields.io/badge/dependencies-0-brightgreen.svg)](Cargo.toml)
[![no_std](https://img.shields.io/badge/no__std-alloc-informational.svg)](src/lib.rs)

**ZeroVG** (`zerovg`) is a zero-dependency, `#![no_std]` + `alloc` 2D vector graphics, computational geometry, and typography engine written in pure Rust. It is designed to replace the legacy C graphics stack (Cairo, Pixman, FreeType, HarfBuzz, Pango, Clipper2, libpng, librsvg) with a single deterministic, memory-safe, L1-cache-resident engine.

## Core Principles

- **Strictly Zero Dependencies:** `[dependencies]`, `[build-dependencies]`, and `[dev-dependencies]` are completely empty. Every algorithm — IEEE-754 transcendentals, DEFLATE/INFLATE, PNG, JPEG, vector flattening, tile binning, and analytic rasterization — is implemented from scratch in pure Rust.
- **`#![no_std]` + `alloc` Core:** Runs identically on desktop, servers, embedded targets, and WebAssembly without `libm` or OS dependencies.
- **Memory-Safe by Construction:** `#![forbid(unsafe_code)]` across the entire core, codecs, geometry, tile binner, and scalar rasterizer.
- **Zero Steady-State Heap Allocations:** Reusable typed-slab `FrameArena`, flat intrusive per-tile command lists, and L1-resident tile scratchpads render frames with `0` allocations and `0` deallocations after warm-up.
- **Linear-Light Color Science & Wide Gamut:** Exact 16-bit premultiplied linear-light sRGB compositing with blue-noise / IGN dithering, plus full `f32` HDR (`> 1.0`) and Display P3 wide-gamut pipelines.
- **Bit-Exact Determinism:** Pure-Rust IEEE-754 math and fixed-point `24.8` rasterization produce identical SHA-256 framebuffer checksums across debug and release builds.

## Quick Start

Add `zerovg` to your `Cargo.toml`:

```toml
[dependencies]
zerovg = "0.1"
```

### Modern Immutable Path API

```rust
use zerovg::{Color, Context, FillRule, PathBuilder, Rect, Surface};

fn main() -> zerovg::Result<()> {
    let mut surface = Surface::new_rgba8(256, 256)?;

    let mut pb = PathBuilder::new();
    pb.push_rounded_rect(Rect::new(24.0, 24.0, 232.0, 232.0), 28.0, 28.0);
    let card = pb.finish();

    let mut circle_pb = PathBuilder::new();
    circle_pb.push_circle(128.0, 128.0, 64.0);
    let circle = circle_pb.finish();

    {
        let mut ctx = Context::new(&mut surface);
        ctx.clear(Color::from_hex(0x141821));
        ctx.fill_with_paint(&card, FillRule::NonZero, Color::from_hex(0x1E293B));
        ctx.fill_with_paint(&circle, FillRule::NonZero, Color::from_hex(0xF97316));
    } // flushes automatically on Drop (or call ctx.flush() / ctx.finish())

    surface.save_png("output.png")?;
    Ok(())
}
```

### Cairo-Compatible Stateful API

```rust
use core::f32::consts::PI;
use zerovg::{Color, Context, FillRule, Surface};

fn main() -> zerovg::Result<()> {
    let mut surface = Surface::new_rgba8(256, 256)?;
    let mut ctx = Context::new(&mut surface);

    ctx.clear(Color::from_hex(0x0F172A));
    ctx.save();
    ctx.translate(128.0, 128.0);
    ctx.arc(0.0, 0.0, 80.0, 0.0, 1.5 * PI);
    ctx.line_to(0.0, 0.0);
    ctx.close_path();

    ctx.set_paint(Color::from_hex(0x38BDF8));
    ctx.set_fill_rule(FillRule::NonZero);
    ctx.fill_preserve();

    ctx.set_paint(Color::new(1.0, 1.0, 1.0, 0.25));
    ctx.fill_current();
    ctx.restore();
    ctx.finish();

    let png_bytes = surface.encode_png()?;
    assert_eq!(&png_bytes[..8], b"\x89PNG\r\n\x1a\n");
    Ok(())
}
```

## Feature Flags

| Feature | Default | Implies | Description |
| :--- | :---: | :--- | :--- |
| `std` | Yes | — | File I/O helpers (`Surface::save_png`, `Surface::load_png`) and `std::io::Error` conversion. |
| `codec-png` | Yes | — | Pure-Rust RFC 1951 DEFLATE/INFLATE, Zlib, and full PNG encoder/decoder (all color types, bit depths 1/2/4/8/16, `PLTE`, `tRNS`, Adam7). |
| `codec-jpeg` | No | — | Opt-in pure-Rust Baseline (`SOF0`/`SOF1`) & Progressive (`SOF2`) JPEG decoder. |
| `threads` | Yes | `std` | Multi-core tile rasterization. |
| `simd` | Yes | — | Hardware SIMD blitter acceleration. |
| `boolean` | Yes | — | Snap-rounded boolean path operations. |
| `filters` | Yes | — | Separable Gaussian blur & color filters. |
| `font` | Yes | — | Native SFNT/TTF/OTF/Variable/COLR typography & shaping. |
| `codec-svg` | Yes | `filters` | Zero-copy SVG 1.1/2 compiler. |
| `export-vector` | Yes | — | SVG & PDF 1.7 vector exporters. |
| `ffi` | No | — | C11 ABI (`zvg_*`) symbols. |

For a minimal `#![no_std]` + `alloc` build:

```toml
[dependencies]
zerovg = { version = "0.1", default-features = false }
```

## Building & Testing

```bash
# Minimal no_std + alloc build & tests
cargo build --no-default-features
cargo test --no-default-features

# Full feature matrix tests (including PNG & opt-in JPEG codecs)
cargo test --all-features

# Release determinism & zero-allocation milestone verification
cargo test --release --test core_raster

# Strict lint checks
cargo clippy --no-default-features --all-targets -- -D warnings
cargo clippy --all-features --all-targets -- -D warnings
```

## Contributing & Security

- Read [CONTRIBUTING.md](CONTRIBUTING.md) before opening a pull request.
- All participants are expected to follow the [Code of Conduct](CODE_OF_CONDUCT.md).
- Report security issues following [SECURITY.md](SECURITY.md).

## License

ZeroVG is licensed under the **Apache License, Version 2.0** — see [LICENSE](LICENSE) for details.
