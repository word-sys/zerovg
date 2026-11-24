# Security Policy

## Supported Versions

| Version | Supported |
| :--- | :---: |
| `0.1.x` | Yes |

## Reporting a Vulnerability

If you discover a security vulnerability (such as an out-of-bounds memory access, integer overflow leading to panic or corruption, decompression bomb / OOM bypass, or infinite loop on untrusted input), please report it privately rather than opening a public issue:

- Use **GitHub Security Advisories** ("Report a vulnerability") on the repository: https://github.com/word-sys/zerovg/security/advisories/new

Please include:
- A concise description of the issue and affected component (e.g., PNG decoder, JPEG decoder, path geometry, tile rasterizer).
- Minimal reproduction input or test case.
- Rust toolchain version and feature flags used.

## Built-In Hardening & Untrusted Input Defenses

ZeroVG is designed to safely process untrusted images, paths, and documents:

- `#![forbid(unsafe_code)]` across all parsers, codecs, geometry, tile binning, and scalar rasterization.
- Explicit resource bounds: `MAX_SURFACE_DIM`, `MAX_SURFACE_BYTES`, `PngDecodeLimits`, `JpegDecodeLimits`, and `FrameArena::shrink_to_limit`.
- Strict RFC 1951 Huffman Kraft checks, CRC32/Adler-32 validation, and decompression-ratio zip-bomb guards.
- Deterministic `NaN`/`Inf` rejection at path and transform boundaries and `wrapping_add` winding accumulation preventing debug/release overflow panics on hostile deep-overlap inputs.
