# Contributing to ZeroVG

Thank you for your interest in contributing to **ZeroVG**! Before submitting a pull request, please review the project's core engineering invariants and development workflow.

## Hard Engineering Invariants

1. **Zero External Dependencies:** `[dependencies]`, `[build-dependencies]`, and `[dev-dependencies]` in `Cargo.toml` must remain completely empty.
2. **`#![no_std]` + `alloc` Core:** Core library modules must compile cleanly with `cargo build --no-default-features` and `cargo test --no-default-features`. Any `std`-only helper must be gated behind `#[cfg(feature = "std")]`.
3. **Strict Memory Safety:** All modules outside `src/raster/simd/` (hardware SIMD intrinsics) and `src/ffi/` (C ABI boundary) must carry `#![forbid(unsafe_code)]`. Every `unsafe` block in allowed modules requires an explicit `// SAFETY:` justification.
4. **Deterministic Math:** Do not call `libm` or platform-dependent `std` transcendentals; use `crate::math::float`. Do not rely on hardware FMA contraction.
5. **Zero Steady-State Frame Allocations:** Per-frame rendering code must reuse `FrameArena` and scratch buffers so repeated frames perform `0` heap allocations and `0` deallocations.
6. **Clean Code & Minimal Comments:** Keep code readable and idiomatic with concise comments only where non-obvious invariants or formulas benefit from explanation.

## Verification Checklist

Run the full check suite before opening a PR:

```bash
cargo build --no-default-features
cargo test --no-default-features
cargo test --all-features
cargo test --release --test core_raster
cargo clippy --no-default-features --all-targets -- -D warnings
cargo clippy --all-features --all-targets -- -D warnings
```

## Community & Licensing

- All contributors are expected to uphold our [Code of Conduct](CODE_OF_CONDUCT.md).
- Contributions are licensed under the **Apache License, Version 2.0** (`Apache-2.0`, matching [LICENSE](LICENSE)).
