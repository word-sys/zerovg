#![forbid(unsafe_code)]

#[cfg(any(feature = "codec-png", feature = "export-vector"))]
pub mod deflate;
