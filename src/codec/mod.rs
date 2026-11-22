#![forbid(unsafe_code)]

#[cfg(any(feature = "codec-png", feature = "export-vector"))]
pub mod deflate;

#[cfg(feature = "codec-jpeg")]
pub mod jpeg;

#[cfg(feature = "codec-png")]
pub mod png;
