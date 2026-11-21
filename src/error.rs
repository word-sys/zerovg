#![forbid(unsafe_code)]

use core::fmt;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ZvgError {
    InvalidArgument(&'static str),
    OutOfBounds,
    AllocationLimit,
    ParseError(&'static str),
    UnsupportedFeature(&'static str),
    #[cfg(feature = "std")]
    Io(std::io::ErrorKind),
}

pub type Result<T> = core::result::Result<T, ZvgError>;

impl fmt::Display for ZvgError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidArgument(msg) => write!(f, "invalid argument: {msg}"),
            Self::OutOfBounds => f.write_str("out of bounds"),
            Self::AllocationLimit => f.write_str("allocation limit exceeded"),
            Self::ParseError(msg) => write!(f, "parse error: {msg}"),
            Self::UnsupportedFeature(msg) => write!(f, "unsupported feature: {msg}"),
            #[cfg(feature = "std")]
            Self::Io(kind) => write!(f, "io error: {kind:?}"),
        }
    }
}

impl core::error::Error for ZvgError {}

#[cfg(feature = "std")]
impl From<std::io::Error> for ZvgError {
    fn from(err: std::io::Error) -> Self {
        Self::Io(err.kind())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::format;

    #[test]
    fn error_display_and_trait() {
        let err = ZvgError::InvalidArgument("negative width");
        assert_eq!(format!("{err}"), "invalid argument: negative width");
        assert_eq!(format!("{}", ZvgError::OutOfBounds), "out of bounds");
        assert_eq!(
            format!("{}", ZvgError::AllocationLimit),
            "allocation limit exceeded"
        );
        assert_eq!(
            format!("{}", ZvgError::ParseError("bad header")),
            "parse error: bad header"
        );
        assert_eq!(
            format!("{}", ZvgError::UnsupportedFeature("unknown")),
            "unsupported feature: unknown"
        );

        let dyn_err: &dyn core::error::Error = &err;
        assert!(dyn_err.source().is_none());
    }

    #[cfg(feature = "std")]
    #[test]
    fn io_error_conversion() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "missing");
        let zvg_err: ZvgError = io_err.into();
        assert_eq!(zvg_err, ZvgError::Io(std::io::ErrorKind::NotFound));
    }
}
