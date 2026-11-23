#![forbid(unsafe_code)]

pub mod bounds;
pub mod flatten;
pub mod path;

pub use bounds::{
    cubic_bounds, path_tight_bounds, path_transformed_tight_bounds, path_visible_in_viewport,
    quad_bounds,
};
pub use flatten::{
    DEFAULT_TOLERANCE, flatten_cubic, flatten_path_for_fill, flatten_path_segments, flatten_quad,
    wang_cubic_subdivs, wang_quad_subdivs,
};
pub use path::{FillRule, Path, PathBuilder, PathEvent, PathIter, PathVerb};
