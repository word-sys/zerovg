#![forbid(unsafe_code)]

pub mod bounds;
pub mod path;

pub use bounds::{
    cubic_bounds, path_tight_bounds, path_transformed_tight_bounds, path_visible_in_viewport,
    quad_bounds,
};
pub use path::{FillRule, Path, PathBuilder, PathEvent, PathIter, PathVerb};
