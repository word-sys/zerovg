#![forbid(unsafe_code)]

#[cfg(feature = "boolean")]
pub mod boolean;
pub mod bounds;
pub mod dash;
pub mod flatten;
pub mod path;
pub mod query;
pub mod stroke;

#[cfg(feature = "boolean")]
pub use boolean::{
    GRID_MAX, GridPoint, GridSeg, GridTransform, HotPixel, SegmentIntersection, collect_hot_pixels,
    intersect_segments, orient2d, orient2d_det,
};

pub use bounds::{
    cubic_bounds, path_tight_bounds, path_transformed_tight_bounds, path_visible_in_viewport,
    quad_bounds,
};
pub use dash::{
    cubic_arc_length, cubic_t_at_arc_length, dash_path, dash_path_into, quad_arc_length,
    quad_t_at_arc_length, subpath_arc_lengths,
};
pub use flatten::{
    DEFAULT_TOLERANCE, flatten_cubic, flatten_path_for_fill, flatten_path_segments, flatten_quad,
    wang_cubic_subdivs, wang_quad_subdivs,
};
pub use path::{FillRule, Path, PathBuilder, PathEvent, PathIter, PathVerb};
pub use query::{
    ClosestPointResult, closest_point_on_path, point_in_fill, point_in_stroke,
    signed_distance_to_path, winding_number,
};
pub use stroke::{
    CubicSeg, LineCap, LineJoin, OffsetSegment, QuadSeg, StrokeStyle,
    cubic_inflection_and_cusp_params, decompose_cubic_for_stroke, decompose_quad_for_stroke,
    expand_stroke_into, expand_stroke_path, offset_cubic_subcurve, offset_quad_subcurve,
    quad_cusp_and_extrema_params,
};
