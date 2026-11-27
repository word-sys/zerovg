#![forbid(unsafe_code)]

pub mod gradient;

use crate::math::color::Color;

pub use gradient::{
    ColorStop, ConicGradient, GRADIENT_RAMP_LEN, GradientRamp, InterpolationSpace, LinearGradient,
    RadialGradient, SpreadMode,
};

#[derive(Debug, Clone, PartialEq)]
pub enum Paint {
    Solid(Color),
    LinearGradient(LinearGradient),
    RadialGradient(RadialGradient),
    ConicGradient(ConicGradient),
}

impl Default for Paint {
    fn default() -> Self {
        Self::Solid(Color::BLACK)
    }
}

impl Paint {
    pub fn bake(&mut self) {
        match self {
            Self::Solid(_) => {}
            Self::LinearGradient(g) => g.bake(),
            Self::RadialGradient(g) => g.bake(),
            Self::ConicGradient(g) => g.bake(),
        }
    }
}

impl From<Color> for Paint {
    #[inline]
    fn from(color: Color) -> Self {
        Self::Solid(color)
    }
}

impl From<LinearGradient> for Paint {
    #[inline]
    fn from(grad: LinearGradient) -> Self {
        Self::LinearGradient(grad)
    }
}

impl From<RadialGradient> for Paint {
    #[inline]
    fn from(grad: RadialGradient) -> Self {
        Self::RadialGradient(grad)
    }
}

impl From<ConicGradient> for Paint {
    #[inline]
    fn from(grad: ConicGradient) -> Self {
        Self::ConicGradient(grad)
    }
}
