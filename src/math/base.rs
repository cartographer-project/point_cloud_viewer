use super::aabb::AABB;
use nalgebra::{Point3, RealField, Scalar};
use std::fmt;

pub trait PointCulling<S>: fmt::Debug + Sync + Send
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool;
    // TODO(nnmm): better name
    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation;
}

pub trait Cuboid<S: Scalar> {
    fn corners(&self) -> [Point3<S>; 8];
}

/// Spatial relation between two objects.
/// Modeled after the collision crate.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialOrd, PartialEq)]
pub enum Relation {
    /// Completely inside.
    In,
    /// Crosses the boundary.
    Cross,
    /// Completely outside.
    Out,
}
