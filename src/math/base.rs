use crate::geometry::Aabb;
use nalgebra::{Point3, RealField};

pub trait PointCulling<S>: Sync + Send
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool;

    fn intersects_aabb(&self, aabb: &Aabb<S>) -> bool;
}
