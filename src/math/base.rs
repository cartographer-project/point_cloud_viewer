use nalgebra::{Point3, RealField};
use std::fmt;

pub trait PointCulling<S>: fmt::Debug + Sync + Send
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool;
}
