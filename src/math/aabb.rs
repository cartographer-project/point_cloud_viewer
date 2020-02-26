use super::base::{PointCulling, Relation};
use super::sat::{ConvexPolyhedron, intersects_aabb};
use crate::proto;
use nalgebra::{Point3, RealField, Vector3};
use serde::{Deserialize, Serialize};

/// An Axis Aligned Bounding Box.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct AABB<S: RealField> {
    mins: Point3<S>,
    maxs: Point3<S>,
}

impl<S: RealField> AABB<S> {
    pub fn new(mins: Point3<S>, maxs: Point3<S>) -> Self {
        AABB {
            mins: mins,
            maxs: maxs,
        }
    }

    pub fn zero() -> Self {
        Self {
            mins: Point3::origin(),
            maxs: Point3::origin(),
        }
    }

    pub fn min(&self) -> &Point3<S> {
        &self.mins
    }

    pub fn max(&self) -> &Point3<S> {
        &self.maxs
    }

    pub fn grow(&mut self, p: Point3<S>) {
        self.mins = nalgebra::inf(&self.mins, &p);
        self.maxs = nalgebra::sup(&self.maxs, &p);
    }

    pub fn corners(&self) -> [Point3<S>; 8] {
        [
            self.mins,
            Point3::new(self.maxs.x, self.mins.y, self.mins.z),
            Point3::new(self.mins.x, self.maxs.y, self.mins.z),
            Point3::new(self.maxs.x, self.maxs.y, self.mins.z),
            Point3::new(self.mins.x, self.mins.y, self.maxs.z),
            Point3::new(self.maxs.x, self.mins.y, self.maxs.z),
            Point3::new(self.mins.x, self.maxs.y, self.maxs.z),
            self.maxs,
        ]
    }

    pub fn contains(&self, p: &Point3<S>) -> bool {
        nalgebra::partial_le(&self.mins, p) && nalgebra::partial_le(p, &self.maxs)
    }

    pub fn center(&self) -> Point3<S> {
        nalgebra::center(&self.mins, &self.maxs)
    }
}

impl From<&proto::AxisAlignedCuboid> for AABB<f64> {
    fn from(aac: &proto::AxisAlignedCuboid) -> Self {
        let aac_min = aac.min.clone().unwrap_or_else(|| {
            let deprecated_min = aac.deprecated_min.clone().unwrap(); // Version 9
            proto::Vector3d::from(deprecated_min)
        });
        let aac_max = aac.max.clone().unwrap_or_else(|| {
            let deprecated_max = aac.deprecated_max.clone().unwrap(); // Version 9
            proto::Vector3d::from(deprecated_max)
        });
        AABB::new(
            std::convert::From::from(&aac_min),
            std::convert::From::from(&aac_max),
        )
    }
}

impl From<&AABB<f64>> for proto::AxisAlignedCuboid {
    fn from(bbox: &AABB<f64>) -> Self {
        let mut aac = proto::AxisAlignedCuboid::new();
        aac.set_min(proto::Vector3d::from(bbox.min()));
        aac.set_max(proto::Vector3d::from(bbox.max()));
        aac
    }
}

impl<S> PointCulling<S> for AABB<S>
where
    S: RealField + num_traits::Bounded,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains(p)
    }
    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation {
        let separating_axes = &[Vector3::x(), Vector3::y(), Vector3::z()];
        intersects_aabb(&self.corners(), separating_axes, aabb)
    }
}

impl<S> ConvexPolyhedron<S> for AABB<S>
where
    S: RealField,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        self.corners()
    }

    fn compute_edges(&self) -> [Option<Vector3<S>>; 6] {
        [
            Some(Vector3::x()),
            Some(Vector3::y()),
            Some(Vector3::z()),
            None,
            None,
            None,
        ]
    }
}
