use super::base::PointCulling;
use super::sat::ConvexPolyhedron;
use crate::proto;
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Point3, RealField, Unit, Vector3};
use serde::{Deserialize, Serialize};

/// An Axis Aligned Bounding Box.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct AABB<S: RealField> {
    mins: Point3<S>,
    maxs: Point3<S>,
}

impl<S: RealField> AABB<S> {
    pub fn new(mins: Point3<S>, maxs: Point3<S>) -> Self {
        AABB { mins, maxs }
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

    pub fn transform(&self, transform: &Isometry3<S>) -> AABB<S> {
        let corners = self.corners();
        let transformed_first = transform.transform_point(&corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..].iter().fold(base, |mut u, &corner| {
            u.grow(transform.transform_point(&corner));
            u
        })
    }

    /// It's convenient to have this associated function for intersection testing.
    /// To be precise, it's nice if we can give this to cache_separating_axes() without
    /// having to reference a specific AABB instance.
    pub fn axes() -> [Unit<Vector3<S>>; 3] {
        [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()]
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
}

impl<S> ConvexPolyhedron<S> for AABB<S>
where
    S: RealField,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        self.corners()
    }

    fn compute_edges(&self) -> ArrayVec<[Unit<Vector3<S>>; 6]> {
        let mut edges = ArrayVec::new();
        edges.push(Vector3::x_axis());
        edges.push(Vector3::y_axis());
        edges.push(Vector3::z_axis());
        edges
    }

    fn compute_face_normals(&self) -> ArrayVec<[Unit<Vector3<S>>; 6]> {
        self.compute_edges()
    }
}
