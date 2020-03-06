//! A bounding box with an arbitrary 3D pose.

use crate::math::{
    CachedAxesIntersector, ConvexPolyhedron, HasAabbIntersector, Isometry3, PointCulling,
};
use arrayvec::ArrayVec;
use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Point3, Quaternion, Vector3};
use collision::{Aabb, Aabb3};
use num_traits::identities::One;
use num_traits::Bounded;
use serde::{Deserialize, Serialize};
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obb<S> {
    query_from_obb: Isometry3<S>,
    obb_from_query: Isometry3<S>,
    half_extent: Vector3<S>,
}

impl<S: BaseFloat> From<&Aabb3<S>> for Obb<S> {
    fn from(aabb: &Aabb3<S>) -> Self {
        Obb::new(
            Isometry3::new(Quaternion::one(), EuclideanSpace::to_vec(aabb.center())),
            aabb.dim() / (S::one() + S::one()),
        )
    }
}

impl<S: BaseFloat> From<Aabb3<S>> for Obb<S> {
    fn from(aabb: Aabb3<S>) -> Self {
        Self::from(&aabb)
    }
}

impl<S: BaseFloat> Obb<S> {
    pub fn new(query_from_obb: Isometry3<S>, half_extent: Vector3<S>) -> Self {
        Obb {
            obb_from_query: query_from_obb.inverse(),
            query_from_obb,
            half_extent,
        }
    }

    pub fn transformed(&self, global_from_query: &Isometry3<S>) -> Self {
        Self::new(global_from_query * &self.query_from_obb, self.half_extent)
    }
}

impl<S: BaseFloat> ConvexPolyhedron<S> for Obb<S> {
    fn compute_corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| &self.query_from_obb * &Point3::new(x, y, z);
        [
            corner_from(
                -self.half_extent.x,
                -self.half_extent.y,
                -self.half_extent.z,
            ),
            corner_from(self.half_extent.x, -self.half_extent.y, -self.half_extent.z),
            corner_from(-self.half_extent.x, self.half_extent.y, -self.half_extent.z),
            corner_from(self.half_extent.x, self.half_extent.y, -self.half_extent.z),
            corner_from(-self.half_extent.x, -self.half_extent.y, self.half_extent.z),
            corner_from(self.half_extent.x, -self.half_extent.y, self.half_extent.z),
            corner_from(-self.half_extent.x, self.half_extent.y, self.half_extent.z),
            corner_from(self.half_extent.x, self.half_extent.y, self.half_extent.z),
        ]
    }
    fn compute_edges(&self) -> ArrayVec<[Vector3<S>; 6]> {
        let mut edges = ArrayVec::new();
        edges.push((self.query_from_obb.rotation * Vector3::unit_x()).normalize());
        edges.push((self.query_from_obb.rotation * Vector3::unit_y()).normalize());
        edges.push((self.query_from_obb.rotation * Vector3::unit_z()).normalize());
        edges
    }

    fn compute_face_normals(&self) -> ArrayVec<[Vector3<S>; 6]> {
        self.compute_edges()
    }
}

impl<S> PointCulling<S> for Obb<S>
where
    S: 'static + BaseFloat + Sync + Send + Bounded,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        let Point3 { x, y, z } = &self.obb_from_query * p;
        x.abs() <= self.half_extent.x
            && y.abs() <= self.half_extent.y
            && z.abs() <= self.half_extent.z
    }
}

impl<'a, S: 'static + BaseFloat + Bounded> HasAabbIntersector<'a, S> for Obb<S> {
    type Intersector = CachedAxesIntersector<S>;
    fn aabb_intersector(&self) -> Self::Intersector {
        let unit_axes = [Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()];
        self.intersector()
            .cache_separating_axes(&unit_axes, &unit_axes)
    }
}
