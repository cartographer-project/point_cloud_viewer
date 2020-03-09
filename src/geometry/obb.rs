//! A bounding box with an arbitrary 3D pose.

use crate::math::{CachedAxesIntersector, ConvexPolyhedron, Intersector, Relation};
use crate::math::{Isometry3, PointCulling};
use arrayvec::ArrayVec;
use cgmath::{BaseFloat, EuclideanSpace, InnerSpace, Point3, Quaternion, Vector3};
use collision::{Aabb, Aabb3};
use num_traits::identities::One;
use num_traits::Bounded;
use serde::{Deserialize, Serialize};

/// An oriented bounding box.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obb<S> {
    query_from_obb: Isometry3<S>,
    obb_from_query: Isometry3<S>,
    half_extent: Vector3<S>,
}

/// TODO(nnmm): Remove
pub struct CachedAxesObb<S: BaseFloat> {
    pub obb: Obb<S>,
    pub separating_axes: CachedAxesIntersector<S>,
}

impl<S: BaseFloat + Bounded> CachedAxesObb<S> {
    pub fn new(obb: Obb<S>) -> Self {
        let unit_axes = [Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()];
        let separating_axes = obb
            .intersector()
            .cache_separating_axes(&unit_axes, &unit_axes);
        Self {
            obb,
            separating_axes,
        }
    }
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

    fn compute_edges(&self) -> ArrayVec<[Vector3<S>; 6]> {
        let mut edges = ArrayVec::new();
        edges.push((self.query_from_obb.rotation * Vector3::unit_x()).normalize());
        edges.push((self.query_from_obb.rotation * Vector3::unit_y()).normalize());
        edges.push((self.query_from_obb.rotation * Vector3::unit_z()).normalize());
        edges
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

    fn intersector(&self) -> Intersector<S> {
        let corners = self.compute_corners();
        let edges = self.compute_edges();
        Intersector {
            corners,
            edges: edges.clone(),
            face_normals: edges,
        }
    }
}

impl<S> PointCulling<S> for CachedAxesObb<S>
where
    S: 'static + Bounded + BaseFloat + Sync + Send,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        let Point3 { x, y, z } = &self.obb.obb_from_query * p;
        x.abs() <= self.obb.half_extent.x
            && y.abs() <= self.obb.half_extent.y
            && z.abs() <= self.obb.half_extent.z
    }

    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        self.separating_axes.intersect(&aabb.compute_corners()) != Relation::Out
    }
}
