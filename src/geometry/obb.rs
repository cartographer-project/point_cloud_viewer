//! A bounding box with an arbitrary 3D pose.

use super::aabb::Aabb;
use crate::math::base::{HasAabbIntersector, PointCulling};
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Intersector};
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Point3, Unit, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use std::iter::FromIterator;

/// An oriented bounding box.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obb {
    query_from_obb: Isometry3<f64>,
    obb_from_query: Isometry3<f64>,
    half_extent: Vector3<f64>,
}

impl From<&Aabb> for Obb {
    fn from(aabb: &Aabb) -> Self {
        Obb::new(
            Isometry3::from_parts(aabb.center().coords.into(), UnitQuaternion::identity()),
            aabb.diag() * 0.5,
        )
    }
}

impl From<Aabb> for Obb {
    fn from(aabb: Aabb) -> Self {
        Self::from(&aabb)
    }
}

impl Obb {
    pub fn new(query_from_obb: Isometry3<f64>, half_extent: Vector3<f64>) -> Self {
        Obb {
            obb_from_query: query_from_obb.inverse(),
            half_extent,
            query_from_obb,
        }
    }

    pub fn transformed(&self, global_from_query: &Isometry3<f64>) -> Self {
        Self::new(global_from_query * self.query_from_obb, self.half_extent)
    }
}

impl ConvexPolyhedron<f64> for Obb {
    fn compute_corners(&self) -> [Point3<f64>; 8] {
        let corner_from = |x, y, z| self.query_from_obb * Point3::new(x, y, z);
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

    fn intersector(&self) -> Intersector<f64> {
        let mut edges = ArrayVec::new();
        edges.push(Unit::new_normalize(self.query_from_obb * Vector3::x()));
        edges.push(Unit::new_normalize(self.query_from_obb * Vector3::y()));
        edges.push(Unit::new_normalize(self.query_from_obb * Vector3::z()));
        let face_normals = ArrayVec::from_iter(edges.clone());
        Intersector {
            corners: self.compute_corners(),
            edges,
            face_normals,
        }
    }
}

has_aabb_intersector_for_convex_polyhedron!(Obb);

impl PointCulling for Obb {
    fn contains(&self, p: &Point3<f64>) -> bool {
        let p = self.obb_from_query * p;
        p.x.abs() <= self.half_extent.x
            && p.y.abs() <= self.half_extent.y
            && p.z.abs() <= self.half_extent.z
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::sat::Relation;
    use nalgebra::{Unit, UnitQuaternion, Vector3};
    use num_traits::One;

    #[test]
    fn test_obb_intersects_aabb() {
        let zero_rot = UnitQuaternion::one();
        let fourty_five_deg_rot: UnitQuaternion<f64> =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::PI / 4.0);
        let arbitrary_rot: UnitQuaternion<f64> = UnitQuaternion::from_axis_angle(
            &Unit::new_normalize(Vector3::new(0.2, 0.5, -0.7)),
            0.123,
        );
        let translation = Vector3::new(0.0, 0.0, 0.0).into();
        let half_extent = Vector3::new(1.0, 2.0, 3.0).into();
        // Object to intersection-test against
        let bbox = Aabb::new(Point3::new(0.5, 1.0, -3.0), Point3::new(1.5, 3.0, 3.0));

        let zero_obb = Obb::new(Isometry3::from_parts(translation, zero_rot), half_extent);
        let zero_obb_isec = zero_obb.intersector().cache_separating_axes_for_aabb();
        assert_eq!(zero_obb_isec.axes.len(), 3);
        assert_eq!(
            zero_obb_isec.intersect(&bbox.compute_corners()),
            Relation::Cross
        );

        let fourty_five_deg_obb = Obb::new(
            Isometry3::from_parts(translation, fourty_five_deg_rot),
            half_extent,
        );
        let fourty_five_deg_obb_isec = fourty_five_deg_obb
            .intersector()
            .cache_separating_axes_for_aabb();
        assert_eq!(fourty_five_deg_obb_isec.axes.len(), 5);
        assert_eq!(
            fourty_five_deg_obb_isec.intersect(&bbox.compute_corners()),
            Relation::Out
        );

        let arbitrary_obb = Obb::new(
            Isometry3::from_parts(translation, arbitrary_rot),
            half_extent,
        );
        let arbitrary_obb_isec = arbitrary_obb.intersector().cache_separating_axes_for_aabb();
        assert_eq!(arbitrary_obb_isec.axes.len(), 15);
    }
}
