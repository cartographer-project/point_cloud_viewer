use super::aabb::AABB;
use super::base::{PointCulling, Relation};
use super::sat::{intersects_aabb, ConvexPolyhedron};
use nalgebra::{Isometry3, Point3, RealField, Scalar, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obb<S: Scalar + RealField> {
    query_from_obb: Isometry3<S>,
    obb_from_query: Isometry3<S>,
    half_extent: Vector3<S>,
}

impl<S: RealField> From<&AABB<S>> for Obb<S> {
    fn from(aabb: &AABB<S>) -> Self {
        Obb::new(
            Isometry3::from_parts(aabb.center().coords.into(), UnitQuaternion::identity()),
            aabb.max() - aabb.center(),
        )
    }
}

impl<S: RealField> From<AABB<S>> for Obb<S> {
    fn from(aabb: AABB<S>) -> Self {
        Self::from(&aabb)
    }
}

impl<S: RealField> Obb<S> {
    pub fn new(query_from_obb: Isometry3<S>, half_extent: Vector3<S>) -> Self {
        Obb {
            obb_from_query: query_from_obb.inverse(),
            half_extent,
            query_from_obb,
        }
    }

    pub fn transformed(&self, global_from_query: &Isometry3<S>) -> Self {
        Self::new(global_from_query * &self.query_from_obb, self.half_extent)
    }

    fn precompute_separating_axes(query_from_obb: &UnitQuaternion<S>) -> Vec<Vector3<S>> {
        let unit_x = Vector3::x();
        let unit_y = Vector3::y();
        let unit_z = Vector3::z();
        let rot_x = query_from_obb * unit_x;
        let rot_y = query_from_obb * unit_y;
        let rot_z = query_from_obb * unit_z;
        let mut separating_axes = vec![unit_x, unit_y, unit_z];
        for axis in &[
            rot_x,
            rot_y,
            rot_z,
            unit_x.cross(&rot_x).normalize(),
            unit_x.cross(&rot_y).normalize(),
            unit_x.cross(&rot_z).normalize(),
            unit_y.cross(&rot_x).normalize(),
            unit_y.cross(&rot_y).normalize(),
            unit_y.cross(&rot_z).normalize(),
            unit_z.cross(&rot_x).normalize(),
            unit_z.cross(&rot_y).normalize(),
            unit_z.cross(&rot_z).normalize(),
        ] {
            let is_finite_and_non_parallel = is_finite(&axis)
                && separating_axes.iter().all(|elem| {
                    (elem - axis).magnitude() > S::default_epsilon()
                        && (elem + axis).magnitude() > S::default_epsilon()
                });
            if is_finite_and_non_parallel {
                separating_axes.push(*axis);
            }
        }
        separating_axes
    }
}

impl<S> PointCulling<S> for Obb<S>
where
    S: RealField,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        let p = &self.obb_from_query * p;
        p.x.abs() <= self.half_extent.x
            && p.y.abs() <= self.half_extent.y
            && p.z.abs() <= self.half_extent.z
    }
    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation {
        unimplemented!()
    }
}

impl<S> ConvexPolyhedron<S> for Obb<S>
where
    S: RealField,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| self.query_from_obb * &Point3::new(x, y, z);
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

    fn compute_edges(&self) -> [Option<Vector3<S>>; 6] {
        [
            Some(self.query_from_obb * Vector3::x()),
            Some(self.query_from_obb * Vector3::y()),
            Some(self.query_from_obb * Vector3::z()),
            None,
            None,
            None,
        ]
    }

    fn compute_face_normals(&self) -> [Option<Vector3<S>>; 6] {
        self.compute_edges()
    }
}

// This guards against the separating axes being NaN, which may happen when the
// orientation aligns with the unit axes.
fn is_finite<S: RealField>(vec: &Vector3<S>) -> bool {
    vec.x.is_finite() && vec.y.is_finite() && vec.z.is_finite()
}

#[cfg(test)]
mod tests {
    use super::*;
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
        let zero_obb = Obb::new(Isometry3::from_parts(translation, zero_rot), half_extent);
        let fourty_five_deg_obb = Obb::new(
            Isometry3::from_parts(translation, fourty_five_deg_rot),
            half_extent,
        );
        let arbitrary_obb = Obb::new(
            Isometry3::from_parts(translation, arbitrary_rot),
            half_extent,
        );
        let bbox = AABB::new(Point3::new(0.5, 1.0, -3.0), Point3::new(1.5, 3.0, 3.0));
        assert_eq!(zero_obb.separating_axes.len(), 3);
        assert_eq!(zero_obb.intersects_aabb(&bbox), Relation::Cross);
        assert_eq!(fourty_five_deg_obb.separating_axes.len(), 5);
        assert_eq!(fourty_five_deg_obb.intersects_aabb(&bbox), Relation::Out);
        assert_eq!(arbitrary_obb.separating_axes.len(), 15);
    }
}
