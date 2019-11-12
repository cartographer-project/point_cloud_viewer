// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use cgmath::{
    BaseFloat, BaseNum, Decomposed, Deg, EuclideanSpace, InnerSpace, Matrix3, Matrix4, Perspective,
    Point3, Quaternion, Rotation, Transform, Vector3, Zero,
};
use collision::{Aabb, Aabb3, Contains, Relation};
use nav_types::{ECEF, WGS84};
use num_traits::identities::One;
use num_traits::Float;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use std::ops::Mul;

/// Lower bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MIN_M: f64 = 6_352_800.0;
/// Upper bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MAX_M: f64 = 6_384_400.0;

pub fn clamp<T>(value: Vector3<T>, low: Vector3<T>, high: Vector3<T>) -> Vector3<T>
where
    T: BaseNum,
{
    let mut clamped = value;
    for i in 0..3 {
        clamped[i] = num::clamp(value[i], low[i], high[i]);
    }
    clamped
}

pub trait PointCulling<S>: Debug + Sync + Send
where
    S: BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool;
    // TODO(catevita): return Relation
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool;
    fn transformed(&self, global_from_query: &Isometry3<S>) -> Box<dyn PointCulling<S>>;
}

pub trait Cuboid<S> {
    fn corners(&self) -> [Point3<S>; 8];
}

impl<S> PointCulling<S> for Aabb3<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        let separating_axes = &[Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()];
        intersects_aabb3(&self.to_corners(), separating_axes, aabb)
    }
    fn contains(&self, p: &Point3<S>) -> bool {
        Contains::contains(self, p)
    }
    fn transformed(&self, global_from_query: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Obb::from(self).transformed(global_from_query)
    }
}

impl<S> Cuboid<S> for Aabb3<S>
where
    S: BaseFloat,
{
    fn corners(&self) -> [Point3<S>; 8] {
        self.to_corners()
    }
}

/// Implementation of PointCulling to return all points
#[derive(Clone, Debug)]
pub struct AllPoints {}

impl<S> PointCulling<S> for AllPoints
where
    S: BaseFloat + Sync + Send,
{
    fn intersects_aabb3(&self, _aabb: &Aabb3<S>) -> bool {
        true
    }
    fn contains(&self, _p: &Point3<S>) -> bool {
        true
    }
    fn transformed(&self, _: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(Self {})
    }
}
#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3<f64>,
    edge_length: f64,
}

impl Cube {
    pub fn bounding(aabb: &Aabb3<f64>) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: aabb.min,
            edge_length,
        }
    }

    pub fn to_aabb3(&self) -> Aabb3<f64> {
        Aabb3::new(self.min(), self.max())
    }

    pub fn new(min: Point3<f64>, edge_length: f64) -> Self {
        Cube { min, edge_length }
    }

    pub fn edge_length(&self) -> f64 {
        self.edge_length
    }

    pub fn min(&self) -> Point3<f64> {
        self.min
    }

    pub fn max(&self) -> Point3<f64> {
        Point3::new(
            self.min.x + self.edge_length,
            self.min.y + self.edge_length,
            self.min.z + self.edge_length,
        )
    }

    /// The center of the box.
    pub fn center(&self) -> Vector3<f64> {
        let min = self.min();
        let max = self.max();
        Vector3::new(
            (min.x + max.x) / 2.,
            (min.y + max.y) / 2.,
            (min.z + max.z) / 2.,
        )
    }
}

// This guards against the separating axes being NaN, which may happen when the
// orientation aligns with the unit axes.
fn is_finite<S: BaseFloat>(vec: &Vector3<S>) -> bool {
    vec.x.is_finite() && vec.y.is_finite() && vec.z.is_finite()
}

fn intersects_aabb3<S: BaseFloat>(
    corners: &[Point3<S>],
    separating_axes: &[Vector3<S>],
    aabb: &Aabb3<S>,
) -> bool {
    // SAT algorithm
    // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
    for sep_axis in separating_axes.iter() {
        // Project the cube and the box onto that axis
        let mut cube_min_proj: S = Float::max_value();
        let mut cube_max_proj: S = Float::min_value();
        for corner in aabb.to_corners().iter() {
            let corner_proj = corner.dot(*sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box onto that axis
        let mut box_min_proj: S = Float::max_value();
        let mut box_max_proj: S = Float::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.dot(*sep_axis);
            box_min_proj = box_min_proj.min(corner_proj);
            box_max_proj = box_max_proj.max(corner_proj);
        }
        if box_min_proj > cube_max_proj || box_max_proj < cube_min_proj {
            return false;
        }
    }
    true
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Isometry3<S> {
    pub rotation: Quaternion<S>,
    pub translation: Vector3<S>,
}

impl<S: BaseFloat> Mul for Isometry3<S> {
    type Output = Self;
    fn mul(self, _rhs: Self) -> Self {
        Self::new(
            self.rotation * _rhs.rotation,
            self.rotation * _rhs.translation + self.translation,
        )
    }
}

impl<'a, S: BaseFloat> Mul for &'a Isometry3<S> {
    type Output = Isometry3<S>;
    fn mul(self, _rhs: &'a Isometry3<S>) -> Isometry3<S> {
        Isometry3::new(
            self.rotation * _rhs.rotation,
            self.rotation * _rhs.translation + self.translation,
        )
    }
}

impl<S: BaseFloat> Mul<Point3<S>> for Isometry3<S> {
    type Output = Point3<S>;
    fn mul(self, _rhs: Point3<S>) -> Point3<S> {
        Point3::from_vec(self.rotation * _rhs.to_vec()) + self.translation
    }
}

impl<'a, S: BaseFloat> Mul<&'a Point3<S>> for &'a Isometry3<S> {
    type Output = Point3<S>;
    fn mul(self, _rhs: &'a Point3<S>) -> Point3<S> {
        Point3::from_vec(self.rotation * _rhs.to_vec()) + self.translation
    }
}

impl<S: BaseFloat> Into<Decomposed<Vector3<S>, Quaternion<S>>> for Isometry3<S> {
    fn into(self) -> Decomposed<Vector3<S>, Quaternion<S>> {
        Decomposed {
            scale: S::one(),
            rot: self.rotation,
            disp: self.translation,
        }
    }
}

impl<S: BaseFloat> From<Decomposed<Vector3<S>, Quaternion<S>>> for Isometry3<S> {
    fn from(decomposed: Decomposed<Vector3<S>, Quaternion<S>>) -> Self {
        Self::new(decomposed.rot, decomposed.disp)
    }
}

impl<S: BaseFloat> Isometry3<S> {
    pub fn new(rotation: Quaternion<S>, translation: Vector3<S>) -> Self {
        Isometry3 {
            rotation,
            translation,
        }
    }

    /// Returns the identity transformation.
    pub fn one() -> Self {
        Isometry3 {
            rotation: Quaternion::one(),
            translation: Vector3::zero(),
        }
    }

    pub fn inverse(&self) -> Self {
        Self::new(
            self.rotation.conjugate(),
            -(self.rotation.conjugate() * self.translation),
        )
    }
}

#[derive(Debug, Clone)]
pub struct Obb<S> {
    pub query_from_obb: Isometry3<S>,
    obb_from_query: Isometry3<S>,
    pub half_extent: Vector3<S>,
    corners: [Point3<S>; 8],
    separating_axes: Vec<Vector3<S>>,
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
            half_extent,
            corners: Obb::precompute_corners(&query_from_obb, &half_extent),
            separating_axes: Obb::precompute_separating_axes(&query_from_obb.rotation),
            query_from_obb,
        }
    }

    fn precompute_corners(
        query_from_obb: &Isometry3<S>,
        half_extent: &Vector3<S>,
    ) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| query_from_obb * &Point3::new(x, y, z);
        [
            corner_from(-half_extent.x, -half_extent.y, -half_extent.z),
            corner_from(half_extent.x, -half_extent.y, -half_extent.z),
            corner_from(-half_extent.x, half_extent.y, -half_extent.z),
            corner_from(half_extent.x, half_extent.y, -half_extent.z),
            corner_from(-half_extent.x, -half_extent.y, half_extent.z),
            corner_from(half_extent.x, -half_extent.y, half_extent.z),
            corner_from(-half_extent.x, half_extent.y, half_extent.z),
            corner_from(half_extent.x, half_extent.y, half_extent.z),
        ]
    }

    fn precompute_separating_axes(query_from_obb: &Quaternion<S>) -> Vec<Vector3<S>> {
        let unit_x = Vector3::unit_x();
        let unit_y = Vector3::unit_y();
        let unit_z = Vector3::unit_z();
        let rot_x = query_from_obb * unit_x;
        let rot_y = query_from_obb * unit_y;
        let rot_z = query_from_obb * unit_z;
        let mut separating_axes = vec![unit_x, unit_y, unit_z];
        for axis in &[
            rot_x,
            rot_y,
            rot_z,
            unit_x.cross(rot_x).normalize(),
            unit_x.cross(rot_y).normalize(),
            unit_x.cross(rot_z).normalize(),
            unit_y.cross(rot_x).normalize(),
            unit_y.cross(rot_y).normalize(),
            unit_y.cross(rot_z).normalize(),
            unit_z.cross(rot_x).normalize(),
            unit_z.cross(rot_y).normalize(),
            unit_z.cross(rot_z).normalize(),
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
    S: 'static + BaseFloat + Sync + Send,
{
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        intersects_aabb3(&self.corners, &self.separating_axes, aabb)
    }
    fn contains(&self, p: &Point3<S>) -> bool {
        let Point3 { x, y, z } = &self.obb_from_query * p;
        x.abs() <= self.half_extent.x
            && y.abs() <= self.half_extent.y
            && z.abs() <= self.half_extent.z
    }
    fn transformed(&self, global_from_query: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(Self::new(
            global_from_query * &self.query_from_obb,
            self.half_extent,
        ))
    }
}

impl<S> Cuboid<S> for Obb<S>
where
    S: BaseFloat,
{
    fn corners(&self) -> [Point3<S>; 8] {
        self.corners
    }
}

/// A frustum is defined in eye coordinates, where x points right, y points up,
/// and z points against the viewing direction. This is not how e.g. OpenCV
/// defines a camera coordinate system. To get from OpenCV camera coordinates
/// to eye coordinates, you need to rotate 180 deg around the x axis before
/// creating the perspective projection, see also the frustum unit test below.
#[derive(Debug, Clone)]
pub struct Frustum<S: BaseFloat> {
    query_from_eye: Isometry3<S>,
    clip_from_eye: Perspective<S>,
    query_from_clip: Matrix4<S>,
    frustum: collision::Frustum<S>,
}

impl<S: BaseFloat> Frustum<S> {
    pub fn new(query_from_eye: Isometry3<S>, clip_from_eye: Perspective<S>) -> Self {
        let eye_from_query: Decomposed<Vector3<S>, Quaternion<S>> = query_from_eye.inverse().into();
        let clip_from_query =
            Matrix4::<S>::from(clip_from_eye) * Matrix4::<S>::from(eye_from_query);
        let query_from_clip = clip_from_query.inverse_transform().unwrap();
        let frustum = collision::Frustum::from_matrix4(clip_from_query).unwrap();
        Frustum {
            query_from_eye,
            clip_from_eye,
            query_from_clip,
            frustum,
        }
    }
}

impl<S> PointCulling<S> for Frustum<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        match self.frustum.contains(point) {
            Relation::Cross => true,
            Relation::In => true,
            Relation::Out => false,
        }
    }
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        match self.frustum.contains(aabb) {
            Relation::Cross => true,
            Relation::In => true,
            Relation::Out => false,
        }
    }
    fn transformed(&self, global_from_query: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(Self::new(
            global_from_query * &self.query_from_eye,
            self.clip_from_eye,
        ))
    }
}

impl<S> Cuboid<S> for Frustum<S>
where
    S: BaseFloat,
{
    fn corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| self.query_from_clip.transform_point(Point3::new(x, y, z));
        [
            corner_from(-S::one(), -S::one(), -S::one()),
            corner_from(-S::one(), -S::one(), S::one()),
            corner_from(-S::one(), S::one(), -S::one()),
            corner_from(-S::one(), S::one(), S::one()),
            corner_from(S::one(), -S::one(), -S::one()),
            corner_from(S::one(), -S::one(), S::one()),
            corner_from(S::one(), S::one(), -S::one()),
            corner_from(S::one(), S::one(), S::one()),
        ]
    }
}

// Returns transform needed to go from ECEF to local frame with the specified origin where
// the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
// used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
pub fn local_frame_from_lat_lng(lat: f64, lon: f64) -> Isometry3<f64> {
    const PI_HALF: Deg<f64> = Deg(90.0);
    let lat_lng_alt = WGS84::new(lat, lon, 0.0);
    let origin = ECEF::from(lat_lng_alt);
    let origin_vector = Vector3::new(origin.x(), origin.y(), origin.z());
    let rotation_matrix = Matrix3::from_angle_z(-PI_HALF)
        * Matrix3::from_angle_y(Deg(lat_lng_alt.latitude_degrees()) - PI_HALF)
        * Matrix3::from_angle_z(Deg(-lat_lng_alt.longitude_degrees()));
    let rotation = Quaternion::from(rotation_matrix);

    let frame = Decomposed {
        scale: 1.0,
        rot: rotation,
        disp: rotation.rotate_vector(-origin_vector),
    };
    Isometry3::from(frame)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::{Rad, Rotation3, Zero};

    #[test]
    fn test_obb_intersects_aabb3() {
        let zero_rot: Quaternion<f64> = Rotation3::from_angle_z(Rad(0.0));
        let fourty_five_deg_rot: Quaternion<f64> =
            Rotation3::from_angle_z(Rad(std::f64::consts::PI / 4.0));
        let arbitrary_rot: Quaternion<f64> =
            Rotation3::from_axis_angle(Vector3::new(0.2, 0.5, -0.7), Rad(0.123));
        let translation = Vector3::new(0.0, 0.0, 0.0);
        let half_extent = Vector3::new(1.0, 2.0, 3.0);
        let zero_obb = Obb::new(Isometry3::new(zero_rot, translation), half_extent);
        let fourty_five_deg_obb = Obb::new(
            Isometry3::new(fourty_five_deg_rot, translation),
            half_extent,
        );
        let arbitrary_obb = Obb::new(Isometry3::new(arbitrary_rot, translation), half_extent);
        let bbox = Aabb3::new(Point3::new(0.5, 1.0, -3.0), Point3::new(1.5, 3.0, 3.0));
        assert_eq!(zero_obb.separating_axes.len(), 3);
        assert_eq!(zero_obb.intersects_aabb3(&bbox), true);
        assert_eq!(fourty_five_deg_obb.separating_axes.len(), 5);
        assert_eq!(fourty_five_deg_obb.intersects_aabb3(&bbox), false);
        assert_eq!(arbitrary_obb.separating_axes.len(), 15);
    }

    #[test]
    fn test_frustum_intersects_aabb3() {
        let rot = Isometry3::<f64> {
            rotation: Quaternion::from_angle_x(Rad(std::f64::consts::PI)),
            translation: Vector3::zero(),
        };
        let perspective = Perspective::<f64> {
            left: -0.5,
            right: 0.0,
            bottom: -0.5,
            top: 0.0,
            near: 1.0,
            far: 4.0,
        };
        let frustum = Frustum::new(rot, perspective);
        let bbox_min = Point3::new(-0.5, 0.25, 1.5);
        let bbox_max = Point3::new(-0.25, 0.5, 3.5);
        let bbox = Aabb3::new(bbox_min, bbox_max);
        assert!(frustum.intersects_aabb3(&bbox));
        assert!(frustum.contains(&bbox_min));
        assert!(frustum.contains(&bbox_max));
    }
}
