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
    BaseFloat, BaseNum, Decomposed, Deg, EuclideanSpace, Matrix3, Point3, Quaternion, Rotation,
    Vector3, Zero,
};
use collision::{Aabb3, Contains};
use nav_types::{ECEF, WGS84};
use num_traits::identities::One;

use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::region::Region;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::Mul;
use std::str::FromStr;

pub mod sat;
pub use sat::*;

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

#[derive(Debug)]
pub struct ParseClosedIntervalError(String);

impl std::error::Error for ParseClosedIntervalError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}

impl fmt::Display for ParseClosedIntervalError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

impl From<std::num::ParseIntError> for ParseClosedIntervalError {
    fn from(error: std::num::ParseIntError) -> Self {
        Self(error.to_string())
    }
}

impl From<std::num::ParseFloatError> for ParseClosedIntervalError {
    fn from(error: std::num::ParseFloatError) -> Self {
        Self(error.to_string())
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct ClosedInterval<T> {
    lower_bound: T,
    upper_bound: T,
}

impl<T> ClosedInterval<T>
where
    T: PartialOrd,
{
    pub fn new(lower_bound: T, upper_bound: T) -> Self {
        assert!(
            lower_bound <= upper_bound,
            "Lower bound needs to be smaller or equal to upper bound."
        );
        Self {
            lower_bound,
            upper_bound,
        }
    }

    pub fn contains(self, value: T) -> bool {
        self.lower_bound <= value && value <= self.upper_bound
    }
}

impl<T> FromStr for ClosedInterval<T>
where
    T: std::str::FromStr,
    ParseClosedIntervalError: From<T::Err>,
{
    type Err = ParseClosedIntervalError;

    fn from_str(s: &str) -> std::result::Result<Self, Self::Err> {
        let bounds: Vec<&str> = s.split(',').collect();
        if bounds.len() != 2 {
            return Err(ParseClosedIntervalError(
                "An interval needs to be defined by exactly 2 bounds.".into(),
            ));
        }
        Ok(ClosedInterval {
            lower_bound: bounds[0].parse()?,
            upper_bound: bounds[1].parse()?,
        })
    }
}

pub trait S2Point {
    fn from(&self) -> s2::point::Point;
}

impl<S> S2Point for Point3<S>
where
    S: BaseFloat,
    f64: From<S>,
{
    fn from(&self) -> s2::point::Point {
        s2::point::Point::from_coords(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }
}

impl<S> S2Point for Vector3<S>
where
    S: BaseFloat,
    f64: From<S>,
{
    fn from(&self) -> s2::point::Point {
        s2::point::Point::from_coords(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }
}

pub trait PointCulling<S>: fmt::Debug + Sync + Send
where
    S: BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool;
    // TODO(catevita): return Relation
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool;
}

pub trait Cuboid<S> {
    fn corners(&self) -> [Point3<S>; 8];
}

impl<S> PointCulling<S> for Aabb3<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        Contains::contains(self, p)
    }
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        let separating_axes = &[Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()];
        intersects_aabb3(&self.to_corners(), separating_axes, aabb)
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
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AllPoints {}

impl<S> PointCulling<S> for AllPoints
where
    S: BaseFloat + Sync + Send,
{
    fn contains(&self, _p: &Point3<S>) -> bool {
        true
    }
    fn intersects_aabb3(&self, _aabb: &Aabb3<S>) -> bool {
        true
    }
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

impl<S: BaseFloat> From<nalgebra::Isometry3<S>> for Isometry3<S>
where
    S: nalgebra::RealField,
{
    fn from(isometry: nalgebra::Isometry3<S>) -> Self {
        let r = isometry.rotation.coords;
        let t = isometry.translation;
        Self::new(
            // nalgebra quaternion vec has `[ x, y, z, w ]` storage order
            Quaternion::new(r[3], r[0], r[1], r[2]),
            Vector3::new(t.x, t.y, t.z),
        )
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

impl<S> PointCulling<S> for CellUnion
where
    S: BaseFloat + Sync + Send,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from(S2Point::from(p)))
    }
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        let point_cells = aabb
            .to_corners()
            .iter()
            .map(|p| CellID::from(S2Point::from(p)))
            .collect();
        let mut cell_union = CellUnion(point_cells);
        cell_union.normalize();
        let rect = cell_union.rect_bound();
        self.0
            .iter()
            .any(|cell_id| rect.intersects_cell(&Cell::from(cell_id)))
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
    use crate::geometry::{Frustum, Obb};
    use cgmath::{Perspective, Rad, Rotation3, Zero};

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
