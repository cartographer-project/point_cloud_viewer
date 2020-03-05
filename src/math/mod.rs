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

use nalgebra::{Isometry3, Point3, RealField, Scalar, UnitQuaternion, Vector3};
use nav_types::{ECEF, WGS84};
use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::region::Region;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::str::FromStr;

pub mod base;
pub mod sat;
pub use base::*;
pub use sat::*;

/// Lower bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MIN_M: f64 = 6_352_800.0;
/// Upper bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MAX_M: f64 = 6_384_400.0;

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

/// An interval, intended to be read from a command line argument
/// and to be used in filtering the point cloud via an attribute.
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

/// Convenience trait to get a CellID from a Point3.
/// `From<Point3<S>>` cannot be used because of orphan rules.
pub trait FromPoint3<S: Scalar> {
    fn from_point(p: &Point3<S>) -> Self;
}

impl<S> FromPoint3<S> for s2::cellid::CellID
where
    S: Scalar,
    f64: From<S>,
{
    fn from_point(p: &Point3<S>) -> Self {
        s2::cellid::CellID::from(s2::point::Point::from_coords(
            f64::from(p.x),
            f64::from(p.y),
            f64::from(p.z),
        ))
    }
}

/// Implementation of PointCulling to return all points
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AllPoints {}

impl<S> PointCulling<S> for AllPoints
where
    S: RealField + num_traits::Bounded,
{
    fn contains(&self, _p: &Point3<S>) -> bool {
        true
    }
}

pub fn cell_union_intersects_aabb<S>(
    cell_union: &CellUnion,
    aabb: &crate::geometry::Aabb<S>,
) -> Relation
where
    S: RealField,
    f64: From<S>,
{
    let aabb_corner_cells = aabb
        .corners()
        .iter()
        .map(|p| CellID::from_point(p))
        .collect();
    let mut aabb_cell_union = CellUnion(aabb_corner_cells);
    aabb_cell_union.normalize();
    let rect = aabb_cell_union.rect_bound();
    let intersects = cell_union
        .0
        .iter()
        .any(|cell_id| rect.intersects_cell(&Cell::from(cell_id)));
    if intersects {
        Relation::Cross
    } else {
        Relation::Out
    }
}

impl<S> PointCulling<S> for CellUnion
where
    S: RealField,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from_point(p))
    }
}

// Returns transform needed to go from ECEF to local frame with the specified origin where
// the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
// used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
pub fn local_frame_from_lat_lng(lat: f64, lon: f64) -> Isometry3<f64> {
    let lat_lng_alt = WGS84::new(lat, lon, 0.0);
    let origin = ECEF::from(lat_lng_alt);
    let origin_vector = Vector3::new(origin.x(), origin.y(), origin.z());

    let rot_1 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -std::f64::consts::FRAC_PI_2);
    let rot_2 = UnitQuaternion::from_axis_angle(
        &Vector3::y_axis(),
        lat_lng_alt.latitude() - std::f64::consts::FRAC_PI_2,
    );
    let rot_3 = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -lat_lng_alt.longitude());

    let rotation = rot_1 * rot_2 * rot_3;

    Isometry3::from_parts(rotation.transform_vector(&-origin_vector).into(), rotation)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{collision, Aabb, Frustum};
    use nalgebra::{UnitQuaternion, Vector3};

    #[test]
    fn test_inverse() {
        let persp = collision::Perspective::new(-0.123, 0.45, 0.04, 0.75, 1.0, 4.0);
        let reference_inverse = persp.as_matrix().try_inverse().unwrap();
        let inverse = persp.inverse();
        let diff = (reference_inverse - inverse).abs();
        assert!(diff.max() < 0.000001, "diff.max() is {}", diff.max());
    }

    #[test]
    fn test_frustum_intersects_aabb() {
        let rot: Isometry3<f64> = nalgebra::convert(UnitQuaternion::from_axis_angle(
            &Vector3::x_axis(),
            std::f64::consts::PI,
        ));
        let perspective = collision::Perspective::new(
            /* left */ -0.5, /* right */ 0.0, /* bottom */ -0.5, /* top */ 0.0,
            /* near */ 1.0, /* far */ 4.0,
        );
        let frustum = Frustum::new(rot, perspective);
        let bbox_min = Point3::new(-0.5, 0.25, 1.5);
        let bbox_max = Point3::new(-0.25, 0.5, 3.5);
        let bbox = Aabb::new(bbox_min, bbox_max);
        assert_eq!(
            frustum.intersector().intersect(&bbox.intersector()),
            Relation::In
        );
        assert!(frustum.contains(&bbox_min));
        assert!(frustum.contains(&bbox_max));
    }
}
