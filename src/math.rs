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

use crate::proto;
use nalgebra::{Isometry3, Matrix4, Point3, RealField, Scalar, UnitQuaternion, Vector3};
use nav_types::{ECEF, WGS84};
use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::region::Region;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::str::FromStr;

/// Lower bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MIN_M: f64 = 6_352_800.0;
/// Upper bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
pub const EARTH_RADIUS_MAX_M: f64 = 6_384_400.0;

#[derive(Debug)]
pub struct ParseClosedIntervalError(String);

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
        AABB::new(aac_min.into(), aac_max.into())
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

/// Spatial relation between two objects.
/// Modeled after the collision crate.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialOrd, PartialEq)]
pub enum Relation {
    /// Completely inside.
    In,
    /// Crosses the boundary.
    Cross,
    /// Completely outside.
    Out,
}

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
    S: RealField,
    f64: From<S>,
{
    fn from(&self) -> s2::point::Point {
        s2::point::Point::from_coords(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }
}

impl<S> S2Point for Vector3<S>
where
    S: RealField,
    f64: From<S>,
{
    fn from(&self) -> s2::point::Point {
        s2::point::Point::from_coords(f64::from(self.x), f64::from(self.y), f64::from(self.z))
    }
}

pub trait PointCulling<S>: fmt::Debug + Sync + Send
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool;
    // TODO(nnmm): better name
    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation;
}

pub trait Cuboid<S: Scalar> {
    fn corners(&self) -> [Point3<S>; 8];
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

impl<S> Cuboid<S> for AABB<S>
where
    S: RealField,
{
    fn corners(&self) -> [Point3<S>; 8] {
        self.corners()
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
    fn intersects_aabb(&self, _aabb: &AABB<S>) -> Relation {
        Relation::In
    }
}
#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3<f64>,
    edge_length: f64,
}

impl Cube {
    pub fn bounding(aabb: &AABB<f64>) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: *aabb.min(),
            edge_length,
        }
    }

    pub fn to_aabb(&self) -> AABB<f64> {
        AABB::new(self.min(), self.max())
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
fn is_finite<S: RealField>(vec: &Vector3<S>) -> bool {
    vec.x.is_finite() && vec.y.is_finite() && vec.z.is_finite()
}

fn intersects_aabb<S: RealField + num_traits::Bounded>(
    corners: &[Point3<S>],
    separating_axes: &[Vector3<S>],
    aabb: &AABB<S>,
) -> Relation {
    // SAT algorithm
    // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
    for sep_axis in separating_axes.iter() {
        // Project the cube and the box onto that axis
        let mut cube_min_proj: S = num_traits::Bounded::max_value();
        let mut cube_max_proj: S = num_traits::Bounded::min_value();
        for corner in aabb.corners().iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box onto that axis
        let mut box_min_proj: S = num_traits::Bounded::max_value();
        let mut box_max_proj: S = num_traits::Bounded::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            box_min_proj = box_min_proj.min(corner_proj);
            box_max_proj = box_max_proj.max(corner_proj);
        }
        if box_min_proj > cube_max_proj || box_max_proj < cube_min_proj {
            return Relation::Out;
        }
    }
    Relation::Cross
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Obb<S: Scalar + RealField> {
    query_from_obb: Isometry3<S>,
    obb_from_query: Isometry3<S>,
    half_extent: Vector3<S>,
    corners: [Point3<S>; 8],
    separating_axes: Vec<Vector3<S>>,
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
            corners: Obb::precompute_corners(&query_from_obb, &half_extent),
            separating_axes: Obb::precompute_separating_axes(&query_from_obb.rotation),
            query_from_obb,
        }
    }

    pub fn transformed(&self, global_from_query: &Isometry3<S>) -> Self {
        Self::new(global_from_query * &self.query_from_obb, self.half_extent)
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
        intersects_aabb(&self.corners, &self.separating_axes, aabb)
    }
}

impl<S> Cuboid<S> for Obb<S>
where
    S: RealField,
{
    fn corners(&self) -> [Point3<S>; 8] {
        self.corners
    }
}

pub mod collision {
    use super::{Relation, AABB};
    use nalgebra::{Matrix4, RealField};
    use serde::{Deserialize, Serialize};

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Perspective<S: RealField> {
        matrix: Matrix4<S>,
    }

    impl<S: RealField> Perspective<S> {
        pub fn new(left: S, right: S, bottom: S, top: S, near: S, far: S) -> Self {
            assert!(
                left <= right,
                "`left` cannot be greater than `right`, found: left: {:?} right: {:?}",
                left,
                right
            );
            assert!(
                bottom <= top,
                "`bottom` cannot be greater than `top`, found: bottom: {:?} top: {:?}",
                bottom,
                top
            );
            assert!(
                near <= far,
                "`near` cannot be greater than `far`, found: near: {:?} far: {:?}",
                near,
                far
            );

            let two: S = nalgebra::convert(2.0);

            let c0r0 = (two * near) / (right - left);
            let c0r1 = nalgebra::zero();
            let c0r2 = nalgebra::zero();
            let c0r3 = nalgebra::zero();

            let c1r0 = nalgebra::zero();
            let c1r1 = (two * near) / (top - bottom);
            let c1r2 = nalgebra::zero();
            let c1r3 = nalgebra::zero();

            let c2r0 = (right + left) / (right - left);
            let c2r1 = (top + bottom) / (top - bottom);
            let c2r2 = -(far + near) / (far - near);
            let c2r3 = -S::one();

            let c3r0 = nalgebra::zero();
            let c3r1 = nalgebra::zero();
            let c3r2 = -(two * far * near) / (far - near);
            let c3r3 = nalgebra::zero();

            #[cfg_attr(rustfmt, rustfmt_skip)]
                let matrix = Matrix4::new(
                    c0r0, c0r1, c0r2, c0r3,
                    c1r0, c1r1, c1r2, c1r3,
                    c2r0, c2r1, c2r2, c2r3,
                    c3r0, c3r1, c3r2, c3r3,
                );
            Self { matrix }
        }

        pub fn as_matrix(&self) -> &Matrix4<S> {
            &self.matrix
        }

        pub fn inverse(&self) -> Matrix4<S> {
            let c0r0 = self.matrix[(0, 0)].recip();
            let c0r1 = nalgebra::zero();
            let c0r2 = nalgebra::zero();
            let c0r3 = nalgebra::zero();

            let c1r0 = nalgebra::zero();
            let c1r1 = self.matrix[(1, 1)].recip();
            let c1r2 = nalgebra::zero();
            let c1r3 = nalgebra::zero();

            let c2r0 = nalgebra::zero();
            let c2r1 = nalgebra::zero();
            let c2r2 = nalgebra::zero();
            let c2r3 = self.matrix[(3, 2)].recip();

            let c3r0 = self.matrix[(2, 0)] / self.matrix[(0, 0)];
            let c3r1 = self.matrix[(2, 1)] / self.matrix[(1, 1)];
            let c3r2 = -S::one();
            let c3r3 = self.matrix[(2, 2)] / self.matrix[(3, 2)];

            #[cfg_attr(rustfmt, rustfmt_skip)]
            Matrix4::new(
                c0r0, c0r1, c0r2, c0r3,
                c1r0, c1r1, c1r2, c1r3,
                c2r0, c2r1, c2r2, c2r3,
                c3r0, c3r1, c3r2, c3r3,
            )
        }
    }

    #[derive(Debug, Clone, Serialize, Deserialize)]
    pub struct Frustum<S: RealField> {
        matrix: Matrix4<S>,
    }

    impl<S: RealField> Frustum<S> {
        pub fn from_matrix4(matrix: Matrix4<S>) -> Self {
            Self { matrix }
        }

        pub fn contains(&self, aabb: &AABB<S>) -> Relation {
            super::contains_aabb(&self.matrix, aabb)
        }
    }
}

fn contains_point<S: RealField>(matrix: &Matrix4<S>, point: &Point3<S>) -> bool {
    let p_clip = matrix.transform_point(point);
    p_clip.coords.min() > nalgebra::convert(-1.0) && p_clip.coords.max() < nalgebra::convert(1.0)
}

fn contains_aabb<S: RealField>(matrix: &Matrix4<S>, aabb: &AABB<S>) -> Relation {
    let corners = aabb.corners();
    let rel = if contains_point(matrix, &corners[0]) {
        Relation::In
    } else {
        Relation::Out
    };
    for corner in corners[1..].iter() {
        match (rel, contains_point(matrix, corner)) {
            (Relation::In, false) | (Relation::Out, true) => return Relation::Cross,
            _ => (),
        }
    }
    rel
}

/// A frustum is defined in eye coordinates, where x points right, y points up,
/// and z points against the viewing direction. This is not how e.g. OpenCV
/// defines a camera coordinate system. To get from OpenCV camera coordinates
/// to eye coordinates, you need to rotate 180 deg around the x axis before
/// creating the perspective projection, see also the frustum unit test below.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frustum<S: RealField> {
    query_from_eye: Isometry3<S>,
    clip_from_eye: collision::Perspective<S>,
    query_from_clip: Matrix4<S>,
    clip_from_query: Matrix4<S>,
}

impl<S: RealField> Frustum<S> {
    pub fn new(query_from_eye: Isometry3<S>, clip_from_eye: collision::Perspective<S>) -> Self {
        let clip_from_query = clip_from_eye.as_matrix() * query_from_eye.inverse().to_homogeneous();
        let query_from_clip = query_from_eye.to_homogeneous() * clip_from_eye.inverse();
        Frustum {
            query_from_eye,
            clip_from_eye,
            query_from_clip,
            clip_from_query,
        }
    }

    pub fn transformed(&self, global_from_query: &Isometry3<S>) -> Self {
        Self::new(
            global_from_query * &self.query_from_eye,
            self.clip_from_eye.clone(),
        )
    }
}

impl<S> PointCulling<S> for Frustum<S>
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        let p_clip = self.clip_from_query.transform_point(point);
        p_clip.coords.min() > nalgebra::convert(-1.0)
            && p_clip.coords.max() < nalgebra::convert(1.0)
    }

    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation {
        let corners = aabb.corners();
        let rel = if self.contains(&corners[0]) {
            Relation::In
        } else {
            Relation::Out
        };
        for corner in corners[1..].iter() {
            match (rel, self.contains(corner)) {
                (Relation::In, false) | (Relation::Out, true) => return Relation::Cross,
                _ => (),
            }
        }
        rel
    }
}

impl<S> Cuboid<S> for Frustum<S>
where
    S: RealField,
{
    fn corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| self.query_from_clip.transform_point(&Point3::new(x, y, z));
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

impl<S> PointCulling<S> for CellUnion
where
    S: RealField,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from(S2Point::from(p)))
    }
    fn intersects_aabb(&self, aabb: &AABB<S>) -> Relation {
        let point_cells = aabb
            .corners()
            .iter()
            .map(|p| CellID::from(S2Point::from(p)))
            .collect();
        let mut cell_union = CellUnion(point_cells);
        cell_union.normalize();
        let rect = cell_union.rect_bound();
        let intersects = self
            .0
            .iter()
            .any(|cell_id| rect.intersects_cell(&Cell::from(cell_id)));
        if intersects {
            Relation::Cross
        } else {
            Relation::Out
        }
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
    use nalgebra::{Unit, UnitQuaternion, Vector3};
    use num_traits::One;

    #[test]
    fn test_inverse() {
        let persp = collision::Perspective::new(-0.123, 0.45, 0.04, 0.75, 1.0, 4.0);
        let reference_inverse = persp.as_matrix().try_inverse().unwrap();
        let inverse = persp.inverse();
        let diff = (reference_inverse - inverse).abs();
        assert!(diff.max() < 0.000001);
    }

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
        let bbox = AABB::new(bbox_min, bbox_max);
        assert_eq!(frustum.intersects_aabb(&bbox), Relation::In);
        assert!(frustum.contains(&bbox_min));
        assert!(frustum.contains(&bbox_max));
    }
}
