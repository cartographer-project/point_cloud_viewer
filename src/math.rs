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
    BaseFloat, BaseNum, Decomposed, Deg, EuclideanSpace, InnerSpace, Matrix3, Matrix4, Point3,
    Quaternion, Rotation, Vector2, Vector3, Vector4,
};
use collision::{Aabb, Aabb3, Contains, Relation};
use nav_types::{ECEF, WGS84};
use num_traits::identities::One;
use num_traits::Float;
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

pub trait PointCulling<S>: objekt::Clone + Debug + Sync + Send
where
    S: BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool;
    // TODO(catevita): return Relation
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool;
    fn transform(&self, isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>>;
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

    fn transform(&self, isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Obb::from(*self).transform(isometry)
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
    fn transform(&self, _isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(AllPoints {})
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
        // Project the cube and the box/beam onto that axis
        let mut cube_min_proj: S = Float::max_value();
        let mut cube_max_proj: S = Float::min_value();
        for corner in aabb.to_corners().iter() {
            let corner_proj = corner.dot(*sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box/beam onto that axis
        let mut beam_min_proj: S = Float::max_value();
        let mut beam_max_proj: S = Float::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.dot(*sep_axis);
            beam_min_proj = beam_min_proj.min(corner_proj);
            beam_max_proj = beam_max_proj.max(corner_proj);
        }
        if beam_min_proj > cube_max_proj || beam_max_proj < cube_min_proj {
            return false;
        }
    }
    true
}

#[derive(Debug, Clone)]
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

impl<S: BaseFloat> Mul<Vector3<S>> for Isometry3<S> {
    type Output = Vector3<S>;
    fn mul(self, _rhs: Vector3<S>) -> Vector3<S> {
        self.rotation * _rhs + self.translation
    }
}

impl<'a, S: BaseFloat> Mul<&'a Vector3<S>> for &'a Isometry3<S> {
    type Output = Vector3<S>;
    fn mul(self, _rhs: &'a Vector3<S>) -> Vector3<S> {
        self.rotation * _rhs + self.translation
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

    pub fn inverse(&self) -> Self {
        Self::new(
            self.rotation.conjugate(),
            -(self.rotation.conjugate() * self.translation),
        )
    }
}

#[derive(Debug, Clone)]
pub struct Obb<S> {
    pub isometry: Isometry3<S>,
    pub isometry_inv: Isometry3<S>,
    pub half_extent: Vector3<S>,
    pub corners: [Point3<S>; 8],
    pub separating_axes: Vec<Vector3<S>>,
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

/// Creates an object oriented box by intersecting the oriented beam with the minimum and maximum
/// earth radius. The OBB has the same orientation and extents as the beam.
impl<S: BaseFloat> From<&OrientedBeam<S>> for Obb<S> {
    fn from(beam: &OrientedBeam<S>) -> Self {
        let earth_radius_mean = S::from(0.5 * (EARTH_RADIUS_MIN_M + EARTH_RADIUS_MAX_M)).unwrap();
        let z_off = earth_radius_mean - beam.isometry.translation.magnitude();
        let vec_off = Vector3::new(S::zero(), S::zero(), z_off);
        let isometry = Isometry3::new(beam.isometry.rotation, &beam.isometry * &vec_off);
        let z_half_extent = S::from(EARTH_RADIUS_MAX_M).unwrap() - earth_radius_mean;
        let half_extent = Vector3::new(beam.half_extent.x, beam.half_extent.y, z_half_extent);
        Self::new(isometry, half_extent)
    }
}

impl<S: BaseFloat> From<OrientedBeam<S>> for Obb<S> {
    fn from(beam: OrientedBeam<S>) -> Self {
        Self::from(&beam)
    }
}

impl<S: BaseFloat> Obb<S> {
    pub fn new(isometry: Isometry3<S>, half_extent: Vector3<S>) -> Self {
        Obb {
            isometry_inv: isometry.inverse(),
            half_extent,
            corners: Obb::precompute_corners(&isometry, &half_extent),
            separating_axes: Obb::precompute_separating_axes(&isometry.rotation),
            isometry,
        }
    }

    fn precompute_corners(isometry: &Isometry3<S>, half_extent: &Vector3<S>) -> [Point3<S>; 8] {
        let corner_from =
            |x, y, z| isometry.rotation.rotate_point(Point3::new(x, y, z)) + isometry.translation;
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

    fn precompute_separating_axes(rotation: &Quaternion<S>) -> Vec<Vector3<S>> {
        let unit_x = Vector3::unit_x();
        let unit_y = Vector3::unit_y();
        let unit_z = Vector3::unit_z();
        let rot_x = rotation * unit_x;
        let rot_y = rotation * unit_y;
        let rot_z = rotation * unit_z;
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
        let Point3 { x, y, z } =
            self.isometry_inv.rotation.rotate_point(*p) + self.isometry_inv.translation;
        x.abs() <= self.half_extent.x
            && y.abs() <= self.half_extent.y
            && z.abs() <= self.half_extent.z
    }

    fn transform(&self, isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(Self::new(isometry * &self.isometry, self.half_extent))
    }
}

#[derive(Debug, Clone)]
pub struct OrientedBeam<S> {
    // The members here are an implementation detail and differ from the
    // minimal representation in the gRPC message to speed up operations.
    // Isometry_inv is the transform from world coordinates into "beam coordinates".
    isometry: Isometry3<S>,
    isometry_inv: Isometry3<S>,
    half_extent: Vector2<S>,
    corners: [Point3<S>; 4],
    separating_axes: Vec<Vector3<S>>,
}

impl<S: BaseFloat> OrientedBeam<S> {
    pub fn new(isometry: Isometry3<S>, half_extent: Vector2<S>) -> Self {
        OrientedBeam {
            isometry_inv: isometry.inverse(),
            half_extent,
            corners: OrientedBeam::precompute_corners(&isometry, &half_extent),
            separating_axes: OrientedBeam::precompute_separating_axes(&isometry.rotation),
            isometry,
        }
    }

    fn precompute_corners(isometry: &Isometry3<S>, half_extent: &Vector2<S>) -> [Point3<S>; 4] {
        let corner_from = |x, y| {
            isometry.rotation.rotate_point(Point3::new(x, y, S::zero())) + isometry.translation
        };
        [
            corner_from(half_extent.x, half_extent.y),
            corner_from(half_extent.x, -half_extent.y),
            corner_from(-half_extent.x, half_extent.y),
            corner_from(-half_extent.x, -half_extent.y),
        ]
    }

    // TODO(nnmm): Change the axes to describe a beam, which is finite in one direction.
    // Currently we have a beam which is infinite in both directions.
    // If we defined a beam on one side of the earth pointing towards the sky,
    // it will also collect points on the other side of the earth, which is undesired.
    fn precompute_separating_axes(rotation: &Quaternion<S>) -> Vec<Vector3<S>> {
        // The separating axis needs to be perpendicular to the beam's main
        // axis, i.e. the possible axes are the cross product of the three unit
        // vectors with the beam's main axis and the beam's face normals.
        let main_axis = rotation * Vector3::unit_z();
        vec![
            rotation * Vector3::unit_x(),
            rotation * Vector3::unit_y(),
            main_axis.cross(Vector3::unit_x()).normalize(),
            main_axis.cross(Vector3::unit_y()).normalize(),
            main_axis.cross(Vector3::unit_z()).normalize(),
        ]
        .into_iter()
        .filter(is_finite)
        .collect()
    }
}

impl<S> PointCulling<S> for OrientedBeam<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        intersects_aabb3(&self.corners, &self.separating_axes, aabb)
    }

    fn contains(&self, p: &Point3<S>) -> bool {
        // What is the point in beam coordinates?
        let Point3 { x, y, .. } =
            self.isometry_inv.rotation.rotate_point(*p) + self.isometry_inv.translation;
        x.abs() <= self.half_extent.x && y.abs() <= self.half_extent.y
    }

    fn transform(&self, isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        Box::new(Self::new(isometry * &self.isometry, self.half_extent))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::{Rad, Rotation3, Zero};
    // These tests were created in Blender by creating two cubes, naming one
    // "Beam" and scaling it up in its local z direction. The corresponding
    // object in Rust was defined by querying Blender's Python API with
    // bpy.data.objects['Beam'].rotation_euler.to_quaternion() and
    // bpy.data.objects['Beam'].location.

    fn some_beam() -> OrientedBeam<f64> {
        let quater = Quaternion::new(
            0.929_242_372_512_817_4,
            -0.267_741_590_738_296_5,
            -0.208_630_219_101_905_82,
            -0.145_932_972_431_182_86,
        );
        let translation = Vector3::new(1.0, 2.0, 0.0);
        let half_extent = Vector2::new(1.0, 1.0);
        OrientedBeam::new(Isometry3::new(quater, translation), half_extent)
    }

    #[test]
    fn test_beam_contains() {
        let point1 = Point3::new(-29.96, 57.85, 76.96); // (0, 0, 100) in local coordinates
        let point2 = Point3::new(-29.50, 58.83, 76.43); // (0, 1.2, 100) in local coordinates

        assert_eq!(some_beam().contains(&point1), true);
        assert_eq!(some_beam().contains(&point2), false);
    }

    #[test]
    fn test_beam_intersects_aabb3() {
        let bbox1 = Aabb3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let bbox2 = Aabb3::new(Point3::new(0.0, 0.0, 3.0), Point3::new(2.0, 2.0, 5.0));
        // bbox3 is completely inside the beam, none of its faces intersect it
        let bbox3 = Aabb3::new(Point3::new(0.25, 1.8, 0.0), Point3::new(1.25, 2.8, 1.0));
        // bbox4 intersects_aabb3 the beam, but has no vertices inside it
        let bbox4 = Aabb3::new(Point3::new(-3.0, -2.0, -3.5), Point3::new(5.0, 6.0, 4.5));
        let bbox5 = Aabb3::new(Point3::new(2.1, -0.55, 0.0), Point3::new(4.1, 1.45, 2.0));
        let bbox6 = Aabb3::new(Point3::new(0.9, -1.67, 0.0), Point3::new(2.9, 0.33, 2.0));
        let bbox7 = Aabb3::new(Point3::new(0.9, -1.57, 0.0), Point3::new(2.9, 0.43, 2.0));
        assert_eq!(some_beam().intersects_aabb3(&bbox1), true);
        assert_eq!(some_beam().intersects_aabb3(&bbox2), false);
        assert_eq!(some_beam().intersects_aabb3(&bbox3), true);
        assert_eq!(some_beam().intersects_aabb3(&bbox4), true);
        assert_eq!(some_beam().intersects_aabb3(&bbox5), false);
        assert_eq!(some_beam().intersects_aabb3(&bbox6), false);
        assert_eq!(some_beam().intersects_aabb3(&bbox7), true);
        let vertical_beam = OrientedBeam::new(
            Isometry3::new(Quaternion::zero(), Vector3::zero()),
            Vector2::new(1.0, 1.0),
        );
        assert_eq!(vertical_beam.intersects_aabb3(&bbox1), true);
    }

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
}

#[derive(Debug, Clone)]
pub struct Frustum<S: BaseFloat> {
    matrix: Matrix4<S>,
    frustum: collision::Frustum<S>,
}

impl<S: BaseFloat> Frustum<S> {
    pub fn new(matrix: Matrix4<S>) -> Self {
        Frustum {
            matrix,
            frustum: collision::Frustum::from_matrix4(matrix).unwrap(),
        }
    }
}

impl<S> PointCulling<S> for Frustum<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        // TODO(ksavinash9) update after https://github.com/rustgd/collision-rs/issues/101 is resolved.
        let v = Vector4::new(point.x, point.y, point.z, S::one());
        let clip_v = self.matrix * v;
        clip_v.x.abs() < clip_v.w
            && clip_v.y.abs() < clip_v.w
            && S::zero() < clip_v.z
            && clip_v.z < clip_v.w
    }

    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        match self.frustum.contains(aabb) {
            Relation::Cross => true,
            Relation::In => true,
            Relation::Out => false,
        }
    }

    fn transform(&self, isometry: &Isometry3<S>) -> Box<dyn PointCulling<S>> {
        let isometry = isometry.clone();
        let matrix: Matrix4<S> = Matrix4::from(
            Into::<Decomposed<Vector3<S>, Quaternion<S>>>::into(isometry),
        ) * self.matrix;
        Box::new(Frustum::new(matrix))
    }
}

// Returns transform needed to go from local frame to ECEF with the specified origin where
// the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
// used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
// https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
pub fn local_frame_from_lat_lng(lat: f64, lon: f64) -> Matrix4<f64> {
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
    Matrix4::from(frame)
}
