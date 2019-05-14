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
    BaseFloat, EuclideanSpace, InnerSpace, Point3, Quaternion, Rotation, Vector2, Vector3,
};
use collision::{Aabb, Aabb3};
use num_traits::Float;

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

// This guards against the separating axes being NaN, which may happen when the orientation aligns with the unit axes.
fn is_finite<S: BaseFloat>(vec: &Vector3<S>) -> bool {
    vec.x.is_finite() && vec.y.is_finite() && vec.z.is_finite()
}

fn intersects<S: BaseFloat>(
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

pub struct Obb<S> {
    rotation_inv: Quaternion<S>,
    translation_inv: Vector3<S>,
    half_extent: Vector3<S>,
    corners: [Point3<S>; 8],
    separating_axes: Vec<Vector3<S>>,
}

impl<S: BaseFloat> Obb<S> {
    pub fn new(rotation: Quaternion<S>, translation: Vector3<S>, half_extent: Vector3<S>) -> Self {
        Obb {
            rotation_inv: rotation.conjugate(),
            translation_inv: -rotation.conjugate().rotate_vector(translation),
            half_extent,
            corners: Obb::precompute_corners(&rotation, &translation, &half_extent),
            separating_axes: Obb::precompute_separating_axes(&rotation),
        }
    }

    pub fn intersects(&self, aabb: &Aabb3<S>) -> bool {
        intersects(&self.corners, &self.separating_axes, aabb)
    }

    pub fn contains(&self, p: &Point3<S>) -> bool {
        let Point3 { x, y, z } = self.rotation_inv.rotate_point(*p) + self.translation_inv;
        x.abs() <= self.half_extent.x
            && y.abs() <= self.half_extent.y
            && z.abs() <= self.half_extent.z
    }

    fn precompute_corners(
        rotation: &Quaternion<S>,
        translation: &Vector3<S>,
        half_extent: &Vector3<S>,
    ) -> [Point3<S>; 8] {
        let transform =
            |x: S, y: S, z: S| rotation.rotate_point(Point3::new(x, y, z)) + translation;
        [
            transform(-half_extent.x, -half_extent.y, -half_extent.z),
            transform(half_extent.x, -half_extent.y, -half_extent.z),
            transform(-half_extent.x, half_extent.y, -half_extent.z),
            transform(half_extent.x, half_extent.y, -half_extent.z),
            transform(-half_extent.x, -half_extent.y, half_extent.z),
            transform(half_extent.x, -half_extent.y, half_extent.z),
            transform(-half_extent.x, half_extent.y, half_extent.z),
            transform(half_extent.x, half_extent.y, half_extent.z),
        ]
    }

    fn precompute_separating_axes(rotation: &Quaternion<S>) -> Vec<Vector3<S>> {
        let unit_x = Vector3::unit_x();
        let unit_y = Vector3::unit_y();
        let unit_z = Vector3::unit_z();
        let rot_x = rotation.rotate_vector(unit_x);
        let rot_y = rotation.rotate_vector(unit_y);
        let rot_z = rotation.rotate_vector(unit_z);
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

#[derive(Debug, Clone)]
pub struct OrientedBeam {
    // The members here are an implementation detail and differ from the
    // minimal representation in the gRPC message to speed up operations.
    // Rotation_inv and translation_inv together form the transform from world
    // coordinates into "beam coordinates".
    rotation_inv: Quaternion<f64>,
    translation_inv: Vector3<f64>,
    half_extent: Vector2<f64>,
    corners: [Point3<f64>; 4],
    separating_axes: Vec<Vector3<f64>>,
}

impl OrientedBeam {
    pub fn new(
        rotation: Quaternion<f64>,
        translation: Vector3<f64>,
        half_extent: Vector2<f64>,
    ) -> Self {
        OrientedBeam {
            rotation_inv: rotation.conjugate(),
            translation_inv: -rotation.conjugate().rotate_vector(translation),
            half_extent,
            corners: OrientedBeam::precompute_corners(&rotation, &translation, &half_extent),
            separating_axes: OrientedBeam::precompute_separating_axes(&rotation),
        }
    }

    pub fn intersects(&self, aabb: &Aabb3<f64>) -> bool {
        intersects(&self.corners, &self.separating_axes, aabb)
    }

    pub fn contains(&self, p: &Point3<f64>) -> bool {
        // What is the point in beam coordinates?
        let Point3 { x, y, .. } = self.rotation_inv.rotate_point(*p) + self.translation_inv;
        x.abs() <= self.half_extent.x && y.abs() <= self.half_extent.y
    }

    fn precompute_corners(
        rotation: &Quaternion<f64>,
        translation: &Vector3<f64>,
        half_extent: &Vector2<f64>,
    ) -> [Point3<f64>; 4] {
        let transform =
            |x: f64, y: f64| rotation.rotate_point(Point3::new(x, y, 0.0)) + translation;
        [
            transform(half_extent.x, half_extent.y),
            transform(half_extent.x, -half_extent.y),
            transform(-half_extent.x, half_extent.y),
            transform(-half_extent.x, -half_extent.y),
        ]
    }

    // TODO(nnmm): Change the axes to describe a beam, which is finitie in one direction.
    // Currently we have a beam which is infinite in both directions.
    // If we defined a beam on one side of the earth pointing towards the sky,
    // it will also collect points on the other side of the earth, which is undesired.
    fn precompute_separating_axes(rotation: &Quaternion<f64>) -> Vec<Vector3<f64>> {
        // The separating axis needs to be perpendicular to the beam's main
        // axis, i.e. the possible axes are the cross product of the three unit
        // vectors with the beam's main axis and the beam's face normals.
        let main_axis = rotation.rotate_vector(Vector3::unit_z());
        vec![
            rotation.rotate_vector(Vector3::unit_x()),
            rotation.rotate_vector(Vector3::unit_y()),
            main_axis.cross(Vector3::unit_x()).normalize(),
            main_axis.cross(Vector3::unit_y()).normalize(),
            main_axis.cross(Vector3::unit_z()).normalize(),
        ]
        .into_iter()
        .filter(is_finite)
        .collect()
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

    fn some_beam() -> OrientedBeam {
        let quater = Quaternion::new(
            0.9292423725128174,
            -0.2677415907382965,
            -0.20863021910190582,
            -0.14593297243118286,
        );
        let translation = Vector3::new(1.0, 2.0, 0.0);
        let half_extent = Vector2::new(1.0, 1.0);
        OrientedBeam::new(quater, translation, half_extent)
    }

    #[test]
    fn test_beam_contains() {
        let point1 = Point3::new(-29.96, 57.85, 76.96); // (0, 0, 100) in local coordinates
        let point2 = Point3::new(-29.50, 58.83, 76.43); // (0, 1.2, 100) in local coordinates

        assert_eq!(some_beam().contains(&point1), true);
        assert_eq!(some_beam().contains(&point2), false);
    }

    #[test]
    fn test_beam_intersects() {
        let bbox1 = Aabb3::new(Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 2.0, 2.0));
        let bbox2 = Aabb3::new(Point3::new(0.0, 0.0, 3.0), Point3::new(2.0, 2.0, 5.0));
        // bbox3 is completely inside the beam, none of its faces intersect it
        let bbox3 = Aabb3::new(Point3::new(0.25, 1.8, 0.0), Point3::new(1.25, 2.8, 1.0));
        // bbox4 intersects the beam, but has no vertices inside it
        let bbox4 = Aabb3::new(Point3::new(-3.0, -2.0, -3.5), Point3::new(5.0, 6.0, 4.5));
        let bbox5 = Aabb3::new(Point3::new(2.1, -0.55, 0.0), Point3::new(4.1, 1.45, 2.0));
        let bbox6 = Aabb3::new(Point3::new(0.9, -1.67, 0.0), Point3::new(2.9, 0.33, 2.0));
        let bbox7 = Aabb3::new(Point3::new(0.9, -1.57, 0.0), Point3::new(2.9, 0.43, 2.0));
        assert_eq!(some_beam().intersects(&bbox1), true);
        assert_eq!(some_beam().intersects(&bbox2), false);
        assert_eq!(some_beam().intersects(&bbox3), true);
        assert_eq!(some_beam().intersects(&bbox4), true);
        assert_eq!(some_beam().intersects(&bbox5), false);
        assert_eq!(some_beam().intersects(&bbox6), false);
        assert_eq!(some_beam().intersects(&bbox7), true);
        let vertical_beam =
            OrientedBeam::new(Quaternion::zero(), Vector3::zero(), Vector2::new(1.0, 1.0));
        assert_eq!(vertical_beam.intersects(&bbox1), true);
    }

    #[test]
    fn test_obb_intersects() {
        let zero_rot: Quaternion<f64> = Rotation3::from_angle_z(Rad(0.0));
        let fourty_five_deg_rot: Quaternion<f64> =
            Rotation3::from_angle_z(Rad(std::f64::consts::PI / 4.0));
        let arbitrary_rot: Quaternion<f64> =
            Rotation3::from_axis_angle(Vector3::new(0.2, 0.5, -0.7), Rad(0.123));
        let translation = Vector3::new(0.0, 0.0, 0.0);
        let half_extent = Vector3::new(1.0, 2.0, 3.0);
        let zero_obb = Obb::new(zero_rot, translation, half_extent);
        let fourty_five_deg_obb = Obb::new(fourty_five_deg_rot, translation, half_extent);
        let arbitrary_obb = Obb::new(arbitrary_rot, translation, half_extent);
        let bbox = Aabb3::new(Point3::new(0.5, 1.0, -3.0), Point3::new(1.5, 3.0, 3.0));
        assert_eq!(zero_obb.separating_axes.len(), 3);
        assert_eq!(zero_obb.intersects(&bbox), true);
        assert_eq!(fourty_five_deg_obb.separating_axes.len(), 5);
        assert_eq!(fourty_five_deg_obb.intersects(&bbox), false);
        assert_eq!(arbitrary_obb.separating_axes.len(), 15);
    }
}
