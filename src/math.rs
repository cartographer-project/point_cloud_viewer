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

use cgmath::{EuclideanSpace, InnerSpace, Point3, Quaternion, Rotation, Vector2, Vector3};
use collision::{Aabb, Aabb3};

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
    separating_axes: [Vector3<f64>; 5],
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
        // SAT algorithm
        // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
        let mut separated = false;
        for sep_axis in self.separating_axes.iter() {
            // Project the cube and the beam onto that axis
            let mut cube_min_proj = std::f64::MAX;
            let mut cube_max_proj = std::f64::MIN;
            for corner in aabb.to_corners().iter() {
                let corner_proj = corner.dot(*sep_axis);
                cube_min_proj = cube_min_proj.min(corner_proj);
                cube_max_proj = cube_max_proj.max(corner_proj);
            }
            // Project four "vertices" of the beam onto that axis
            let mut beam_min_proj = std::f64::MAX;
            let mut beam_max_proj = std::f64::MIN;
            for corner in self.corners.iter() {
                let corner_proj = corner.dot(*sep_axis);
                beam_min_proj = beam_min_proj.min(corner_proj);
                beam_max_proj = beam_max_proj.max(corner_proj);
            }
            separated |= beam_min_proj > cube_max_proj || beam_max_proj < cube_min_proj;
        }
        !separated
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
        [
            rotation.rotate_point(Point3::new(half_extent.x, half_extent.y, 0.0)) + translation,
            rotation.rotate_point(Point3::new(half_extent.x, -half_extent.y, 0.0)) + translation,
            rotation.rotate_point(Point3::new(-half_extent.x, half_extent.y, 0.0)) + translation,
            rotation.rotate_point(Point3::new(-half_extent.x, -half_extent.y, 0.0)) + translation,
        ]
    }

    fn precompute_separating_axes(rotation: &Quaternion<f64>) -> [Vector3<f64>; 5] {
        // The separating axis needs to be perpendicular to the beam's main
        // axis, i.e. the possible axes are the cross product of the three unit
        // vectors with the beam's main axis and the beam's face normals
        let main_axis = rotation.rotate_vector(Vector3::unit_z());
        [
            rotation.rotate_vector(Vector3::unit_x()),
            rotation.rotate_vector(Vector3::unit_y()),
            main_axis.cross(Vector3::unit_x()).normalize(),
            main_axis.cross(Vector3::unit_y()).normalize(),
            main_axis.cross(Vector3::unit_z()).normalize(),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
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
    }
}
