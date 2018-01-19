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

use cgmath;
use collision;

// TODO(hrapp): collision-rs has nearly everything we need. The Frustum is missing a 'intersects'
// method and it needs updating to work with newer cgmaths.

pub type Vector2f = cgmath::Vector2<f32>;
pub type Vector3f = cgmath::Vector3<f32>;
pub type Matrix4f = cgmath::Matrix4<f32>;
pub type Point3f = cgmath::Point3<f32>;
pub type Aabb3f = collision::Aabb3<f32>;
pub use cgmath::prelude::*;
pub use collision::Aabb;

#[derive(Debug, Clone)]
struct Plane {
    normal: Vector3f,
    w: f32,
}

impl Plane {
    pub fn new(n: Vector3f, w: f32) -> Self {
        let norm = n.magnitude();
        Plane {
            normal: n / norm,
            w: w / norm,
        }
    }

    pub fn get_distance(&self, v: &Vector3f) -> f32 {
        self.normal.dot(*v) + self.w
    }
}

#[derive(Debug)]
pub struct Frustum {
    planes: [Plane; 6],
}

impl Frustum {
    pub fn from_matrix(m: &Matrix4f) -> Self {
        Frustum {
            planes: [
                Plane::new(
                    Vector3f::new(m[0][3] - m[0][0], m[1][3] - m[1][0], m[2][3] - m[2][0]),
                    m[3][3] - m[3][0],
                ),
                Plane::new(
                    Vector3f::new(m[0][3] + m[0][0], m[1][3] + m[1][0], m[2][3] + m[2][0]),
                    m[3][3] + m[3][0],
                ),
                Plane::new(
                    Vector3f::new(m[0][3] + m[0][1], m[1][3] + m[1][1], m[2][3] + m[2][1]),
                    m[3][3] + m[3][1],
                ),
                Plane::new(
                    Vector3f::new(m[0][3] - m[0][1], m[1][3] - m[1][1], m[2][3] - m[2][1]),
                    m[3][3] - m[3][1],
                ),
                Plane::new(
                    Vector3f::new(m[0][3] - m[0][2], m[1][3] - m[1][2], m[2][3] - m[2][2]),
                    m[3][3] - m[3][2],
                ),
                Plane::new(
                    Vector3f::new(m[0][3] + m[0][2], m[1][3] + m[1][2], m[2][3] + m[2][2]),
                    m[3][3] + m[3][2],
                ),
            ],
        }
    }

    pub fn intersects(&self, bb: &Cube) -> bool {
        for plane in &self.planes {
            let p1 = Vector3f::new(
                if plane.normal.x > 0f32 {
                    bb.min().x
                } else {
                    bb.max().x
                },
                if plane.normal.y > 0f32 {
                    bb.min().y
                } else {
                    bb.max().y
                },
                if plane.normal.z > 0f32 {
                    bb.min().z
                } else {
                    bb.max().z
                },
            );
            let p2 = Vector3f::new(
                if plane.normal.x > 0f32 {
                    bb.max().x
                } else {
                    bb.min().x
                },
                if plane.normal.y > 0f32 {
                    bb.max().y
                } else {
                    bb.min().y
                },
                if plane.normal.z > 0f32 {
                    bb.max().z
                } else {
                    bb.min().z
                },
            );
            let d1 = plane.get_distance(&p1);
            let d2 = plane.get_distance(&p2);
            if d1 < 0f32 && d2 < 0f32 {
                return false;
            }
        }
        true
    }
}

#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3f,
    edge_length: f32,
}

impl Cube {
    pub fn bounding(aabb: &Aabb3f) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: aabb.min,
            edge_length: edge_length,
        }
    }

    pub fn new(min: Point3f, edge_length: f32) -> Self {
        Cube {
            min: min,
            edge_length: edge_length,
        }
    }

    pub fn edge_length(&self) -> f32 {
        self.edge_length
    }

    pub fn min(&self) -> Point3f {
        self.min
    }

    pub fn max(&self) -> Point3f {
        Point3f::new(
            self.min.x + self.edge_length,
            self.min.y + self.edge_length,
            self.min.z + self.edge_length,
        )
    }

    /// The center of the box.
    pub fn center(&self) -> Vector3f {
        let min = self.min();
        let max = self.max();
        Vector3f::new(
            (min.x + max.x) / 2.,
            (min.y + max.y) / 2.,
            (min.z + max.z) / 2.,
        )
    }
}

pub fn clamp(value: f32, low: f32, high: f32) -> f32 {
    if value < high {
        value.max(low)
    } else {
        value.min(high)
    }
}
