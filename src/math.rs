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
use std;

// TODO(hrapp): collision-rs has nearly everything we need. The Frustum is missing a 'intersects'
// method and it needs updating to work with newer cgmaths.

pub type Vector2f = cgmath::Vector2<f32>;
pub type Vector3f = cgmath::Vector3<f32>;
pub type Matrix4f = cgmath::Matrix4<f32>;
pub use cgmath::prelude::*;

pub trait CuboidLike {
    fn min(&self) -> Vector3f;
    fn max(&self) -> Vector3f;

    /// Returns true if 'p' is contained in the box.
    fn contains(&self, p: &Vector3f) -> bool {
        let min = self.min();
        let max = self.max();
        min.x <= p.x && p.x <= max.x && min.y <= p.y && p.y <= max.y && min.z <= p.z && p.z <= max.z
    }

    /// The center of the box.
    fn center(&self) -> Vector3f {
        let min = self.min();
        let max = self.max();
        Vector3f::new(
            (min.x + max.x) / 2.,
            (min.y + max.y) / 2.,
            (min.z + max.z) / 2.,
        )
    }

    /// The size of the box.
    fn size(&self) -> Vector3f {
        self.max() - self.min()
    }
}

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

    pub fn intersects<C: CuboidLike>(&self, bb: &C) -> bool {
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
pub struct Cuboid {
    min: Vector3f,
    max: Vector3f,
}

impl CuboidLike for Cuboid {
    fn min(&self) -> Vector3f {
        self.min
    }

    fn max(&self) -> Vector3f {
        self.max
    }
}

#[derive(Debug, Clone)]
pub struct Cube {
    min: Vector3f,
    edge_length: f32,
}

impl CuboidLike for Cube {
    fn min(&self) -> Vector3f {
        self.min
    }

    fn max(&self) -> Vector3f {
        Vector3f::new(
            self.min.x + self.edge_length,
            self.min.y + self.edge_length,
            self.min.z + self.edge_length,
        )
    }
}

impl Cube {
    pub fn new(min: Vector3f, edge_length: f32) -> Self {
        Cube {
            min: min,
            edge_length: edge_length,
        }
    }

    pub fn edge_length(&self) -> f32 {
        self.edge_length
    }
}

/// An axis-aligned bounding box.
impl Cuboid {
    pub fn with_dimensions(min: Vector3f, max: Vector3f) -> Self {
        Cuboid { min, max }
    }
    pub fn new() -> Self {
        Cuboid {
            min: Vector3f::new(std::f32::MAX, std::f32::MAX, std::f32::MAX),
            max: Vector3f::new(std::f32::MIN, std::f32::MIN, std::f32::MIN),
        }
    }

    /// Grows the box to contain 'p'.
    pub fn update(&mut self, p: &Vector3f) {
        self.min.x = self.min.x.min(p.x);
        self.min.y = self.min.y.min(p.y);
        self.min.z = self.min.z.min(p.z);
        self.max.x = self.max.x.max(p.x);
        self.max.y = self.max.y.max(p.y);
        self.max.z = self.max.z.max(p.z);
    }

    /// Changes the edge_length of the box to be cubic, i.e. all dimensions have the same length.
    /// The new 'Cube' will fully contain the old 'Cuboid'.
    pub fn into_cube(self) -> Cube {
        let edge_length = (self.max.x - self.min.x)
            .max(self.max.y - self.min.y)
            .max(self.max.z - self.min.z);
        Cube {
            min: self.min,
            edge_length: edge_length,
        }
    }
}

pub fn clamp(value: f32, low: f32, high: f32) -> f32 {
    if value < high {
        value.max(low)
    } else {
        value.min(high)
    }
}
