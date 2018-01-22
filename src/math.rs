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

pub type Vector2f = cgmath::Vector2<f32>;
pub type Vector3f = cgmath::Vector3<f32>;
pub type Matrix4f = cgmath::Matrix4<f32>;
pub type Point3f = cgmath::Point3<f32>;
pub type Aabb3f = collision::Aabb3<f32>;
pub use cgmath::prelude::*;
pub use collision::Aabb;

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

    pub fn to_aabb3(&self) -> Aabb3f {
        Aabb3f::new(self.min(), self.max())
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
