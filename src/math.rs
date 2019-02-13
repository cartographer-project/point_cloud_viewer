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

use cgmath::{Point3, Vector3};
use collision::{Aabb, Aabb3};

#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3<f32>,
    edge_length: f32,
}

impl Cube {
    pub fn bounding(aabb: &Aabb3<f32>) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: aabb.min,
            edge_length,
        }
    }

    pub fn to_aabb3(&self) -> Aabb3<f32> {
        Aabb3::new(self.min(), self.max())
    }

    pub fn new(min: Point3<f32>, edge_length: f32) -> Self {
        Cube { min, edge_length }
    }

    pub fn edge_length(&self) -> f32 {
        self.edge_length
    }

    pub fn min(&self) -> Point3<f32> {
        self.min
    }

    pub fn max(&self) -> Point3<f32> {
        Point3::new(
            self.min.x + self.edge_length,
            self.min.y + self.edge_length,
            self.min.z + self.edge_length,
        )
    }

    /// The center of the box.
    pub fn center(&self) -> Vector3<f32> {
        let min = self.min();
        let max = self.max();
        Vector3::new(
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
