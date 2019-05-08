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

// TODO(sirver): Work around suppressing a warning until
// https://github.com/rust-lang-nursery/error-chain/pull/246 is released.
#![allow(renamed_and_removed_lints)]
#![recursion_limit = "1024"]

pub mod codec;
pub mod color;
// TODO(feuerste): Remove this, once https://github.com/rust-lang-nursery/error-chain/pull/255 is merged.
#[allow(deprecated)]
pub mod errors;
pub mod generation;
pub mod math;
pub mod octree;
pub mod ply;
pub mod pts;

use cgmath::{Vector3, Vector4};
use fnv::FnvHashMap;

#[derive(Debug, Clone)]
pub struct Point {
    pub position: cgmath::Vector3<f64>,
    // TODO(sirver): Make color optional, we might not always have it.
    pub color: color::Color<u8>,

    // The intensity of the point if it exists. This value is usually handed through directly by a
    // sensor and has therefore no defined range - or even meaning.
    pub intensity: Option<f32>,
}
pub enum LayerData {
    F32(Vec<f32>),
    F64Vec3(Vec<Vector3<f64>>),
    U8Vec4(Vec<Vector4<u8>>),
}

impl LayerData {
    pub fn len(&self) -> usize {
        match self {
            LayerData::F32(data) => data.len(),
            LayerData::F64Vec3(data) => data.len(),
            LayerData::U8Vec4(data) => data.len(),
        }
    }
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
    pub fn dim(&self) -> usize {
        match self {
            LayerData::F32(_) => 1,
            LayerData::F64Vec3(_) => 3,
            LayerData::U8Vec4(_) => 4,
        }
    }
}

pub struct PointData {
    pub position: Vec<Vector3<f64>>,
    pub layers: FnvHashMap<String, LayerData>,
}

struct PointStream<'a, F> {
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    func: &'a mut F,
}

impl<'a, F> PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    fn new(num_points_per_batch: usize, func: &'a mut F) -> Self {
        PointStream {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            func,
        }
    }

    fn push_point(&mut self, point: Point) {
        self.position.push(point.position);
        self.color.push(Vector4::new(
            point.color.red,
            point.color.green,
            point.color.blue,
            point.color.alpha,
        ));
        if let Some(point_intensity) = point.intensity {
            self.intensity.push(point_intensity);
        };
    }

    fn callback(&mut self) -> Result<()> {
        if self.position.is_empty() {
            return Ok(());
        }

        let mut layers = FnvHashMap::default();
        layers.insert(
            "color".to_string(),
            LayerData::U8Vec4(self.color.split_off(0)),
        );
        if !self.intensity.is_empty() {
            layers.insert(
                "intensity".to_string(),
                LayerData::F32(self.intensity.split_off(0)),
            );
        }
        let point_data = PointData {
            position: self.position.split_off(0),
            layers,
        };
        (self.func)(point_data)
    }

    fn push_point_and_callback(&mut self, point: Point) -> Result<()> {
        self.push_point(point);
        if self.position.len() == self.position.capacity() {
            return self.callback();
        }
        Ok(())
    }
}

pub trait BatchIterator: Iterator{
    type BatchContainer;

    fn try_for_each_batch<F: FnMut(Self::BatchContainer)>(&mut self, f: F) -> Result<()>;
    
}

pub use point_viewer_proto_rust::proto;
