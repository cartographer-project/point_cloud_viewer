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

pub mod batch_iterator;
pub mod color;
pub mod data_provider;
pub mod errors;
pub mod math;
pub mod octree;
pub mod read_write;
pub mod s2_geo;

use cgmath::Vector3;
use std::collections::BTreeMap;

#[derive(Debug, Clone)]
pub struct Point {
    pub position: cgmath::Vector3<f64>,
    // TODO(sirver): Make color optional, we might not always have it.
    pub color: color::Color<u8>,

    // The intensity of the point if it exists. This value is usually handed through directly by a
    // sensor and has therefore no defined range - or even meaning.
    pub intensity: Option<f32>,
}

pub fn attribute_extension(attribute: &str) -> &str {
    match attribute {
        "position" => "xyz",
        "color" => "rgb",
        _ => attribute,
    }
}

/// General field to describe point feature attributes such as color, intensity, ...
#[derive(Debug, Clone)]
pub enum AttributeData {
    U8(Vec<u8>),
    I64(Vec<i64>),
    U64(Vec<u64>),
    F32(Vec<f32>),
    F64(Vec<f64>),
    U8Vec3(Vec<Vector3<u8>>),
    F64Vec3(Vec<Vector3<f64>>),
}

impl AttributeData {
    pub fn len(&self) -> usize {
        match self {
            AttributeData::U8(data) => data.len(),
            AttributeData::I64(data) => data.len(),
            AttributeData::U64(data) => data.len(),
            AttributeData::F32(data) => data.len(),
            AttributeData::F64(data) => data.len(),
            AttributeData::U8Vec3(data) => data.len(),
            AttributeData::F64Vec3(data) => data.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn dim(&self) -> usize {
        match self {
            AttributeData::U8(_)
            | AttributeData::I64(_)
            | AttributeData::U64(_)
            | AttributeData::F32(_)
            | AttributeData::F64(_) => 1,
            AttributeData::U8Vec3(_) | AttributeData::F64Vec3(_) => 3,
        }
    }
}

/// General structure that contains points and attached feature attributes.
pub struct PointsBatch {
    pub position: Vec<Vector3<f64>>,
    // BTreeMap for deterministic iteration order.
    pub attributes: BTreeMap<String, AttributeData>,
}

pub use point_viewer_proto_rust::proto;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;
