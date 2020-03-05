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
#[macro_use]
pub mod attributes;
pub mod color;
pub mod data_provider;
// Workaround for https://github.com/rust-lang-nursery/error-chain/issues/254
#[allow(deprecated)]
pub mod errors;
pub mod geometry;
pub mod iterator;
pub mod math;
pub mod octree;
pub mod read_write;
pub mod s2_cells;
pub mod utils;

use cgmath::Vector3;
use errors::Result;
use std::collections::{BTreeMap, HashMap};
use std::convert::{TryFrom, TryInto};

// Version 9 -> 10: Change in NodeId proto from level (u8) and index (u64) to high (u64) and low
// (u64). We are able to convert the proto on read, so the tools can still read version 9.
// Version 10 -> 11: Change in AxisAlignedCuboid proto from Vector3f min/max to Vector3d min/max.
// We are able to convert the proto on read, so the tools can still read version 9/10.
// Version 11 -> 12: Change in Meta names and structures, to allow both s2 and octree meta.
// We are able to convert the proto on read, so the tools can still read version 9/10/11.
// Version 12 -> 13: Change back bounding box from OctreeMeta to Meta.
// We are able to convert the proto on read, so the tools can still read version 9/10/11/12.
pub const CURRENT_VERSION: i32 = 13;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

pub trait NumberOfPoints {
    fn num_points(&self) -> usize;
}

use attributes::{AttributeData, AttributeDataType};

#[derive(Debug, Clone)]
pub struct Point {
    pub position: Vector3<f64>,
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

trait PointCloudMeta {
    fn attribute_data_types(&self) -> &HashMap<String, AttributeDataType>;
    fn attribute_data_types_for(
        &self,
        attributes: &[&str],
    ) -> Result<HashMap<String, AttributeDataType>> {
        attributes
            .iter()
            .map(|a| {
                self.attribute_data_types()
                    .get(*a)
                    .map(|d| ((*a).to_string(), *d))
                    .ok_or_else(|| format!("Data type for attribute '{}' not found.", a).into())
            })
            .collect()
    }
}

/// General structure that contains points and attached feature attributes.
#[derive(Debug, Clone)]
pub struct PointsBatch {
    pub position: Vec<Vector3<f64>>,
    // BTreeMap for deterministic iteration order.
    pub attributes: BTreeMap<String, AttributeData>,
}

impl PointsBatch {
    pub fn append(&mut self, other: &mut PointsBatch) -> std::result::Result<(), String> {
        if self.position.is_empty() {
            *self = other.split_off(0);
        } else {
            assert_eq!(self.attributes.len(), other.attributes.len());
            self.position.append(&mut other.position);
            for (s, o) in self
                .attributes
                .values_mut()
                .zip(other.attributes.values_mut())
            {
                s.append(o)?;
            }
        }
        Ok(())
    }

    pub fn split_off(&mut self, at: usize) -> Self {
        let position = self.position.split_off(at);
        let attributes = self
            .attributes
            .iter_mut()
            .map(|(n, a)| (n.clone(), a.split_off(at)))
            .collect();
        Self {
            position,
            attributes,
        }
    }

    pub fn retain(&mut self, keep: &[bool]) {
        assert_eq!(self.position.len(), keep.len());
        let mut keep = keep.iter().copied().cycle();
        self.position.retain(|_| keep.next().unwrap());
        for a in self.attributes.values_mut() {
            macro_rules! rhs {
                ($dtype:ident, $data:ident, $keep:expr) => {
                    $data.retain(|_| $keep.next().unwrap())
                };
            }
            match_attr_data!(a, rhs, keep)
        }
    }

    pub fn get_attribute_vec<'a, T>(
        &'a self,
        key: impl AsRef<str>,
    ) -> std::result::Result<&'a Vec<T>, String>
    where
        &'a Vec<T>: TryFrom<&'a AttributeData, Error = String>,
    {
        self.attributes
            .get(key.as_ref())
            .ok_or_else(|| format!("Attribute '{}' not found.", key.as_ref()))
            .and_then(|val| val.try_into())
    }

    pub fn get_attribute_vec_mut<'a, T>(
        &'a mut self,
        key: impl AsRef<str>,
    ) -> std::result::Result<&'a mut Vec<T>, String>
    where
        &'a mut Vec<T>: TryFrom<&'a mut AttributeData, Error = String>,
    {
        self.attributes
            .get_mut(key.as_ref())
            .ok_or_else(|| format!("Attribute '{}' not found.", key.as_ref()))
            .and_then(|val| val.try_into())
    }

    pub fn remove_attribute_vec<T>(
        &mut self,
        key: impl AsRef<str>,
    ) -> std::result::Result<Vec<T>, String>
    where
        Vec<T>: TryFrom<AttributeData, Error = String>,
    {
        self.attributes
            .remove(key.as_ref())
            .ok_or_else(|| format!("Attribute '{}' not found.", key.as_ref()))
            .and_then(|val| val.try_into())
    }
}

pub use point_viewer_proto_rust::proto;
