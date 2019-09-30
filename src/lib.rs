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

pub mod color;
pub mod data_provider;
pub mod errors;
pub mod iterator;
pub mod math;
pub mod octree;
pub mod read_write;
pub mod s2_cells;

use cgmath::Vector3;
use errors::{ErrorKind, Result};
use std::collections::BTreeMap;
use std::convert::{TryFrom, TryInto};

// Version 9 -> 10: Change in NodeId proto from level (u8) and index (u64) to high (u64) and low
// (u64). We are able to convert the proto on read, so the tools can still read version 9.
// Version 10 -> 11: Change in AxisAlignedCuboid proto from Vector3f min/max to Vector3d min/max.
// We are able to convert the proto on read, so the tools can still read version 9/10.
// Version 11 -> 12: Change in Meta names and structures, to allow both s2 and octree meta.
// We are able to convert the proto on read, so the tools can still read version 9/10/11.
pub const CURRENT_VERSION: i32 = 12;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

pub trait NumberOfPoints {
    fn num_points(&self) -> Option<usize>;
}

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

#[derive(Copy, Debug, Clone, PartialEq, Eq)]
pub enum AttributeDataType {
    U8,
    I64,
    U64,
    F32,
    F64,
    U8Vec3,
    F64Vec3,
}

impl AttributeDataType {
    pub fn to_proto(self) -> proto::AttributeDataType {
        match self {
            AttributeDataType::U8 => proto::AttributeDataType::U8,
            AttributeDataType::I64 => proto::AttributeDataType::I64,
            AttributeDataType::U64 => proto::AttributeDataType::U64,
            AttributeDataType::F32 => proto::AttributeDataType::F32,
            AttributeDataType::F64 => proto::AttributeDataType::F64,
            AttributeDataType::U8Vec3 => proto::AttributeDataType::U8Vec3,
            AttributeDataType::F64Vec3 => proto::AttributeDataType::F64Vec3,
        }
    }

    pub fn from_proto(attr_proto: proto::AttributeDataType) -> Result<Self> {
        let attr = match attr_proto {
            proto::AttributeDataType::U8 => AttributeDataType::U8,
            proto::AttributeDataType::I64 => AttributeDataType::I64,
            proto::AttributeDataType::U64 => AttributeDataType::U64,
            proto::AttributeDataType::F32 => AttributeDataType::F32,
            proto::AttributeDataType::F64 => AttributeDataType::F64,
            proto::AttributeDataType::U8Vec3 => AttributeDataType::U8Vec3,
            proto::AttributeDataType::F64Vec3 => AttributeDataType::F64Vec3,
            proto::AttributeDataType::U16
            | proto::AttributeDataType::U32
            | proto::AttributeDataType::I8
            | proto::AttributeDataType::I16
            | proto::AttributeDataType::I32 => {
                return Err(ErrorKind::InvalidInput(
                    "Attribute data type not supported yet".to_string(),
                )
                .into())
            }
            proto::AttributeDataType::INVALID_DATA_TYPE => {
                return Err(
                    ErrorKind::InvalidInput("Attribute data type invalid".to_string()).into(),
                )
            }
        };
        Ok(attr)
    }

    pub fn size_of(self) -> usize {
        match self {
            AttributeDataType::U8 => 1,
            AttributeDataType::F32 => 4,
            AttributeDataType::I64 | AttributeDataType::U64 | AttributeDataType::F64 => 8,
            AttributeDataType::U8Vec3 => 3,
            AttributeDataType::F64Vec3 => 3 * 8,
        }
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

    pub fn data_type(&self) -> AttributeDataType {
        match self {
            AttributeData::U8(_) => AttributeDataType::U8,
            AttributeData::I64(_) => AttributeDataType::I64,
            AttributeData::U64(_) => AttributeDataType::U64,
            AttributeData::F32(_) => AttributeDataType::F32,
            AttributeData::F64(_) => AttributeDataType::F64,
            AttributeData::U8Vec3(_) => AttributeDataType::U8Vec3,
            AttributeData::F64Vec3(_) => AttributeDataType::F64Vec3,
        }
    }
}

macro_rules! try_from_impl {
    ($data:ident, $attribute_data_type:ident, $vec_data_type:ty) => {
        match $data {
            AttributeData::$attribute_data_type(data) => Ok(data),
            _ => Err(format!(
                "Attribute data type '{:?}' is incompatible with requested type '{}'.",
                $data.data_type(),
                stringify!($vec_data_type)
            )),
        }
    };
}

macro_rules! try_from_attribute_data {
    ($attribute_data_type:ident, $vec_data_type:ty) => {
        impl TryFrom<AttributeData> for Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }

        impl<'a> TryFrom<&'a AttributeData> for &'a Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: &'a AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }

        impl<'a> TryFrom<&'a mut AttributeData> for &'a mut Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: &'a mut AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }
    };
}
try_from_attribute_data!(U8, u8);
try_from_attribute_data!(I64, i64);
try_from_attribute_data!(U64, u64);
try_from_attribute_data!(F32, f32);
try_from_attribute_data!(F64, f64);
try_from_attribute_data!(U8Vec3, Vector3<u8>);
try_from_attribute_data!(F64Vec3, Vector3<f64>);

fn append_attribute<'a, T: 'a>(
    name: &str,
    to: &'a mut PointsBatch,
    from: &'a mut PointsBatch,
) -> std::result::Result<(), String>
where
    &'a mut Vec<T>: TryFrom<&'a mut AttributeData, Error = String>,
{
    let to_vec: &'a mut Vec<T> = to.get_attribute_vec_mut(name)?;
    let from_vec: &'a mut Vec<T> = from.get_attribute_vec_mut(name)?;
    to_vec.append(from_vec);
    Ok(())
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
            let data_types: Vec<(String, AttributeDataType)> = self
                .attributes
                .iter()
                .map(|(name, attrib)| (name.clone(), attrib.data_type()))
                .collect();
            for (name, dtype) in &data_types {
                use AttributeDataType::*;
                match dtype {
                    U8 => append_attribute::<u8>(name, self, other)?,
                    U64 => append_attribute::<u64>(name, self, other)?,
                    I64 => append_attribute::<i64>(name, self, other)?,
                    F32 => append_attribute::<f32>(name, self, other)?,
                    F64 => append_attribute::<f64>(name, self, other)?,
                    U8Vec3 => append_attribute::<Vector3<u8>>(name, self, other)?,
                    F64Vec3 => append_attribute::<Vector3<f64>>(name, self, other)?,
                };
            }
        }
        Ok(())
    }

    pub fn split_off(&mut self, at: usize) -> Self {
        let mut res = Self {
            position: self.position.split_off(at),
            attributes: BTreeMap::new(),
        };
        for (name, attribute) in self.attributes.iter_mut() {
            let name = name.clone();
            use AttributeData::*;
            match attribute {
                U8(data) => res.attributes.insert(name, U8(data.split_off(at))),
                U64(data) => res.attributes.insert(name, U64(data.split_off(at))),
                I64(data) => res.attributes.insert(name, I64(data.split_off(at))),
                F32(data) => res.attributes.insert(name, F32(data.split_off(at))),
                F64(data) => res.attributes.insert(name, F64(data.split_off(at))),
                U8Vec3(data) => res.attributes.insert(name, U8Vec3(data.split_off(at))),
                F64Vec3(data) => res.attributes.insert(name, F64Vec3(data.split_off(at))),
            };
        }
        res
    }

    pub fn retain(&mut self, keep: &[bool]) {
        assert_eq!(self.position.len(), keep.len());
        let mut keep = keep.iter().copied().cycle();
        self.position.retain(|_| keep.next().unwrap());
        for a in self.attributes.values_mut() {
            match a {
                AttributeData::U8(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::U64(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::I64(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::F32(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::F64(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::U8Vec3(data) => data.retain(|_| keep.next().unwrap()),
                AttributeData::F64Vec3(data) => data.retain(|_| keep.next().unwrap()),
            }
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
