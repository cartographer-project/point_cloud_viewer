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

use crate::errors::*;
use crate::math::{self, Cube};
use crate::proto;
use crate::AttributeData;
use cgmath::{BaseFloat, Vector3, Zero};
use num::clamp;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PositionEncoding {
    Uint8,
    Uint16,
    Float32,
    Float64,
}

impl PositionEncoding {
    pub fn new(bounding_cube: &Cube, resolution: f64) -> PositionEncoding {
        let min_bits = (bounding_cube.edge_length() / resolution).log2() as u32 + 1;
        match min_bits {
            0..=8 => PositionEncoding::Uint8,
            9..=16 => PositionEncoding::Uint16,
            // Capping at 24 keeps the worst resolution at ~1 mm for an edge length of ~8389 km.
            17..=24 => PositionEncoding::Float32,
            _ => PositionEncoding::Float64,
        }
    }

    // TODO(sirver): Returning a Result here makes this function more expensive than needed - since
    // we require stack space for the full Result. This should be fixable to moving to failure.
    pub fn from_proto(proto: proto::PositionEncoding) -> Result<Self> {
        match proto {
            proto::PositionEncoding::Uint8 => Ok(PositionEncoding::Uint8),
            proto::PositionEncoding::Uint16 => Ok(PositionEncoding::Uint16),
            proto::PositionEncoding::Float32 => Ok(PositionEncoding::Float32),
            proto::PositionEncoding::Float64 => Ok(PositionEncoding::Float64),
            proto::PositionEncoding::INVALID => Err(ErrorKind::InvalidInput(
                "Proto: PositionEncoding is invalid".to_string(),
            )
            .into()),
        }
    }

    pub fn to_proto(&self) -> proto::PositionEncoding {
        match *self {
            PositionEncoding::Uint8 => proto::PositionEncoding::Uint8,
            PositionEncoding::Uint16 => proto::PositionEncoding::Uint16,
            PositionEncoding::Float32 => proto::PositionEncoding::Float32,
            PositionEncoding::Float64 => proto::PositionEncoding::Float64,
        }
    }

    pub fn bytes_per_coordinate(&self) -> usize {
        match *self {
            PositionEncoding::Uint8 => 1,
            PositionEncoding::Uint16 => 2,
            PositionEncoding::Float32 => 4,
            PositionEncoding::Float64 => 8,
        }
    }
}

#[derive(Clone)]
pub enum Encoding {
    Plain,
    ScaledToCube(Vector3<f64>, f64, PositionEncoding),
}

pub fn fixpoint_encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let value =
        clamp((value - min) / edge_length, 0., 1.) * num::cast::<T, f64>(T::max_value()).unwrap();
    num::cast(value).unwrap()
}

pub fn encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: num_traits::NumCast,
{
    num::cast(clamp((value - min) / edge_length, 0., 1.)).unwrap()
}

pub fn vec3_fixpoint_encode<T>(
    value: &Vector3<f64>,
    min: &Vector3<f64>,
    edge_length: f64,
) -> Vector3<T>
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let value = math::clamp(
        (value - min) / edge_length,
        Vector3::zero(),
        Vector3::new(1.0, 1.0, 1.0),
    ) * num::cast::<T, f64>(T::max_value()).unwrap();
    value.cast::<T>().unwrap()
}

pub fn vec3_encode<T>(value: &Vector3<f64>, min: &Vector3<f64>, edge_length: f64) -> Vector3<T>
where
    T: BaseFloat,
{
    math::clamp(
        (value - min) / edge_length,
        Vector3::zero(),
        Vector3::new(1.0, 1.0, 1.0),
    )
    .cast::<T>()
    .unwrap()
}

pub fn fixpoint_decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let max: f64 = num::cast(T::max_value()).unwrap();
    let v: f64 = num::cast(value).unwrap();
    v / max * edge_length + min
}

pub fn decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: num_traits::NumCast,
{
    num::cast::<T, f64>(value).unwrap() * edge_length + min
}

pub fn attribute_to_proto(attribute: &AttributeData) -> proto::AttributeType {
    match attribute {
        AttributeData::I64(_) => proto::AttributeType::I64,
        AttributeData::U64(_) => proto::AttributeType::U64,
        AttributeData::F32(_) => proto::AttributeType::F32,
        AttributeData::F64(_) => proto::AttributeType::F64,
        AttributeData::U8Vec3(_) => proto::AttributeType::U8Vec3,
        AttributeData::F64Vec3(_) => proto::AttributeType::F64Vec3,
    }
}
