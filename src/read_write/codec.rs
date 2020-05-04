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
use crate::geometry::Cube;
use crate::proto;
use nalgebra::{Point3, Scalar, Vector3};
use num::clamp;
use std::fmt::Debug;

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

/// The _encode and _decode functions are not methods of this type, because
/// we are sometimes able to pull the match outside the inner loop.
#[derive(Clone)]
pub enum Encoding {
    Plain,
    ScaledToCube(Point3<f64>, f64, PositionEncoding),
}

/// Encode float as integer.
pub fn fixpoint_encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: num_traits::PrimInt + num_traits::Bounded + simba::scalar::SubsetOf<f64>,
{
    let value =
        clamp((value - min) / edge_length, 0., 1.) * nalgebra::convert::<T, f64>(T::max_value());
    nalgebra::try_convert(value).unwrap()
}

/// Encode float as f32 or f64 unit interval float.
pub fn _encode<T>(value: f64, min: f64, edge_length: f64) -> T
where
    T: simba::scalar::SubsetOf<f64>,
{
    nalgebra::try_convert(clamp((value - min) / edge_length, 0., 1.)).unwrap()
}

pub fn vec3_fixpoint_encode<T>(
    value: &Point3<f64>,
    min: &Point3<f64>,
    edge_length: f64,
) -> Vector3<T>
where
    T: Scalar + num_traits::PrimInt + num_traits::Bounded + simba::scalar::SubsetOf<f64>,
{
    let scale: f64 = nalgebra::convert(T::max_value());
    let value = clamp_elementwise((value - min) / edge_length, 0.0, 1.0);
    nalgebra::try_convert(scale * value).unwrap()
}

pub fn vec3_encode<T>(value: &Point3<f64>, min: &Point3<f64>, edge_length: f64) -> Vector3<T>
where
    T: Scalar + simba::scalar::SubsetOf<f64>,
{
    let value = clamp_elementwise((value - min) / edge_length, 0.0, 1.0);
    nalgebra::try_convert(value).unwrap()
}

/// Decode integer as float.
pub fn fixpoint_decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: num_traits::PrimInt + num_traits::Bounded + simba::scalar::SubsetOf<f64>,
{
    let max: f64 = nalgebra::convert(T::max_value());
    let v: f64 = nalgebra::convert(value);
    (v / max).mul_add(edge_length, min)
}

/// Decode f32 or f64 unit interval float as f64.
pub fn decode<T>(value: T, min: f64, edge_length: f64) -> f64
where
    T: simba::scalar::SubsetOf<f64>,
{
    nalgebra::convert::<T, f64>(value).mul_add(edge_length, min)
}

// Careful: num's (or nalgebra's) clamp accepts Vector3 too, but does not work elementwise like this
fn clamp_elementwise(value: Vector3<f64>, lower: f64, upper: f64) -> Vector3<f64> {
    Vector3::new(
        clamp(value.x, lower, upper),
        clamp(value.y, lower, upper),
        clamp(value.z, lower, upper),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn plain_scalar() {
        let value = 41.33333;
        let min = 40.0;
        let edge_length = 2.0;

        let value_f32 = _encode::<f32>(value, min, edge_length);
        let value_f32_decoded = decode(value_f32, min, edge_length);
        // We could use the approx crate here and elsewhere
        assert!(
            (value_f32_decoded - value).abs() < 1e-7,
            "Reconstructed from f32: {}, original: {}",
            value_f32_decoded,
            value
        );

        let value_f64 = _encode::<f64>(value, min, edge_length);
        let value_f64_decoded = decode(value_f64, min, edge_length);
        assert!(
            (value_f64_decoded - value).abs() < 1e-14,
            "Reconstructed from f64: {}, original: {}",
            value_f64_decoded,
            value
        );
    }

    #[test]
    fn fixpoint_scalar() {
        let value = 41.33333;
        let min = 40.0;
        let edge_length = 2.0;

        let value_u8 = fixpoint_encode::<u8>(value, min, edge_length);
        let value_u8_decoded = fixpoint_decode(value_u8, min, edge_length);
        assert!(
            (value_u8_decoded - value).abs() < 1e-2,
            "Reconstructed from u8: {}, original: {}",
            value_u8_decoded,
            value
        );

        let value_u16 = fixpoint_encode::<u16>(value, min, edge_length);
        let value_u16_decoded = fixpoint_decode(value_u16, min, edge_length);
        assert!(
            (value_u16_decoded - value).abs() < 1e-4,
            "Reconstructed from u16: {}, original: {}",
            value_u16_decoded,
            value
        );

        let value_u32 = fixpoint_encode::<u32>(value, min, edge_length);
        let value_u32_decoded = fixpoint_decode(value_u32, min, edge_length);
        assert!(
            (value_u32_decoded - value).abs() < 1e-7,
            "Reconstructed from u32: {}, original: {}",
            value_u32_decoded,
            value
        );
    }
}
