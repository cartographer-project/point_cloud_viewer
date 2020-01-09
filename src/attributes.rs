use crate::errors::{ErrorKind, Result};
use cgmath::Vector3;
use std::convert::TryFrom;

pub use point_viewer_proto_rust::proto;

#[derive(Copy, Debug, Clone, PartialEq, Eq)]
pub enum AttributeDataType {
    U8,
    U16,
    U32,
    U64,
    I8,
    I16,
    I32,
    I64,
    F32,
    F64,
    U8Vec3,
    F64Vec3,
}

impl AttributeDataType {
    pub fn to_proto(self) -> proto::AttributeDataType {
        match self {
            AttributeDataType::U8 => proto::AttributeDataType::U8,
            AttributeDataType::U16 => proto::AttributeDataType::U16,
            AttributeDataType::U32 => proto::AttributeDataType::U32,
            AttributeDataType::U64 => proto::AttributeDataType::U64,
            AttributeDataType::I8 => proto::AttributeDataType::I8,
            AttributeDataType::I16 => proto::AttributeDataType::I16,
            AttributeDataType::I32 => proto::AttributeDataType::I32,
            AttributeDataType::I64 => proto::AttributeDataType::I64,
            AttributeDataType::F32 => proto::AttributeDataType::F32,
            AttributeDataType::F64 => proto::AttributeDataType::F64,
            AttributeDataType::U8Vec3 => proto::AttributeDataType::U8Vec3,
            AttributeDataType::F64Vec3 => proto::AttributeDataType::F64Vec3,
        }
    }

    pub fn from_proto(attr_proto: proto::AttributeDataType) -> Result<Self> {
        let attr = match attr_proto {
            proto::AttributeDataType::U8 => AttributeDataType::U8,
            proto::AttributeDataType::U16 => AttributeDataType::U16,
            proto::AttributeDataType::U32 => AttributeDataType::U32,
            proto::AttributeDataType::U64 => AttributeDataType::U64,
            proto::AttributeDataType::I8 => AttributeDataType::I8,
            proto::AttributeDataType::I16 => AttributeDataType::I16,
            proto::AttributeDataType::I32 => AttributeDataType::I32,
            proto::AttributeDataType::I64 => AttributeDataType::I64,
            proto::AttributeDataType::F32 => AttributeDataType::F32,
            proto::AttributeDataType::F64 => AttributeDataType::F64,
            proto::AttributeDataType::U8Vec3 => AttributeDataType::U8Vec3,
            proto::AttributeDataType::F64Vec3 => AttributeDataType::F64Vec3,
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
            AttributeDataType::U8 | AttributeDataType::I8 => 1,
            AttributeDataType::U16 | AttributeDataType::I16 => 2,
            AttributeDataType::U32 | AttributeDataType::I32 | AttributeDataType::F32 => 4,
            AttributeDataType::U64 | AttributeDataType::I64 | AttributeDataType::F64 => 8,
            AttributeDataType::U8Vec3 => 3,
            AttributeDataType::F64Vec3 => 3 * 8,
        }
    }
}

/// General field to describe point feature attributes such as color, intensity, ...
#[derive(Debug, Clone)]
pub enum AttributeData {
    U8(Vec<u8>),
    U16(Vec<u16>),
    U32(Vec<u32>),
    U64(Vec<u64>),
    I8(Vec<i8>),
    I16(Vec<i16>),
    I32(Vec<i32>),
    I64(Vec<i64>),
    F32(Vec<f32>),
    F64(Vec<f64>),
    U8Vec3(Vec<Vector3<u8>>),
    F64Vec3(Vec<Vector3<f64>>),
}

// Convenience macro if you want to operate on the Vec inside an AttributeData
macro_rules! match_attr_data {
    ($x:expr, $match_rhs:tt $(, $arg:tt )* ) => {
        #[allow(unused_variables)]
        match $x {
            AttributeData::U8(d) => $match_rhs!(U8, d $(, $arg )* ),
            AttributeData::U16(d) => $match_rhs!(U16, d $(, $arg )* ),
            AttributeData::U32(d) => $match_rhs!(U32, d $(, $arg )* ),
            AttributeData::U64(d) => $match_rhs!(U64, d $(, $arg )* ),
            AttributeData::I8(d) => $match_rhs!(I8, d $(, $arg )* ),
            AttributeData::I16(d) => $match_rhs!(I16, d $(, $arg )* ),
            AttributeData::I32(d) => $match_rhs!(I32, d $(, $arg )* ),
            AttributeData::I64(d) => $match_rhs!(I64, d $(, $arg )* ),
            AttributeData::F32(d) => $match_rhs!(F32, d $(, $arg )* ),
            AttributeData::F64(d) => $match_rhs!(F64, d $(, $arg )* ),
            AttributeData::U8Vec3(d) => $match_rhs!(U8Vec3, d $(, $arg )* ),
            AttributeData::F64Vec3(d) => $match_rhs!(F64Vec3, d $(, $arg )* ),
        }
    };
}

impl AttributeData {
    pub fn len(&self) -> usize {
        macro_rules! rhs {
            ($dtype:ident, $data:ident) => {
                $data.len()
            };
        }
        match_attr_data!(self, rhs)
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn dim(&self) -> usize {
        match self {
            AttributeData::U8(_)
            | AttributeData::U16(_)
            | AttributeData::U32(_)
            | AttributeData::U64(_)
            | AttributeData::I8(_)
            | AttributeData::I16(_)
            | AttributeData::I32(_)
            | AttributeData::I64(_)
            | AttributeData::F32(_)
            | AttributeData::F64(_) => 1,
            AttributeData::U8Vec3(_) | AttributeData::F64Vec3(_) => 3,
        }
    }

    pub fn data_type(&self) -> AttributeDataType {
        macro_rules! rhs {
            ($dtype:ident, $data:ident) => {
                AttributeDataType::$dtype
            };
        }
        match_attr_data!(self, rhs)
    }

    pub fn append(&mut self, other: &mut Self) -> std::result::Result<(), String> {
        match (self, other) {
            (AttributeData::U8(s), AttributeData::U8(o)) => s.append(o),
            (AttributeData::U16(s), AttributeData::U16(o)) => s.append(o),
            (AttributeData::U32(s), AttributeData::U32(o)) => s.append(o),
            (AttributeData::U64(s), AttributeData::U64(o)) => s.append(o),
            (AttributeData::I8(s), AttributeData::I8(o)) => s.append(o),
            (AttributeData::I16(s), AttributeData::I16(o)) => s.append(o),
            (AttributeData::I32(s), AttributeData::I32(o)) => s.append(o),
            (AttributeData::I64(s), AttributeData::I64(o)) => s.append(o),
            (AttributeData::F32(s), AttributeData::F32(o)) => s.append(o),
            (AttributeData::F64(s), AttributeData::F64(o)) => s.append(o),
            (AttributeData::U8Vec3(s), AttributeData::U8Vec3(o)) => s.append(o),
            (AttributeData::F64Vec3(s), AttributeData::F64Vec3(o)) => s.append(o),
            (s, o) => {
                return Err(format!(
                    "Own data type '{:?}' is incompatible with other type '{:?}'.",
                    s.data_type(),
                    o.data_type(),
                ))
            }
        };
        Ok(())
    }

    pub fn split_off(&mut self, at: usize) -> Self {
        macro_rules! rhs {
            ($dtype:ident, $data:ident, $at:expr) => {
                AttributeData::$dtype($data.split_off($at))
            };
        }
        match_attr_data!(self, rhs, at)
    }

    pub fn get(&self, idx: usize) -> Self {
        macro_rules! rhs {
            ($dtype:ident, $data:ident, $idx:expr) => {
                AttributeData::$dtype(vec![$data[idx]])
            };
        }
        match_attr_data!(self, rhs, idx)
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
        // By value
        impl TryFrom<AttributeData> for Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }

        // By shared reference
        impl<'a> TryFrom<&'a AttributeData> for &'a Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: &'a AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }

        // By mutable reference
        impl<'a> TryFrom<&'a mut AttributeData> for &'a mut Vec<$vec_data_type> {
            type Error = String;

            fn try_from(data: &'a mut AttributeData) -> std::result::Result<Self, Self::Error> {
                try_from_impl!(data, $attribute_data_type, $vec_data_type)
            }
        }
    };
}

try_from_attribute_data!(U8, u8);
try_from_attribute_data!(U16, u16);
try_from_attribute_data!(U32, u32);
try_from_attribute_data!(U64, u64);
try_from_attribute_data!(I8, i8);
try_from_attribute_data!(I16, i16);
try_from_attribute_data!(I32, i32);
try_from_attribute_data!(I64, i64);
try_from_attribute_data!(F32, f32);
try_from_attribute_data!(F64, f64);
try_from_attribute_data!(U8Vec3, Vector3<u8>);
try_from_attribute_data!(F64Vec3, Vector3<f64>);
