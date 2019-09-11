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

use crate::color::Color;
use crate::read_write::{vec3_encode, vec3_fixpoint_encode, Encoding, PositionEncoding};
use crate::AttributeData;
use byteorder::{ByteOrder, LittleEndian, WriteBytesExt};
use cgmath::Vector3;
use std::fs::{remove_file, File, OpenOptions};
use std::io::{BufWriter, Result, Seek, SeekFrom, Write};
use std::path::PathBuf;

#[derive(Clone, Copy, PartialEq)]
pub enum OpenMode {
    Truncate,
    Append,
}

pub struct DataWriter {
    inner: BufWriter<File>,
    bytes_written: u64,
    path: PathBuf,
}

impl DataWriter {
    pub fn new(path: impl Into<PathBuf>, open_mode: OpenMode) -> Result<Self> {
        let path = path.into();
        let mut inner = OpenOptions::new()
            .write(true)
            .create(true)
            .truncate(open_mode == OpenMode::Truncate)
            .open(&path)
            .map(BufWriter::new)?;
        let bytes_written = inner.seek(SeekFrom::End(0))?;
        Ok(DataWriter {
            inner,
            bytes_written,
            path,
        })
    }

    pub fn bytes_written(&self) -> u64 {
        self.bytes_written
    }
}

impl Write for DataWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let res = self.inner.write(buf);
        if let Ok(size) = res {
            self.bytes_written += size as u64;
        }
        res
    }

    fn flush(&mut self) -> Result<()> {
        self.inner.flush()
    }
}

impl Seek for DataWriter {
    fn seek(&mut self, pos: SeekFrom) -> Result<u64> {
        self.inner.seek(pos)
    }
}

impl Drop for DataWriter {
    fn drop(&mut self) {
        // If we did not write anything into this node, it should not exist.
        if self.bytes_written == 0 {
            // We are ignoring deletion errors here in case the file is already gone.
            let _ = remove_file(&self.path);
        }

        // TODO(hrapp): Add some sanity checks that we do not have nodes with ridiculously low
        // amount of points laying around?
    }
}

pub trait WriteLE {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()>;
}

macro_rules! derive_write_le {
    ($scalar:ty, $method:ident) => {
        impl WriteLE for $scalar {
            fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
                writer.$method::<LittleEndian>(*self)
            }
        }
    };
}

macro_rules! derive_write_le_vec {
    ($scalar:ty, $method:ident) => {
        impl WriteLE for Vec<$scalar> {
            fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
                let mut bytes = vec![0; std::mem::size_of::<$scalar>() * self.len()];
                LittleEndian::$method(self, &mut bytes);
                writer.write_all(&bytes)
            }
        }
    };
}

derive_write_le!(f32, write_f32);
derive_write_le!(f64, write_f64);
derive_write_le!(i64, write_i64);
derive_write_le!(u64, write_u64);
derive_write_le_vec!(f32, write_f32_into);
derive_write_le_vec!(f64, write_f64_into);
derive_write_le_vec!(i64, write_i64_into);
derive_write_le_vec!(u64, write_u64_into);

impl WriteLE for Vector3<u8> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        writer.write_all(&self[..])
    }
}

impl WriteLE for Vector3<u16> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        let mut bytes = [0; 6];
        LittleEndian::write_u16_into(&self[..], &mut bytes);
        writer.write_all(&bytes)
    }
}

impl WriteLE for Vector3<f32> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        let mut bytes = [0; 12];
        LittleEndian::write_f32_into(&self[..], &mut bytes);
        writer.write_all(&bytes)
    }
}

impl WriteLE for Vector3<f64> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        let mut bytes = [0; 24];
        LittleEndian::write_f64_into(&self[..], &mut bytes);
        writer.write_all(&bytes)
    }
}

impl WriteLE for Color<u8> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        writer.write_u8(self.red)?;
        writer.write_u8(self.green)?;
        writer.write_u8(self.blue)
        // Alpha is not written on purpose to be compatible with old versions and not waste space.
    }
}

impl WriteLE for Vec<Vector3<u8>> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        for elem in self {
            elem.write_le(writer)?;
        }
        Ok(())
    }
}

impl WriteLE for Vec<Vector3<f64>> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        for elem in self {
            elem.write_le(writer)?;
        }
        Ok(())
    }
}

impl WriteLE for AttributeData {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        match self {
            AttributeData::U8(data) => writer.write_all(data),
            AttributeData::I64(data) => data.write_le(writer),
            AttributeData::U64(data) => data.write_le(writer),
            AttributeData::F32(data) => data.write_le(writer),
            AttributeData::F64(data) => data.write_le(writer),
            AttributeData::U8Vec3(data) => data.write_le(writer),
            AttributeData::F64Vec3(data) => data.write_le(writer),
        }
    }
}

pub trait WriteLEPos {
    fn write_le_pos(&self, pos: usize, writer: &mut DataWriter) -> Result<()>;
}

impl WriteLEPos for AttributeData {
    fn write_le_pos(&self, pos: usize, writer: &mut DataWriter) -> Result<()> {
        match self {
            AttributeData::U8(data) => writer.write_u8(data[pos]),
            AttributeData::I64(data) => data[pos].write_le(writer),
            AttributeData::U64(data) => data[pos].write_le(writer),
            AttributeData::F32(data) => data[pos].write_le(writer),
            AttributeData::F64(data) => data[pos].write_le(writer),
            AttributeData::U8Vec3(data) => data[pos].write_le(writer),
            AttributeData::F64Vec3(data) => data[pos].write_le(writer),
        }
    }
}

pub trait WriteEncoded {
    fn write_encoded(&self, encoding: &Encoding, writer: &mut DataWriter) -> Result<()>;
}

impl WriteEncoded for Vector3<f64> {
    fn write_encoded(&self, encoding: &Encoding, writer: &mut DataWriter) -> Result<()> {
        match encoding {
            Encoding::Plain => self.write_le(writer),
            Encoding::ScaledToCube(min, edge_length, position_encoding) => {
                // Note that due to floating point rounding errors while calculating bounding boxes, it
                // could be here that 'p' is not quite inside the bounding box of our node.
                match position_encoding {
                    PositionEncoding::Uint8 => {
                        vec3_fixpoint_encode::<u8>(self, min, *edge_length).write_le(writer)
                    }
                    PositionEncoding::Uint16 => {
                        vec3_fixpoint_encode::<u16>(self, min, *edge_length).write_le(writer)
                    }
                    PositionEncoding::Float32 => {
                        vec3_encode::<f32>(self, min, *edge_length).write_le(writer)
                    }
                    PositionEncoding::Float64 => {
                        vec3_encode::<f64>(self, min, *edge_length).write_le(writer)
                    }
                }
            }
        }
    }
}

impl WriteEncoded for Vec<Vector3<f64>> {
    fn write_encoded(&self, encoding: &Encoding, writer: &mut DataWriter) -> Result<()> {
        match encoding {
            Encoding::Plain => self.write_le(writer),
            Encoding::ScaledToCube(min, edge_length, position_encoding) => {
                // Note that due to floating point rounding errors while calculating bounding boxes, it
                // could be here that 'p' is not quite inside the bounding box of our node.
                match position_encoding {
                    PositionEncoding::Uint8 => {
                        for position in self {
                            vec3_fixpoint_encode::<u8>(position, min, *edge_length)
                                .write_le(writer)?;
                        }
                    }
                    PositionEncoding::Uint16 => {
                        for position in self {
                            vec3_fixpoint_encode::<u16>(position, min, *edge_length)
                                .write_le(writer)?;
                        }
                    }
                    PositionEncoding::Float32 => {
                        for position in self {
                            vec3_encode::<f32>(position, min, *edge_length).write_le(writer)?;
                        }
                    }
                    PositionEncoding::Float64 => {
                        for position in self {
                            vec3_encode::<f64>(position, min, *edge_length).write_le(writer)?;
                        }
                    }
                }
                Ok(())
            }
        }
    }
}

pub trait NodeWriter<P> {
    fn new(path: impl Into<PathBuf>, codec: Encoding, open_mode: OpenMode) -> Self;
    fn write(&mut self, p: &P) -> Result<usize>;
}
