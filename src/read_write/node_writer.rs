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
use crate::AttributeData;
use byteorder::{ByteOrder, LittleEndian, WriteBytesExt};
use cgmath::{Vector3, Vector4};
use std::fs::{remove_file, File};
use std::io::{BufWriter, Result, Seek, SeekFrom, Write};
use std::path::PathBuf;

pub struct DataWriter {
    inner: BufWriter<File>,
    bytes_written: usize,
    path: PathBuf,
}

impl DataWriter {
    pub fn new(path: impl Into<PathBuf>) -> Result<Self> {
        let path = path.into();
        File::create(&path).map(|w| DataWriter {
            inner: BufWriter::new(w),
            bytes_written: 0,
            path,
        })
    }

    pub fn bytes_written(&self) -> usize {
        self.bytes_written
    }
}

impl Write for DataWriter {
    fn write(&mut self, buf: &[u8]) -> Result<usize> {
        let res = self.inner.write(buf);
        if let Ok(size) = res {
            self.bytes_written += size;
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

impl WriteLE for Vector4<u8> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        writer.write_all(&self[..])
    }
}

impl WriteLE for Color<u8> {
    fn write_le(&self, writer: &mut DataWriter) -> Result<()> {
        writer.write_u8(self.red)?;
        writer.write_u8(self.green)?;
        writer.write_u8(self.blue)?;
        writer.write_u8(self.alpha)
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

impl WriteLE for Vec<Vector4<u8>> {
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
            AttributeData::I64(data) => data.write_le(writer),
            AttributeData::U64(data) => data.write_le(writer),
            AttributeData::F32(data) => data.write_le(writer),
            AttributeData::F64(data) => data.write_le(writer),
            AttributeData::F64Vec3(data) => data.write_le(writer),
            AttributeData::U8Vec4(data) => data.write_le(writer),
        }
    }
}

pub trait WriteLEPos {
    fn write_le_pos(&self, pos: usize, writer: &mut DataWriter) -> Result<()>;
}

impl WriteLEPos for AttributeData {
    fn write_le_pos(&self, pos: usize, writer: &mut DataWriter) -> Result<()> {
        match self {
            AttributeData::I64(data) => data[pos].write_le(writer),
            AttributeData::U64(data) => data[pos].write_le(writer),
            AttributeData::F32(data) => data[pos].write_le(writer),
            AttributeData::F64(data) => data[pos].write_le(writer),
            AttributeData::F64Vec3(data) => data[pos].write_le(writer),
            AttributeData::U8Vec4(data) => data[pos].write_le(writer),
        }
    }
}

pub trait NodeWriter<P> {
    fn write(&mut self, p: &P) -> Result<()>;
}
