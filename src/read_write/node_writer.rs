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
use crate::Point;
use byteorder::{LittleEndian, WriteBytesExt};
use cgmath::Vector3;
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
    fn write_le(self, writer: &mut DataWriter);
}

impl WriteLE for f32 {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_f32::<LittleEndian>(self).unwrap()
    }
}

impl WriteLE for Vector3<f64> {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_f64::<LittleEndian>(self.x).unwrap();
        writer.write_f64::<LittleEndian>(self.y).unwrap();
        writer.write_f64::<LittleEndian>(self.z).unwrap();
    }
}

impl WriteLE for Color<u8> {
    fn write_le(self, writer: &mut DataWriter) {
        writer.write_u8(self.red).unwrap();
        writer.write_u8(self.green).unwrap();
        writer.write_u8(self.blue).unwrap();
    }
}

pub trait NodeWriter {
    fn write(&mut self, p: &Point);
}
