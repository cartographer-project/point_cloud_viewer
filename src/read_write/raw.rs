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

use crate::color;
use crate::errors::*;
use crate::math::Cube;
use crate::read_write::{
    decode, fixpoint_decode, DataWriter, Encoding, NodeReader, NodeWriter, OpenMode,
    PositionEncoding, WriteEncoded, WriteLE,
};
use crate::{attribute_extension, Point, PointsBatch};
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::collections::HashMap;
use std::io::{self, BufReader, Read};
use std::path::PathBuf;

pub struct RawNodeReader {
    xyz_reader: BufReader<Box<dyn Read>>,
    attribute_readers: Vec<BufReader<Box<dyn Read>>>,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeReader for RawNodeReader {
    fn read(&mut self) -> io::Result<Point> {
        let mut point = Point {
            position: Vector3::zero(),
            color: color::RED.to_u8(), // is overwritten
            intensity: None,
        };

        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();

        // I tried pulling out this match by taking a function pointer to a 'decode_position'
        // function. This replaces a branch per point vs a function call per point and turned
        // out to be marginally slower.
        match self.position_encoding {
            PositionEncoding::Uint8 => {
                point.position.x = fixpoint_decode(self.xyz_reader.read_u8()?, min.x, edge_length);
                point.position.y = fixpoint_decode(self.xyz_reader.read_u8()?, min.y, edge_length);
                point.position.z = fixpoint_decode(self.xyz_reader.read_u8()?, min.z, edge_length);
            }
            PositionEncoding::Uint16 => {
                point.position.x = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = fixpoint_decode(
                    self.xyz_reader.read_u16::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float32 => {
                point.position.x = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f32::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
            PositionEncoding::Float64 => {
                point.position.x = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.x,
                    edge_length,
                );
                point.position.y = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.y,
                    edge_length,
                );
                point.position.z = decode(
                    self.xyz_reader.read_f64::<LittleEndian>()?,
                    min.z,
                    edge_length,
                );
            }
        }

        point.color.red = self.attribute_readers[0].read_u8()?;
        point.color.green = self.attribute_readers[0].read_u8()?;
        point.color.blue = self.attribute_readers[0].read_u8()?;

        if let Some(ir) = self.attribute_readers.get_mut(1) {
            point.intensity = Some(ir.read_f32::<LittleEndian>()?);
        }

        Ok(point)
    }
}

impl RawNodeReader {
    pub fn new(
        mut attributes: HashMap<String, Box<dyn Read>>,
        position_encoding: PositionEncoding,
        bounding_cube: Cube,
    ) -> Result<Self> {
        let xyz_reader = BufReader::new(
            attributes
                .remove("position")
                .ok_or_else(|| "No position reader available.")?,
        );
        let rgb_reader = BufReader::new(
            attributes
                .remove("color")
                .ok_or_else(|| "No color reader available.")?,
        );
        let mut attribute_readers = vec![rgb_reader];

        if let Some(intensity_read) = attributes.remove("intensity") {
            attribute_readers.push(BufReader::new(intensity_read));
        };

        Ok(Self {
            xyz_reader,
            attribute_readers,
            position_encoding,
            bounding_cube,
        })
    }
}

pub struct RawNodeWriter {
    xyz_writer: DataWriter,
    attribute_writers: Vec<DataWriter>,
    stem: PathBuf,
    encoding: Encoding,
    open_mode: OpenMode,
}

impl NodeWriter<PointsBatch> for RawNodeWriter {
    fn new(path: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        Self::new(path, encoding, open_mode)
    }

    fn write(&mut self, p: &PointsBatch) -> io::Result<()> {
        p.position
            .write_encoded(&self.encoding, &mut self.xyz_writer)?;

        if self.attribute_writers.is_empty() {
            for name in p.attributes.keys() {
                self.attribute_writers.push(DataWriter::new(
                    &self.stem.with_extension(attribute_extension(&name)),
                    self.open_mode,
                )?)
            }
        }

        for (i, data) in p.attributes.values().enumerate() {
            data.write_le(&mut self.attribute_writers[i])?;
        }

        Ok(())
    }
}

impl NodeWriter<Point> for RawNodeWriter {
    fn new(path: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        Self::new(path, encoding, open_mode)
    }

    fn write(&mut self, p: &Point) -> io::Result<()> {
        p.position
            .write_encoded(&self.encoding, &mut self.xyz_writer)?;

        if self.attribute_writers.is_empty() {
            self.attribute_writers.push(DataWriter::new(
                &self.stem.with_extension(attribute_extension("color")),
                self.open_mode,
            )?);
            if p.intensity.is_some() {
                self.attribute_writers.push(DataWriter::new(
                    &self.stem.with_extension(attribute_extension("intensity")),
                    self.open_mode,
                )?);
            }
        }
        p.color.write_le(&mut self.attribute_writers[0])?;
        if let Some(i) = p.intensity {
            i.write_le(&mut self.attribute_writers[1])?;
        }

        Ok(())
    }
}

impl RawNodeWriter {
    pub fn new(path: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        let stem: PathBuf = path.into();
        let xyz_writer = DataWriter::new(
            &stem.with_extension(attribute_extension("position")),
            open_mode,
        )
        .unwrap();
        let attribute_writers = Vec::new();
        Self {
            xyz_writer,
            attribute_writers,
            stem,
            encoding,
            open_mode,
        }
    }

    pub fn num_written(&self) -> i64 {
        let bytes_per_coordinate = match &self.encoding {
            Encoding::Plain => std::mem::size_of::<f64>(),
            Encoding::ScaledToCube(_, _, pos_enc) => pos_enc.bytes_per_coordinate(),
        } as i64;
        (self.xyz_writer.bytes_written() as i64 / bytes_per_coordinate / 3)
    }
}
