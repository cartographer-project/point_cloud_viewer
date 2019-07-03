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
use crate::octree::PositionEncoding;
use crate::read_write::{
    decode, encode, fixpoint_decode, fixpoint_encode, DataWriter, NodeReader, NodeWriter, WriteLE,
};
use crate::{NodeLayer, Point};
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::collections::HashMap;
use std::io::{self, BufReader, Read};
use std::path::PathBuf;

pub struct CubeNodeReader {
    xyz_reader: BufReader<Box<dyn Read>>,
    layer_readers: Vec<BufReader<Box<dyn Read>>>,
    num_points: usize,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeReader for CubeNodeReader {
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

        point.color.red = self.layer_readers[0].read_u8()?;
        point.color.green = self.layer_readers[0].read_u8()?;
        point.color.blue = self.layer_readers[0].read_u8()?;

        if let Some(ir) = self.layer_readers.get_mut(1) {
            point.intensity = Some(ir.read_f32::<LittleEndian>()?);
        }

        Ok(point)
    }

    fn num_points(&self) -> usize {
        self.num_points
    }
}

impl CubeNodeReader {
    pub fn new(
        mut layers: HashMap<NodeLayer, Box<dyn Read>>,
        num_points: usize,
        position_encoding: PositionEncoding,
        bounding_cube: Cube,
    ) -> Result<Self> {
        let xyz_reader = BufReader::new(
            layers
                .remove(&NodeLayer::Position)
                .ok_or_else(|| "No position reader available.")?,
        );
        let rgb_reader = BufReader::new(
            layers
                .remove(&NodeLayer::Color)
                .ok_or_else(|| "No color reader available.")?,
        );
        let mut layer_readers = vec![rgb_reader];

        if let Some(intensity_read) = layers.remove(&NodeLayer::Intensity) {
            layer_readers.push(BufReader::new(intensity_read));
        };

        Ok(Self {
            xyz_reader,
            layer_readers,
            num_points,
            position_encoding,
            bounding_cube,
        })
    }
}

pub struct CubeNodeWriter {
    xyz_writer: DataWriter,
    layer_writers: Vec<DataWriter>,
    stem: PathBuf,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeWriter for CubeNodeWriter {
    fn write(&mut self, p: &Point) {
        // Note that due to floating point rounding errors while calculating bounding boxes, it
        // could be here that 'p' is not quite inside the bounding box of our node.
        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();
        match self.position_encoding {
            PositionEncoding::Uint8 => {
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Uint16 => {
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Float32 => {
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Float64 => {
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f64::<LittleEndian>(encode(p.position.z, min.z, edge_length))
                    .unwrap();
            }
        }

        p.color.write_le(&mut self.layer_writers[0]);
        if let Some(i) = p.intensity {
            if self.layer_writers.len() < 2 {
                self.layer_writers.push(
                    DataWriter::new(&self.stem.with_extension(NodeLayer::Intensity.extension()))
                        .unwrap(),
                );
            }
            i.write_le(&mut self.layer_writers[1]);
        };
    }
}

impl CubeNodeWriter {
    pub fn new(
        path: impl Into<PathBuf>,
        position_encoding: PositionEncoding,
        bounding_cube: Cube,
    ) -> Self {
        let stem: PathBuf = path.into();
        let xyz_writer =
            DataWriter::new(&stem.with_extension(NodeLayer::Position.extension())).unwrap();
        let layer_writers =
            vec![DataWriter::new(&stem.with_extension(NodeLayer::Color.extension())).unwrap()];
        Self {
            xyz_writer,
            layer_writers,
            stem,
            position_encoding,
            bounding_cube,
        }
    }

    pub fn num_written(&self) -> i64 {
        self.layer_writers[0].bytes_written() as i64 / 3
    }
}
