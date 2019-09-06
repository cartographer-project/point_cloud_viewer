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
use crate::read_write::{
    decode, fixpoint_decode, AttributeReader, DataWriter, Encoding, NodeReader, NodeWriter,
    OpenMode, PositionEncoding, WriteEncoded, WriteLE,
};
use crate::{attribute_extension, AttributeData, AttributeDataType, Point, PointsBatch};
use byteorder::{LittleEndian, ReadBytesExt};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::collections::{BTreeMap, HashMap};
use std::io::{self, BufReader, ErrorKind, Read};
use std::path::PathBuf;

pub struct RawNodeReader {
    xyz_reader: BufReader<Box<dyn Read>>,
    attribute_readers: HashMap<String, AttributeReader>,
    encoding: Encoding,
}

impl NodeReader for RawNodeReader {
    fn read(&mut self) -> io::Result<Point> {
        let mut point = Point {
            position: Vector3::zero(),
            color: color::RED.to_u8(), // is overwritten
            intensity: None,
        };

        // I tried pulling out this match by taking a function pointer to a 'decode_position'
        // function. This replaces a branch per point vs a function call per point and turned
        // out to be marginally slower.
        match self.encoding {
            Encoding::Plain => {
                point.position.x = self.xyz_reader.read_f64::<LittleEndian>()?;
                point.position.y = self.xyz_reader.read_f64::<LittleEndian>()?;
                point.position.z = self.xyz_reader.read_f64::<LittleEndian>()?;
            }
            Encoding::ScaledToCube(min, edge_length, ref pos) => match pos {
                PositionEncoding::Uint8 => {
                    point.position.x =
                        fixpoint_decode(self.xyz_reader.read_u8()?, min.x, edge_length);
                    point.position.y =
                        fixpoint_decode(self.xyz_reader.read_u8()?, min.y, edge_length);
                    point.position.z =
                        fixpoint_decode(self.xyz_reader.read_u8()?, min.z, edge_length);
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
            },
        }

        if let Some(cr) = self.attribute_readers.get_mut("color") {
            point.color.red = cr.reader.read_u8()?;
            point.color.green = cr.reader.read_u8()?;
            point.color.blue = cr.reader.read_u8()?;
        }

        if let Some(ir) = self.attribute_readers.get_mut("intensity") {
            point.intensity = Some(ir.reader.read_f32::<LittleEndian>()?);
        }

        Ok(point)
    }
}

impl RawNodeReader {
    pub fn read_batch(&mut self, num_points: usize) -> io::Result<PointsBatch> {
        let mut batch = PointsBatch {
            position: vec![],
            attributes: BTreeMap::new(),
        };

        match self.encoding {
            Encoding::Plain => (0..num_points).try_for_each(|_| -> io::Result<()> {
                let x = self.xyz_reader.read_f64::<LittleEndian>()?;
                let y = self.xyz_reader.read_f64::<LittleEndian>()?;
                let z = self.xyz_reader.read_f64::<LittleEndian>()?;
                batch.position.push(Vector3::new(x, y, z));
                Ok(())
            })?,
            Encoding::ScaledToCube(min, edge_length, ref pos) => match pos {
                PositionEncoding::Uint8 => (0..num_points).try_for_each(|_| -> io::Result<()> {
                    let x = fixpoint_decode(self.xyz_reader.read_u8()?, min.x, edge_length);
                    let y = fixpoint_decode(self.xyz_reader.read_u8()?, min.y, edge_length);
                    let z = fixpoint_decode(self.xyz_reader.read_u8()?, min.z, edge_length);
                    batch.position.push(Vector3::new(x, y, z));
                    Ok(())
                })?,

                PositionEncoding::Uint16 => {
                    (0..num_points).try_for_each(|_| -> io::Result<()> {
                        let x = fixpoint_decode(
                            self.xyz_reader.read_u16::<LittleEndian>()?,
                            min.x,
                            edge_length,
                        );
                        let y = fixpoint_decode(
                            self.xyz_reader.read_u16::<LittleEndian>()?,
                            min.y,
                            edge_length,
                        );
                        let z = fixpoint_decode(
                            self.xyz_reader.read_u16::<LittleEndian>()?,
                            min.z,
                            edge_length,
                        );
                        batch.position.push(Vector3::new(x, y, z));
                        Ok(())
                    })?
                }

                PositionEncoding::Float32 => {
                    (0..num_points).try_for_each(|_| -> io::Result<()> {
                        let x = decode(
                            self.xyz_reader.read_f32::<LittleEndian>()?,
                            min.x,
                            edge_length,
                        );
                        let y = decode(
                            self.xyz_reader.read_f32::<LittleEndian>()?,
                            min.y,
                            edge_length,
                        );
                        let z = decode(
                            self.xyz_reader.read_f32::<LittleEndian>()?,
                            min.z,
                            edge_length,
                        );
                        batch.position.push(Vector3::new(x, y, z));
                        Ok(())
                    })?
                }

                PositionEncoding::Float64 => {
                    (0..num_points).try_for_each(|_| -> io::Result<()> {
                        let x = decode(
                            self.xyz_reader.read_f64::<LittleEndian>()?,
                            min.x,
                            edge_length,
                        );
                        let y = decode(
                            self.xyz_reader.read_f64::<LittleEndian>()?,
                            min.y,
                            edge_length,
                        );
                        let z = decode(
                            self.xyz_reader.read_f64::<LittleEndian>()?,
                            min.z,
                            edge_length,
                        );
                        batch.position.push(Vector3::new(x, y, z));
                        Ok(())
                    })?
                }
            },
        };

        self.attribute_readers.iter_mut().try_for_each(
            |(key, AttributeReader { data_type, reader })| -> io::Result<()> {
                match data_type {
                    AttributeDataType::U8 => {
                        let mut attr = Vec::with_capacity(num_points);
                        reader.read_exact(&mut attr)?;
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::U8(attr));
                    }
                    AttributeDataType::I64 => {
                        let mut attr = Vec::with_capacity(num_points);
                        reader.read_i64_into::<LittleEndian>(&mut attr)?;
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::I64(attr));
                    }
                    AttributeDataType::U64 => {
                        let mut attr = Vec::with_capacity(num_points);
                        reader.read_u64_into::<LittleEndian>(&mut attr)?;
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::U64(attr));
                    }
                    AttributeDataType::F32 => {
                        let mut attr = Vec::with_capacity(num_points);
                        reader.read_f32_into::<LittleEndian>(&mut attr)?;
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::F32(attr));
                    }
                    AttributeDataType::F64 => {
                        let mut attr = Vec::with_capacity(num_points);
                        reader.read_f64_into::<LittleEndian>(&mut attr)?;
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::F64(attr));
                    }
                    AttributeDataType::U8Vec3 => {
                        let mut attr = Vec::with_capacity(num_points);
                        let mut buffer = Vec::with_capacity(3 * num_points);
                        reader.read_exact(&mut buffer)?;
                        for i in 0..num_points {
                            attr.push(Vector3::new(buffer[i], buffer[i + 1], buffer[i + 2]));
                        }
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::U8Vec3(attr));
                    }
                    AttributeDataType::F64Vec3 => {
                        let mut attr = Vec::with_capacity(num_points);
                        let mut buffer = Vec::with_capacity(3 * num_points);
                        reader.read_f64_into::<LittleEndian>(&mut buffer)?;
                        for i in 0..num_points {
                            attr.push(Vector3::new(buffer[i], buffer[i + 1], buffer[i + 2]));
                        }
                        batch
                            .attributes
                            .insert(key.to_owned(), AttributeData::F64Vec3(attr));
                    }
                };
                Ok(())
            },
        )?;

        let num_points = batch.position.len();

        // If the attributes differ in length, something was wrong with the files.
        if batch
            .attributes
            .values()
            .all(|attr| attr.len() == num_points)
        {
            Ok(batch)
        } else {
            Err(io::Error::new(
                ErrorKind::InvalidData,
                "Attributes differ in length",
            ))
        }
    }
}

impl RawNodeReader {
    pub fn new(
        xyz_reader: Box<dyn Read>,
        attribute_readers: HashMap<String, AttributeReader>,
        encoding: Encoding,
    ) -> Result<Self> {
        let xyz_reader = BufReader::new(xyz_reader);

        Ok(Self {
            xyz_reader,
            attribute_readers,
            encoding,
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
