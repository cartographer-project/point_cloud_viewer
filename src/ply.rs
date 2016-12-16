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

use byteorder::{LittleEndian, ByteOrder};
use math::Vector3f;
use Point;
use std::fs::File;
use std::io::{BufRead, BufReader, SeekFrom, Seek};
use std::ops::Index;
use std::path::Path;
use std::str;
use errors::*;

#[derive(Debug)]
struct Header {
    format: Format,
    elements: Vec<Element>,
}

#[derive(Debug,Copy,Clone,PartialEq)]
enum DataType {
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Float32,
    Float64,
}

impl DataType {
    fn from_str(input: &str) -> Result<Self> {
        match input {
            "float" | "float32" => Ok(DataType::Float32),
            "double" | "float64" => Ok(DataType::Float64),
            "char" | "int8" => Ok(DataType::Int8),
            "uchar" | "uint8" => Ok(DataType::Uint8),
            "short" | "int16" => Ok(DataType::Int16),
            "ushort" | "uint16" => Ok(DataType::Uint16),
            "int" | "int32" => Ok(DataType::Int32),
            "uint" | "uint32" => Ok(DataType::Uint32),
            _ => Err(ErrorKind::InvalidInput(format!("Invalid data type: {}", input)).into()),
        }
    }

    fn size_in_bytes(&self) -> usize {
        match *self {
            DataType::Int8 => 1,
            DataType::Uint8 => 1,
            DataType::Int16 => 2,
            DataType::Uint16 => 2,
            DataType::Int32 => 4,
            DataType::Uint32 => 4,
            DataType::Float32 => 4,
            DataType::Float64 => 8,
        }
    }

    fn read<B: ByteOrder>(&self, slice: &[u8]) -> f32 {
        match *self {
            DataType::Int8 => (slice[0] as i8) as f32,
            DataType::Uint8 => slice[0] as f32,
            DataType::Int16 => B::read_i16(slice) as f32,
            DataType::Uint16 => B::read_u16(slice) as f32,
            DataType::Int32 => B::read_i32(slice) as f32,
            DataType::Uint32 => B::read_u32(slice) as f32,
            DataType::Float32 => B::read_f32(slice) as f32,
            DataType::Float64 => B::read_f64(slice) as f32,
        }
    }
}

impl Header {
    fn has_element(&self, name: &str) -> bool {
        self.elements.iter().any(|e| e.name == name)
    }
}

impl<'a> Index<&'a str> for Header {
    type Output = Element;
    fn index(&self, name: &'a str) -> &Self::Output {
        for element in &self.elements {
            if element.name == name {
                return element;
            }
        }
        panic!("Element {} does not exist.", name);
    }
}

#[derive(Debug,PartialEq)]
enum Format {
    BinaryLittleEndianV1,
    BinaryBigEndianV1,
    AsciiV1,
}

// TODO(hrapp): Maybe support list properties too?
#[derive(Debug)]
struct ScalarProperty {
    name: String,
    data_type: DataType,
}

#[derive(Debug)]
struct Element {
    name: String,
    count: i64,
    properties: Vec<ScalarProperty>,
}

impl<'a> Index<&'a str> for Element {
    type Output = ScalarProperty;
    fn index(&self, name: &'a str) -> &Self::Output {
        for p in &self.properties {
            if p.name == name {
                return p;
            }
        }
        panic!("Property does not exist!")
    }
}

impl Element {
    fn has_property(&self, name: &str) -> bool {
        self.properties.iter().any(|p| p.name == name)
    }
}

fn parse_header<R: BufRead>(reader: &mut R) -> Result<(Header, usize)> {
    use errors::ErrorKind::InvalidInput;

    let mut header_len = 0;
    let mut line = String::new();
    header_len += reader.read_line(&mut line)?;
    if line.trim() != "ply" {
        return Err(InvalidInput("Not a PLY file".to_string()).into());
    }

    let mut format = None;
    let mut current_element = None;
    let mut elements = Vec::new();
    loop {
        line.clear();
        header_len += reader.read_line(&mut line)?;
        let entries: Vec<&str> = line.trim().split_whitespace().collect();
        match entries[0] {
            "format" if entries.len() == 3 => {
                if entries[2] != "1.0" {
                    return Err(InvalidInput(format!("Invalid version: {}", entries[2])).into());
                }
                format = Some(match entries[1] {
                    "ascii" => Format::AsciiV1,
                    "binary_little_endian" => Format::BinaryLittleEndianV1,
                    "binary_big_endian" => Format::BinaryBigEndianV1,
                    _ => return Err(InvalidInput(format!("Invalid format: {}", entries[1])).into()),
                });
            }
            "element" if entries.len() == 3 => {
                if let Some(element) = current_element.take() {
                    elements.push(element);
                }
                current_element = Some(Element {
                    name: entries[1].to_string(),
                    count: entries[2].parse::<i64>()
                        .chain_err(|| InvalidInput(format!("Invalid count: {}", entries[2])))?,
                    properties: Vec::new(),
                });
            }
            "property" => {
                if current_element.is_none() {
                    return Err(InvalidInput(format!("property outside of element: {}", line))
                        .into());
                };
                let property = match entries[1] {
                    "list" if entries.len() == 5 => {
                        // We do not support list properties.
                        continue;
                    }
                    data_type_str if entries.len() == 3 => {
                        let data_type = DataType::from_str(data_type_str)?;
                        ScalarProperty {
                            name: entries[2].to_string(),
                            data_type: data_type,
                        }
                    }
                    _ => return Err(InvalidInput(format!("Invalid line: {}", line)).into()),
                };
                current_element.as_mut().unwrap().properties.push(property);
            }
            "end_header" => break,
            "comment" => (),
            _ => return Err(InvalidInput(format!("Invalid line: {}", line)).into()),
        }
    }

    if let Some(element) = current_element {
        elements.push(element);
    }

    if format.is_none() {
        return Err(InvalidInput("No format specified".into()).into());
    }

    Ok((Header {
            elements: elements,
            format: format.unwrap(),
        },
        header_len))
}

#[derive(Debug)]
struct PointFormat {
    position_data_type: DataType,
    // None, if the file does not contain color.
    color_data_type: Option<DataType>,
}

/// Opens a PLY file and checks that it is the correct format we support. Seeks in the file to the
/// beginning of the binary data which must be (x, y, z, r, g, b) tuples.
fn open(ply_file: &Path) -> Result<(BufReader<File>, i64, PointFormat)> {
    let mut file = File::open(ply_file).chain_err(|| "Could not open input file.")?;
    let mut reader = BufReader::new(file);
    let (header, header_len) = parse_header(&mut reader)?;
    file = reader.into_inner();
    file.seek(SeekFrom::Start(header_len as u64))?;

    if !header.has_element("vertex") {
        panic!("Header does not have element 'vertex'");
    }

    if header.format != Format::BinaryLittleEndianV1 {
        panic!("Unsupported PLY format: {:?}", header.format);
    }

    let vertex = &header["vertex"];
    if !vertex.has_property("x") || !vertex.has_property("y") || !vertex.has_property("z") {
        panic!("PLY must contain properties 'x', 'y', 'z' for 'vertex'.");
    }

    let mut num_bytes_per_point = 0;
    let position_data_type = header["vertex"]["x"].data_type;
    num_bytes_per_point += position_data_type.size_in_bytes() * 3;

    let color_data_type = {
        if vertex.has_property("red") && vertex.has_property("green") &&
           vertex.has_property("blue") {
            Some(vertex["red"].data_type)
        } else if vertex.has_property("r") && vertex.has_property("g") && vertex.has_property("b") {
            Some(vertex["r"].data_type)
        } else {
            None
        }
    };
    if let Some(ref data_type) = color_data_type {
        num_bytes_per_point += data_type.size_in_bytes() * 3;
    }

    let point_format = PointFormat {
        position_data_type: position_data_type,
        color_data_type: color_data_type,
    };

    // We align the buffer of this 'BufReader' to points, so that we can index this buffer and know
    // that it will always contain full points to parse.
    Ok((BufReader::with_capacity(num_bytes_per_point * 1024, file),
        header["vertex"].count,
        point_format))
}


/// Abstraction to read binary points from ply files into points.
pub struct PlyIterator {
    reader: BufReader<File>,
    point_format: PointFormat,
    num_points_read: i64,
    pub num_total_points: i64,
}

impl PlyIterator {
    pub fn new(ply_file: &Path) -> Result<Self> {
        let (reader, num_total_points, point_format) = open(ply_file)?;
        Ok(PlyIterator {
            reader: reader,
            point_format: point_format,
            num_total_points: num_total_points,
            num_points_read: 0,
        })
    }
}

impl Iterator for PlyIterator {
    type Item = Point;

    fn next(&mut self) -> Option<Self::Item> {
        if self.num_points_read >= self.num_total_points {
            return None;
        }

        let mut nread = 0;
        let point = {
            // We made sure before that the internal buffer of 'reader' is aligned to the number of
            // bytes for a single point, therefore we can access it here and know that we can
            // always read into it and are sure that it contains at least a full point.
            let buf = self.reader.fill_buf().unwrap();

            let position = {
                let data_type = &self.point_format.position_data_type;
                let size_in_bytes = data_type.size_in_bytes();
                let x = data_type.read::<LittleEndian>(&buf[nread..]);
                nread += size_in_bytes;
                let y = data_type.read::<LittleEndian>(&buf[nread..]);
                nread += size_in_bytes;
                let z = data_type.read::<LittleEndian>(&buf[nread..]);
                nread += size_in_bytes;
                Vector3f::new(x, y, z)
            };

            let mut r = 255;
            let mut g = 255;
            let mut b = 255;
            if let Some(ref data_type) = self.point_format.color_data_type {
                let size_in_bytes = data_type.size_in_bytes();
                r = data_type.read::<LittleEndian>(&buf[nread..]) as u8;
                nread += size_in_bytes;
                g = data_type.read::<LittleEndian>(&buf[nread..]) as u8;
                nread += size_in_bytes;
                b = data_type.read::<LittleEndian>(&buf[nread..]) as u8;
                nread += size_in_bytes;
            };

            Point {
                position: position,
                r: r,
                g: g,
                b: b,
            }
        };
        self.num_points_read += 1;
        self.reader.consume(nread);
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.num_total_points as usize, Some(self.num_total_points as usize))
    }
}
