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

use {InternalIterator, Point};
use byteorder::{ByteOrder, LittleEndian};
use cgmath::Vector3;
use color;
use errors::*;
use std::fs::File;
use std::io::{BufRead, BufReader, Seek, SeekFrom};
use std::ops::Index;
use std::path::Path;
use std::str;

#[derive(Debug)]
struct Header {
    format: Format,
    elements: Vec<Element>,
}

#[derive(Debug, Copy, Clone, PartialEq)]
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

#[derive(Debug, PartialEq)]
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
                    count: entries[2]
                        .parse::<i64>()
                        .chain_err(|| InvalidInput(format!("Invalid count: {}", entries[2])))?,
                    properties: Vec::new(),
                });
            }
            "property" => {
                if current_element.is_none() {
                    return Err(
                        InvalidInput(format!("property outside of element: {}", line)).into(),
                    );
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

    Ok((
        Header {
            elements: elements,
            format: format.unwrap(),
        },
        header_len,
    ))
}

type ReadingFn = fn(nread: &mut usize, buf: &[u8], val: &mut Point);

// The two macros create a 'ReadingFn' that reads a value of '$data_type' out of a reader, assigns
// it to '$property' (e.g. 'position.x') of 'point' while casting it to the correct type. I did not
// find a way of doing this purely using generic programming, so I resorted to this macro.
macro_rules! create_and_return_reading_fn {
    ($($property:ident).+, $size:ident, $num_bytes:expr, $reading_fn:expr) => (
        {
            $size += $num_bytes;
            fn _read_fn(nread: &mut usize, buf: &[u8], point: &mut Point) {
                point $( .$property )+ = $reading_fn(buf) as _;
                *nread += $num_bytes;
            }
            _read_fn
        }
    )
}

macro_rules! read_casted_property {
    ($data_type:expr, point. $($property:ident).+, &mut $size:ident) => (
        match $data_type {
            DataType::Uint8 => {
                create_and_return_reading_fn!($($property).+, $size, 1,
                    |buf: &[u8]| buf[0])
            },
            DataType::Int8 => {
                create_and_return_reading_fn!($($property).+, $size, 1,
                    |buf: &[u8]| buf[0])
            },
            DataType::Uint16 => {
                create_and_return_reading_fn!($($property).+, $size, 2,
                    LittleEndian::read_u16)
            },
            DataType::Int16 => {
                create_and_return_reading_fn!($($property).+, $size, 2,
                    LittleEndian::read_i16)
            },
            DataType::Uint32 => {
                create_and_return_reading_fn!($($property).+, $size, 4,
                    LittleEndian::read_u32)
            },
            DataType::Int32 => {
                create_and_return_reading_fn!($($property).+, $size, 4,
                    LittleEndian::read_i32)
            },
            DataType::Float32 => {
                create_and_return_reading_fn!($($property).+, $size, 4,
                    LittleEndian::read_f32)
            },
            DataType::Float64 => {
                create_and_return_reading_fn!($($property).+, $size, 8,
                    LittleEndian::read_f64)
            },
        }
    )
}

// Similar to 'create_and_return_reading_fn', but creates a function that just advances the read
// pointer.
macro_rules! create_skip_fn {
    (&mut $size:ident, $num_bytes:expr) => (
        {
            $size += $num_bytes;
            fn _read_fn(nread: &mut usize, _: &[u8], _: &mut Point) {
                *nread += $num_bytes;
            }
            _read_fn
        }
    )
}

/// Opens a PLY file and checks that it is the correct format we support. Seeks in the file to the
/// beginning of the binary data which must be (x, y, z, r, g, b) tuples.
fn open(ply_file: &Path) -> Result<(BufReader<File>, i64, Vec<ReadingFn>)> {
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
    let mut seen_x = false;
    let mut seen_y = false;
    let mut seen_z = false;

    let mut readers: Vec<ReadingFn> = Vec::new();
    let mut num_bytes_per_point = 0;

    for prop in &vertex.properties {
        match &prop.name as &str {
            "x" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.position.x,
                    &mut num_bytes_per_point
                ));
                seen_x = true;
            }
            "y" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.position.y,
                    &mut num_bytes_per_point
                ));
                seen_y = true;
            }
            "z" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.position.z,
                    &mut num_bytes_per_point
                ));
                seen_z = true;
            }
            "r" | "red" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.color.red,
                    &mut num_bytes_per_point
                ));
            }
            "g" | "green" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.color.green,
                    &mut num_bytes_per_point
                ));
            }
            "b" | "blue" => {
                readers.push(read_casted_property!(
                    prop.data_type,
                    point.color.blue,
                    &mut num_bytes_per_point
                ));
            }
            other => {
                println!("Will ignore property '{}' on 'vertex'.", other);
                use self::DataType::*;
                match prop.data_type {
                    Uint8 | Int8 => readers.push(create_skip_fn!(&mut num_bytes_per_point, 1)),
                    Uint16 | Int16 => readers.push(create_skip_fn!(&mut num_bytes_per_point, 2)),
                    Uint32 | Int32 | Float32 => {
                        readers.push(create_skip_fn!(&mut num_bytes_per_point, 4))
                    }
                    Float64 => readers.push(create_skip_fn!(&mut num_bytes_per_point, 8)),
                }
            }
        }
    }

    if !seen_x || !seen_y || !seen_z {
        panic!("PLY must contain properties 'x', 'y', 'z' for 'vertex'.");
    }

    // We align the buffer of this 'BufReader' to points, so that we can index this buffer and know
    // that it will always contain full points to parse.
    Ok((
        BufReader::with_capacity(num_bytes_per_point * 1024, file),
        header["vertex"].count,
        readers,
    ))
}

/// Abstraction to read binary points from ply files into points.
pub struct PlyIterator {
    reader: BufReader<File>,
    readers: Vec<ReadingFn>,
    pub num_total_points: i64,
}

impl PlyIterator {
    pub fn new<P: AsRef<Path>>(ply_file: P) -> Result<Self> {
        let (reader, num_total_points, readers) = open(ply_file.as_ref())?;
        Ok(PlyIterator {
            reader: reader,
            readers: readers,
            num_total_points: num_total_points,
        })
    }
}

impl InternalIterator for PlyIterator {
    fn size_hint(&self) -> Option<usize> {
        Some(self.num_total_points as usize)
    }

    fn for_each<F: FnMut(&Point)>(mut self, mut func: F) {
        let mut point = Point {
            position: Vector3::new(0., 0., 0.),
            color: color::WHITE.to_u8(),
        };

        for _ in 0..self.num_total_points {
            let mut nread = 0;

            // We made sure before that the internal buffer of 'reader' is aligned to the number of
            // bytes for a single point, therefore we can access it here and know that we can always
            // read into it and are sure that it contains at least a full point.
            {
                let buf = self.reader.fill_buf().unwrap();
                for r in &self.readers {
                    let cnread = nread;
                    r(&mut nread, &buf[cnread..], &mut point);
                }
            }

            func(&point);
            self.reader.consume(nread);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn points_from_file<P: AsRef<Path>>(path: P) -> Vec<Point> {
        let iterator = PlyIterator::new(path).unwrap();
        let mut points = Vec::new();
        iterator.for_each(|p| {
            points.push(p.clone());
        });
        points
    }

    #[test]
    fn test_xyz_f32_rgb_u8_le() {
        let points = points_from_file("src/test_data/xyz_f32_rgb_u8_le.ply");
        assert_eq!(8, points.len());
        assert_eq!(points[0].position.x, 1.);
        assert_eq!(points[7].position.x, 22.);
        assert_eq!(points[0].color.red, 255);
        assert_eq!(points[7].color.red, 234);
    }

    #[test]
    fn test_xyz_f32_rgba_u8_le() {
        let points = points_from_file("src/test_data/xyz_f32_rgba_u8_le.ply");
        assert_eq!(8, points.len());
        assert_eq!(points[0].position.x, 1.);
        assert_eq!(points[7].position.x, 22.);
        assert_eq!(points[0].color.red, 255);
        assert_eq!(points[7].color.red, 227);
    }
}
