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
    DataWriter, Encoding, NodeWriter, OpenMode, PositionEncoding, WriteEncoded, WriteLE, WriteLEPos,
};
use crate::{AttributeData, Point, PointsBatch};
use byteorder::{ByteOrder, LittleEndian};
use cgmath::Vector3;
use num_traits::identities::Zero;
use std::fs::File;
use std::io::{self, BufRead, BufReader, Read, Seek, SeekFrom, Write};
use std::ops::Index;
use std::path::{Path, PathBuf};
use std::str::{from_utf8, FromStr};

const HEADER_START_TO_NUM_VERTICES: &[u8] =
    b"ply\nformat binary_little_endian 1.0\nelement vertex ";
const HEADER_NUM_VERTICES: &[u8] = b"00000000000000000000";

#[derive(Debug)]
struct Header {
    format: Format,
    elements: Vec<Element>,
    offset: Vector3<f64>,
}

#[derive(Debug, Copy, Clone, PartialEq)]
enum DataType {
    Int8,
    Uint8,
    Int16,
    Uint16,
    Int32,
    Uint32,
    Int64,  // Not a legal datatype
    Uint64, // Not a legal datatype
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
            "longlong" | "int64" => Ok(DataType::Int64),
            "ulonglong" | "uint64" => Ok(DataType::Uint64),
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
    use crate::errors::ErrorKind::InvalidInput;

    let mut header_len = 0;
    let mut line = String::new();
    header_len += reader.read_line(&mut line)?;
    if line.trim() != "ply" {
        return Err(InvalidInput("Not a PLY file".to_string()).into());
    }

    let mut format = None;
    let mut current_element = None;
    let mut offset = Vector3::zero();
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
                            data_type,
                        }
                    }
                    _ => return Err(InvalidInput(format!("Invalid line: {}", line)).into()),
                };
                current_element.as_mut().unwrap().properties.push(property);
            }
            "end_header" => break,
            "comment" => {
                if entries.len() == 5 && entries[1] == "offset:" {
                    let x = entries[2]
                        .parse::<f64>()
                        .chain_err(|| InvalidInput(format!("Invalid offset: {}", entries[2])))?;
                    let y = entries[3]
                        .parse::<f64>()
                        .chain_err(|| InvalidInput(format!("Invalid offset: {}", entries[3])))?;
                    let z = entries[4]
                        .parse::<f64>()
                        .chain_err(|| InvalidInput(format!("Invalid offset: {}", entries[4])))?;
                    offset = Vector3::new(x, y, z)
                }
            }
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
            elements,
            format: format.unwrap(),
            offset,
        },
        header_len,
    ))
}

type ReadingFn = fn(nread: &mut usize, buf: &[u8], val: &mut Point);

// The two macros create a 'ReadingFn' that reads a value of '$data_type' out of a reader, and
// calls '$assign' with it while casting it to the correct type. I did not find a way of doing this
// purely using generic programming, so I resorted to this macro.
macro_rules! create_and_return_reading_fn {
    ($assign:expr, $size:ident, $num_bytes:expr, $reading_fn:expr) => {{
        $size += $num_bytes;
        |nread: &mut usize, buf: &[u8], point: &mut Point| {
            #[allow(clippy::cast_lossless)]
            $assign(point, $reading_fn(buf) as _);
            *nread += $num_bytes;
        }
    }};
}

macro_rules! read_casted_property {
    ($data_type:expr, $assign:expr, &mut $size:ident) => {
        match $data_type {
            DataType::Uint8 => {
                create_and_return_reading_fn!($assign, $size, 1, |buf: &[u8]| buf[0])
            }
            DataType::Int8 => create_and_return_reading_fn!($assign, $size, 1, |buf: &[u8]| buf[0]),
            DataType::Uint16 => {
                create_and_return_reading_fn!($assign, $size, 2, LittleEndian::read_u16)
            }
            DataType::Int16 => {
                create_and_return_reading_fn!($assign, $size, 2, LittleEndian::read_i16)
            }
            DataType::Uint32 => {
                create_and_return_reading_fn!($assign, $size, 4, LittleEndian::read_u32)
            }
            DataType::Int32 => {
                create_and_return_reading_fn!($assign, $size, 4, LittleEndian::read_i32)
            }
            DataType::Uint64 => {
                create_and_return_reading_fn!($assign, $size, 4, LittleEndian::read_u64)
            }
            DataType::Int64 => {
                create_and_return_reading_fn!($assign, $size, 4, LittleEndian::read_i64)
            }
            DataType::Float32 => {
                create_and_return_reading_fn!($assign, $size, 4, LittleEndian::read_f32)
            }
            DataType::Float64 => {
                create_and_return_reading_fn!($assign, $size, 8, LittleEndian::read_f64)
            }
        }
    };
}

// Similar to 'create_and_return_reading_fn', but creates a function that just advances the read
// pointer.
macro_rules! create_skip_fn {
    (&mut $size:ident, $num_bytes:expr) => {{
        $size += $num_bytes;
        fn _read_fn(nread: &mut usize, _: &[u8], _: &mut Point) {
            *nread += $num_bytes;
        }
        _read_fn
    }};
}

/// Abstraction to read binary points from ply files into points.
pub struct PlyIterator {
    reader: BufReader<File>,
    readers: Vec<ReadingFn>,
    pub num_total_points: i64,
    offset: Vector3<f64>,
    point_count: usize,
}

impl PlyIterator {
    pub fn from_file<P: AsRef<Path>>(ply_file: P) -> Result<Self> {
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
                        |p: &mut Point, val: f64| p.position.x = val,
                        &mut num_bytes_per_point
                    ));
                    seen_x = true;
                }
                "y" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: f64| p.position.y = val,
                        &mut num_bytes_per_point
                    ));
                    seen_y = true;
                }
                "z" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: f64| p.position.z = val,
                        &mut num_bytes_per_point
                    ));
                    seen_z = true;
                }
                "r" | "red" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: u8| p.color.red = val,
                        &mut num_bytes_per_point
                    ));
                }
                "g" | "green" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: u8| p.color.green = val,
                        &mut num_bytes_per_point
                    ));
                }
                "b" | "blue" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: u8| p.color.blue = val,
                        &mut num_bytes_per_point
                    ));
                }
                "a" | "alpha" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val: u8| p.color.alpha = val,
                        &mut num_bytes_per_point
                    ));
                }
                "intensity" => {
                    readers.push(read_casted_property!(
                        prop.data_type,
                        |p: &mut Point, val| p.intensity = Some(val),
                        &mut num_bytes_per_point
                    ));
                }
                other => {
                    println!("Will ignore property '{}' on 'vertex'.", other);
                    use self::DataType::*;
                    match prop.data_type {
                        Uint8 | Int8 => readers.push(create_skip_fn!(&mut num_bytes_per_point, 1)),
                        Uint16 | Int16 => {
                            readers.push(create_skip_fn!(&mut num_bytes_per_point, 2))
                        }
                        Uint32 | Int32 | Float32 => {
                            readers.push(create_skip_fn!(&mut num_bytes_per_point, 4))
                        }
                        Float64 | Uint64 | Int64 => {
                            readers.push(create_skip_fn!(&mut num_bytes_per_point, 8))
                        }
                    }
                }
            }
        }

        if !seen_x || !seen_y || !seen_z {
            panic!("PLY must contain properties 'x', 'y', 'z' for 'vertex'.");
        }

        // We align the buffer of this 'BufReader' to points, so that we can index this buffer and know
        // that it will always contain full points to parse.
        Ok(PlyIterator {
            reader: BufReader::with_capacity(num_bytes_per_point * 1024, file),
            readers,
            num_total_points: header["vertex"].count,
            offset: header.offset,
            point_count: 0,
        })
    }
}

impl Iterator for PlyIterator {
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let size = self.num_total_points as usize;
        (size, Some(size))
    }

    fn next(&mut self) -> Option<Point> {
        if self.point_count == self.num_total_points as usize {
            return None;
        }

        let mut point = Point {
            position: Vector3::zero(),
            color: color::WHITE.to_u8(),
            intensity: None,
        };

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
        point.position += self.offset;
        self.reader.consume(nread);
        self.point_count += 1;

        Some(point)
    }
}

pub struct PlyNodeWriter {
    writer: DataWriter,
    point_count: usize,
    encoding: Encoding,
}

impl NodeWriter<PointsBatch> for PlyNodeWriter {
    fn new(filename: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        Self::new(filename, encoding, open_mode)
    }

    fn write(&mut self, p: &PointsBatch) -> io::Result<()> {
        if self.point_count == 0 {
            self.create_header(
                &p.attributes
                    .iter()
                    .map(|(k, data)| {
                        (
                            &k[..],
                            match data {
                                AttributeData::U8(_) => "uchar",
                                AttributeData::I64(_) => "longlong",
                                AttributeData::U64(_) => "ulonglong",
                                AttributeData::F32(_) => "float",
                                AttributeData::F64(_) => "double",
                                AttributeData::U8Vec3(_) => "uchar",
                                AttributeData::F64Vec3(_) => "double",
                            },
                            data.dim(),
                        )
                    })
                    .collect::<Vec<_>>()[..],
            )?;
        }

        for (i, pos) in p.position.iter().enumerate() {
            pos.write_encoded(&self.encoding, &mut self.writer)?;
            for data in p.attributes.values() {
                data.write_le_pos(i, &mut self.writer)?;
            }
        }

        self.point_count += p.position.len();

        Ok(())
    }
}

impl NodeWriter<Point> for PlyNodeWriter {
    fn new(filename: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        Self::new(filename, encoding, open_mode)
    }

    fn write(&mut self, p: &Point) -> io::Result<()> {
        if self.point_count == 0 {
            let mut attributes = vec![("color", "uchar", 3)];
            if p.intensity.is_some() {
                attributes.push(("intensity", "float", 1));
            }
            self.create_header(&attributes)?;
        }

        p.position.write_encoded(&self.encoding, &mut self.writer)?;
        p.color.write_le(&mut self.writer)?;
        if let Some(i) = p.intensity {
            i.write_le(&mut self.writer)?;
        }

        self.point_count += 1;

        Ok(())
    }
}

impl Drop for PlyNodeWriter {
    fn drop(&mut self) {
        if self.point_count == 0 {
            return;
        }
        self.writer.write_all(b"\n").unwrap();
        if self
            .writer
            .seek(SeekFrom::Start(HEADER_START_TO_NUM_VERTICES.len() as u64))
            .is_ok()
        {
            let _res = write!(
                &mut self.writer,
                "{:0width$}",
                self.point_count,
                width = HEADER_NUM_VERTICES.len()
            );
        }
    }
}

impl PlyNodeWriter {
    pub fn new(filename: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        let filename = filename.into();
        let mut point_count = 0;
        if open_mode == OpenMode::Append {
            if let Ok(mut file) = File::open(&filename) {
                if file.metadata().unwrap().len()
                    >= HEADER_START_TO_NUM_VERTICES.len() as u64 + HEADER_NUM_VERTICES.len() as u64
                {
                    file.seek(SeekFrom::Start(HEADER_START_TO_NUM_VERTICES.len() as u64))
                        .unwrap();
                    let mut buf = vec![0; HEADER_NUM_VERTICES.len()];
                    file.read_exact(&mut buf).unwrap();
                    point_count = usize::from_str(from_utf8(&buf).unwrap()).unwrap();
                }
            }
        }
        let mut writer = DataWriter::new(filename, open_mode).unwrap();
        if point_count > 0 {
            // Our ply files always have a newline at the end.
            writer.seek(SeekFrom::End(-1)).unwrap();
        }
        Self {
            writer,
            point_count,
            encoding,
        }
    }

    fn create_header(&mut self, elements: &[(&str, &str, usize)]) -> io::Result<()> {
        self.writer.write_all(HEADER_START_TO_NUM_VERTICES)?;
        self.writer.write_all(HEADER_NUM_VERTICES)?;
        self.writer.write_all(b"\n")?;
        let pos_data_str = match &self.encoding {
            Encoding::Plain => "double",
            Encoding::ScaledToCube(_, _, pos_enc) => match pos_enc {
                PositionEncoding::Uint8 => "uchar",
                PositionEncoding::Uint16 => "ushort",
                PositionEncoding::Float32 => "float",
                PositionEncoding::Float64 => "double",
            },
        };
        for pos in &["x", "y", "z"] {
            let prop = &["property", " ", pos_data_str, " ", pos, "\n"].concat();
            self.writer.write_all(&prop.as_bytes())?;
        }
        for (name, data_str, num_properties) in elements {
            match &name[..] {
                "color" => {
                    let colors = ["red", "green", "blue", "alpha"];
                    for color in colors.iter().take(*num_properties) {
                        let prop = &["property", " ", data_str, " ", color, "\n"].concat();
                        self.writer.write_all(&prop.as_bytes())?;
                    }
                }
                _ if *num_properties > 1 => {
                    for i in 0..*num_properties {
                        let prop =
                            &["property", " ", data_str, " ", name, &i.to_string(), "\n"].concat();
                        self.writer.write_all(&prop.as_bytes())?;
                    }
                }
                _ => {
                    let prop = &["property", " ", data_str, " ", name, "\n"].concat();
                    self.writer.write_all(&prop.as_bytes())?;
                }
            }
        }
        self.writer.write_all(b"end_header\n")
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::read_write::PlyIterator;
    use tempdir::TempDir;

    fn points_from_file<P: AsRef<Path>>(path: P) -> Vec<Point> {
        let iterator = PlyIterator::from_file(path).unwrap();
        let mut points = Vec::new();
        iterator.for_each(|p| {
            points.push(p);
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

    #[test]
    fn test_xyz_f32_rgb_u8_intensity_f32_le() {
        // All intensities in this file are NaN, but set.
        let points = points_from_file("src/test_data/xyz_f32_rgb_u8_intensity_f32.ply");
        assert_eq!(8, points.len());
        assert_eq!(points[0].position.x, 1.);
        assert!(points[0].intensity.is_some());
        assert_eq!(points[7].position.x, 22.);
        assert!(points[7].intensity.is_some());
        assert_eq!(points[0].color.red, 255);
        assert_eq!(points[7].color.red, 234);
    }

    #[test]
    fn test_ply_read_write() {
        let tmp_dir = TempDir::new("test_ply_read_write").unwrap();
        let file_path_test = tmp_dir.path().join("out.ply");
        let file_path_gt = "src/test_data/xyz_f32_rgb_u8_intensity_f32.ply";
        {
            let mut ply_writer =
                PlyNodeWriter::new(&file_path_test, Encoding::Plain, OpenMode::Truncate);
            PlyIterator::from_file(file_path_gt).unwrap().for_each(|p| {
                ply_writer.write(&p).unwrap();
            });
        }
        // Now append to the file
        {
            let mut ply_writer =
                PlyNodeWriter::new(&file_path_test, Encoding::Plain, OpenMode::Append);
            PlyIterator::from_file(file_path_gt).unwrap().for_each(|p| {
                ply_writer.write(&p).unwrap();
            });
        }
        PlyIterator::from_file(file_path_gt)
            .unwrap()
            .chain(PlyIterator::from_file(file_path_gt).unwrap())
            .zip(PlyIterator::from_file(&file_path_test).unwrap())
            .for_each(|(gt, test)| {
                assert_eq!(gt.position, test.position);
                assert_eq!(gt.color, test.color);
                // All intensities in this file are NaN, but set.
                assert_eq!(gt.intensity.is_some(), test.intensity.is_some());
            });
    }
}
