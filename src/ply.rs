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

use ::byteorder::{LittleEndian, ByteOrder};
use ::math::Vector3f;
use ::nom;
use ::Point;
use ::std::fs::File;
use std::io::{SeekFrom, Read, Seek};
use ::std::path::Path;
use ::std::str;

// TODO(hrapp): nom is a PITA when working with streams instead of byte arrays. Maybe get rid of it
// again?

#[derive(Debug)]
struct Header {
    pub format: Format,
    pub vertex: Element,
}

#[derive(Debug,PartialEq)]
enum Format {
    BinaryLittleEndianV1,
    BinaryBigEndianV1,
    AsciiV1,
}

// TODO(hrapp): there are more, support them.
#[derive(Debug)]
enum DataType {
    U8,
    F8,
}

#[derive(Debug)]
struct Property {
    name: String,
    data_type: DataType,
}

#[derive(Debug)]
struct Element {
    name: String,
    count: i64,
    properties: Vec<Property>,
}

named!(comment<String>,
    chain!(
        tag!("comment ") ~
        c: is_not!("\n") ~
        tag!("\n"),
        || {
             String::from_utf8(c.into()).unwrap()
        }
    )
);

named!(format<Format>,
    chain!(
        tag!("format ") ~
        format: alt!(
            value!(Format::BinaryLittleEndianV1, tag!("binary_little_endian 1.0")) |
            value!(Format::BinaryBigEndianV1, tag!("binary_big_endian 1.0")) |
            value!(Format::AsciiV1, tag!("ascii 1.0"))
        ) ~
        tag!("\n"),
        || format
    )
);

named!(property<Property>,
    chain!(
        tag!("property ") ~
        data_type: alt!(
            value!(DataType::U8, tag!("uchar")) |
            value!(DataType::F8, tag!("float"))
        ) ~
        tag!(" ") ~
        name: is_not!("\n") ~
        tag!("\n"),
        || Property {
                name: String::from_utf8(name.into()).unwrap(),
                data_type: data_type
        }
    )
);

named!(element<Element>,
    chain!(
        tag!("element ") ~
        name: is_not!(" ") ~
        tag!(" ") ~
        count: take_while!(nom::is_digit) ~
        tag!("\n") ~
        properties: many1!(property),
        || Element {
            name: String::from_utf8(name.into()).unwrap(),
            count: str::from_utf8(count).unwrap().parse().unwrap(),
            properties: properties,
        }
    )
);

named!(point<Point>,
       chain!(
           x: map!(take!(4), LittleEndian::read_f32) ~
           y: map!(take!(4), LittleEndian::read_f32) ~
           z: map!(take!(4), LittleEndian::read_f32) ~
           r: map!(take!(1), |c: &[u8]| c[0]) ~
           g: map!(take!(1), |c: &[u8]| c[0]) ~
           b: map!(take!(1), |c: &[u8]| c[0]),
           || Point {
               position: Vector3f::new(x, y, z),
               r: r,
               g: g,
               b: b,
           }
       )
);

named!(header<Header>,
    chain!(
       tag!("ply\n") ~
// TODO(hrapp): These can come in any order. Did not figure out how to do that with NOM.
       format: format ~
       many0!(comment) ~
// TODO(hrapp): There can be multiple elements and the name should be free formed.
       vertex: element ~
       tag!("end_header\n"),
       || {
           Header{
               format: format,
               vertex: vertex,
           }
       })
);

/// Opens a PLY file and checks that it is the correct format we support. Seeks in the file to the
/// beginning of the binary data which must be (x, y, z, r, g, b) tuples.
// TODO(hrapp): support more of PLY and maybe pull this out into a separate crate.
pub fn open(ply_file: &Path) -> (File, i64) {
    // hopefully, the header is shorter.
    let mut header_bytes = vec![0u8; 4096];
    let mut file = File::open(ply_file).unwrap();
    file.read_exact(&mut header_bytes).unwrap();
    let header = {
        let (remaining_data, header) = match header(&header_bytes) {
            nom::IResult::Done(r, h) => (r, h),
            nom::IResult::Error(err) => panic!("Parsing error: {}", err),
            nom::IResult::Incomplete(_) => panic!("Parsing error: Unexpected end of data."),
        };

        if header.format != Format::BinaryLittleEndianV1 {
            panic!("Unsupported PLY format: {:?}", header.format);
        }

        if header.vertex.name != "vertex" && header.vertex.properties.len() != 6 &&
           header.vertex.properties[0].name != "x" &&
           header.vertex.properties[1].name != "y" &&
           header.vertex.properties[2].name != "z" &&
           header.vertex.properties[3].name != "red" &&
           header.vertex.properties[4].name != "green" &&
           header.vertex.properties[4].name != "blue" {
            panic!("PLY must contain (x,y,z,red,green,blue) tuples.");
        }
        let offset = header_bytes.len() - remaining_data.len();
        file.seek(SeekFrom::Start(offset as u64)).unwrap();
        header
    };
    (file, header.vertex.count)
}

/// Read a single point (x, y, z, r, g, b) tuple out of 'data'.
pub fn read_point<T: Read>(data: &mut T) -> Point {
    let mut bytes = [0u8; 15];
    data.read_exact(&mut bytes).unwrap();
    match point(&bytes) {
        nom::IResult::Done(_, h) => h,
        nom::IResult::Error(err) => panic!("Parsing error: {}", err),
        nom::IResult::Incomplete(_) => panic!("Parsing error: Unexpected end of data."),
    }
}
