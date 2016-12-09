// Copyright 2016 The Cartographer Authors
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

use byteorder::{LittleEndian, WriteBytesExt, ReadBytesExt};
use errors::*;
use math::{Cube, CuboidLike, Vector3f, Zero, clamp};
use num;
use num_traits;
use Point;
use proto;
use protobuf::{self, Message};
use std::fmt;
use std::fs::{self, File};
use std::io::BufReader;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::result;

const META_EXT: &'static str = ".pb";
const POSITION_EXT: &'static str = "xyz";
const COLOR_EXT: &'static str = "rgb";

/// Represents a child of an octree Node.
#[derive(Debug,PartialEq,Eq)]
pub struct ChildIndex(u8);

impl ChildIndex {
    pub fn from_u8(index: u8) -> Self {
        assert!(index < 8);
        ChildIndex(index)
    }

    pub fn as_u8(&self) -> u8 {
        self.0
    }
}

/// A unique identifier to a node. Currently this is implemented as 'r' being the root and r[0-7]
/// being the children, r[0-7][0-7] being the grand children and so on. The actual representation
/// might change though.
#[derive(Debug,Hash,Clone,PartialEq,Eq)]
pub struct NodeId {
    name: String,
}

impl fmt::Display for NodeId {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> result::Result<(), fmt::Error> {
        self.name.fmt(formatter)
    }
}

impl NodeId {
    /// Construct a NodeId. No checking is done if this is a valid Id.
    pub fn from_string(name: String) -> Self {
        NodeId { name: name }
    }

    /// Returns the path on disk where the data for this node is saved.
    fn get_stem(&self, directory: &Path) -> PathBuf {
        directory.join(&self.name)
    }

    /// Returns the root node of the octree.
    fn root() -> Self {
        NodeId::from_string("r".to_string())
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    fn get_child_id(&self, child_index: ChildIndex) -> Self {
        let mut child_name = self.name.clone();
        child_name.push((child_index.0 + '0' as u8) as char);
        NodeId::from_string(child_name)
    }

    /// The child index of this node in its parent.
    fn child_index(&self) -> Option<ChildIndex> {
        if self.level() == 0 {
            return None;
        }
        match *self.name.as_bytes().last().unwrap() as char {
            '0' => Some(ChildIndex(0)),
            '1' => Some(ChildIndex(1)),
            '2' => Some(ChildIndex(2)),
            '3' => Some(ChildIndex(3)),
            '4' => Some(ChildIndex(4)),
            '5' => Some(ChildIndex(5)),
            '6' => Some(ChildIndex(6)),
            '7' => Some(ChildIndex(7)),
            _ => panic!("Invalid node name: {}", self.name),
        }
    }

    /// Returns the parents id or None if this is the root.
    fn parent_id(&self) -> Option<NodeId> {
        if self.level() == 0 {
            return None;
        }
        let parent_name = self.name.split_at(self.name.len() - 1).0.to_string();
        Some(NodeId::from_string(parent_name))
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    fn level(&self) -> usize {
        self.name.len() - 1
    }
}

#[derive(Debug)]
pub struct Node {
    pub id: NodeId,
    pub bounding_cube: Cube,
}

impl Node {
    pub fn root_with_bounding_cube(cube: Cube) -> Self {
        Node {
            id: NodeId::root(),
            bounding_cube: cube,
        }
    }

    pub fn get_child(&self, child_index: ChildIndex) -> Node {
        let child_bounding_cube = {
            let half_edge_length = self.bounding_cube.edge_length() / 2.;
            let mut min = self.bounding_cube.min();
            if (child_index.0 & 0b001) != 0 {
                min.z += half_edge_length;
            }

            if (child_index.0 & 0b010) != 0 {
                min.y += half_edge_length;
            }

            if (child_index.0 & 0b100) != 0 {
                min.x += half_edge_length;
            }
            Cube::new(min, half_edge_length)
        };
        Node {
            id: self.id.get_child_id(child_index),
            bounding_cube: child_bounding_cube,
        }
    }

    /// Returns the ChildId of the child containing 'v'.
    pub fn get_child_id_containing_point(&self, v: &Vector3f) -> ChildIndex {
        // This is a bit flawed: it is not guaranteed that 'child_bounding_box.contains(&v)' is true
        // using this calculated index due to floating point precision.
        let center = self.bounding_cube.center();
        let gt_x = v.x > center.x;
        let gt_y = v.y > center.y;
        let gt_z = v.z > center.z;
        ChildIndex((gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8)
    }

    // TODO(hrapp): This function could use some testing.
    pub fn parent(&self) -> Option<Node> {
        let maybe_parent_id = self.id.parent_id();
        if maybe_parent_id.is_none() {
            return None;
        }

        let parent_cube = {
            let child_index = self.id.child_index().unwrap().0;
            let mut min = self.bounding_cube.min();
            let edge_length = self.bounding_cube.edge_length();
            if (child_index & 0b001) != 0 {
                min.z -= edge_length;
            }

            if (child_index & 0b010) != 0 {
                min.y -= edge_length;
            }

            if (child_index & 0b100) != 0 {
                min.x -= edge_length;
            }
            Cube::new(min, edge_length * 2.)
        };
        Some(Node {
            id: maybe_parent_id.unwrap(),
            bounding_cube: parent_cube,
        })
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    pub fn level(&self) -> usize {
        self.id.level()
    }
}

pub struct NodeIterator {
    xyz_reader: BufReader<File>,
    rgb_reader: BufReader<File>,
    num_points_read: i64,
    num_total_points: i64,
    position_encoding: PositionEncoding,
    bounding_cube: Cube,
}

impl NodeIterator {
    pub fn from_disk(directory: &Path, id: &NodeId) -> Result<Self> {
        let stem = id.get_stem(directory);
        let xyz_path = stem.with_extension(META_EXT);
        if !xyz_path.exists() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let meta = {
            let mut reader = File::open(&stem.with_extension(META_EXT))?;
            protobuf::parse_from_reader::<proto::Node>(&mut reader).chain_err(|| "Could not parse node protobuf.")?
        };

        Ok(NodeIterator {
            xyz_reader: BufReader::new(File::open(&stem.with_extension(POSITION_EXT))?),
            rgb_reader: BufReader::new(File::open(stem.with_extension(COLOR_EXT))?),
            num_points_read: 0,
            num_total_points: meta.get_num_points(),
            position_encoding: PositionEncoding::from_proto(&meta.get_position_encoding()),
            // TODO(hrapp): Would be nice to have a from_proto and to_proto as a trait.
            bounding_cube: {
                let proto = meta.get_bounding_cube();
                let min = proto.get_min();
                Cube::new(Vector3f::new(min.get_x(), min.get_y(), min.get_z()),
                          proto.get_edge_length())
            },
        })
    }
}

impl Iterator for NodeIterator {
    type Item = Point;

    fn next(&mut self) -> Option<Self::Item> {
        if self.num_points_read >= self.num_total_points {
            return None;
        }

        let mut point = Point {
            position: Vector3f::zero(),
            r: 0,
            g: 0,
            b: 0,
        };

        match self.position_encoding {
            PositionEncoding::Float32 => {
                point.position.x = self.xyz_reader.read_f32::<LittleEndian>().unwrap();
                point.position.y = self.xyz_reader.read_f32::<LittleEndian>().unwrap();
                point.position.z = self.xyz_reader.read_f32::<LittleEndian>().unwrap();
            }
            PositionEncoding::Uint8 => {
                let edge_length = self.bounding_cube.edge_length();
                let min = self.bounding_cube.min();
                point.position.x =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.x, edge_length);
                point.position.y =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.y, edge_length);
                point.position.z =
                    fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.z, edge_length);
            }
            PositionEncoding::Uint16 => {
                let edge_length = self.bounding_cube.edge_length();
                let min = self.bounding_cube.min();
                point.position.x =
                    fixpoint_decode(self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                                    min.x,
                                    edge_length);
                point.position.y =
                    fixpoint_decode(self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                                    min.y,
                                    edge_length);
                point.position.z =
                    fixpoint_decode(self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                                    min.z,
                                    edge_length);
            }

        }

        point.r = self.rgb_reader.read_u8().unwrap();
        point.g = self.rgb_reader.read_u8().unwrap();
        point.b = self.rgb_reader.read_u8().unwrap();
        self.num_points_read += 1;
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.num_total_points as usize, Some(self.num_total_points as usize))
    }
}

#[derive(Debug)]
pub enum PositionEncoding {
    Uint8,
    Uint16,
    Float32,
}

impl PositionEncoding {
    fn new(bounding_cube: &Cube, resolution: f64) -> PositionEncoding {
        let min_bits = (bounding_cube.edge_length() as f64 / resolution).log2() as u32 + 1;
        match min_bits {
            0...8 => PositionEncoding::Uint8,
            9...16 => PositionEncoding::Uint16,
            _ => PositionEncoding::Float32,
        }
    }

    fn from_proto(proto: &proto::Node_PositionEncoding) -> Self {
        match *proto {
            proto::Node_PositionEncoding::Uint8 => PositionEncoding::Uint8,
            proto::Node_PositionEncoding::Uint16 => PositionEncoding::Uint16,
            proto::Node_PositionEncoding::Float32 => PositionEncoding::Float32,
        }
    }

    fn to_proto(&self) -> proto::Node_PositionEncoding {
        match *self {
            PositionEncoding::Uint8 => proto::Node_PositionEncoding::Uint8,
            PositionEncoding::Uint16 => proto::Node_PositionEncoding::Uint16,
            PositionEncoding::Float32 => proto::Node_PositionEncoding::Float32,
        }
    }
}

fn fixpoint_encode<T>(value: f32, min: f32, edge_length: f32) -> T
    where T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast
{
    let value = clamp((value - min) / edge_length, 0., 1.) *
                num::cast::<T, f32>(T::max_value()).unwrap();
    num::cast(value).unwrap()
}

fn fixpoint_decode<T>(value: T, min: f32, edge_length: f32) -> f32
    where T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast
{
    let max: f32 = num::cast(T::max_value()).unwrap();
    let v: f32 = num::cast(value).unwrap();
    v / max * edge_length + min
}

#[derive(Debug)]
pub struct NodeWriter {
    xyz_writer: BufWriter<File>,
    rgb_writer: BufWriter<File>,
    bounding_cube: Cube,
    position_encoding: PositionEncoding,
    stem: PathBuf,
    num_written: i64,
}

impl Drop for NodeWriter {
    fn drop(&mut self) {
        // If we did not write anything into this node, it should not exist.
        if self.num_written == 0 {
            self.remove_all_files();
        } else {
            let mut proto = proto::Node::new();
            proto.mut_bounding_cube().mut_min().set_x(self.bounding_cube.min().x);
            proto.mut_bounding_cube().mut_min().set_y(self.bounding_cube.min().y);
            proto.mut_bounding_cube().mut_min().set_z(self.bounding_cube.min().z);
            proto.mut_bounding_cube().set_edge_length(self.bounding_cube.edge_length());
            proto.set_position_encoding(self.position_encoding.to_proto());
            proto.set_num_points(self.num_written);
            let mut file = File::create(&self.stem.with_extension(META_EXT)).unwrap();
            proto.write_to_writer(&mut file).unwrap();
        }

        // TODO(hrapp): Add some sanity checks that we do not have nodes with ridiculously low
        // amount of points laying around?
    }
}

impl NodeWriter {
    pub fn new(output_directory: &Path, node: &Node, resolution: f64) -> Self {

        let stem = node.id.get_stem(output_directory);
        NodeWriter {
            xyz_writer: BufWriter::new(File::create(&stem.with_extension(POSITION_EXT)).unwrap()),
            rgb_writer: BufWriter::new(File::create(&stem.with_extension(COLOR_EXT)).unwrap()),
            stem: stem,
            position_encoding: PositionEncoding::new(&node.bounding_cube, resolution),
            bounding_cube: node.bounding_cube.clone(),
            num_written: 0,
        }
    }

    pub fn write(&mut self, p: &Point) {
        // Note that due to floating point rounding errors while calculating bounding boxes, it
        // could be here that 'p' is not quite inside the bounding box of our node.
        match self.position_encoding {
            PositionEncoding::Float32 => {
                self.xyz_writer.write_f32::<LittleEndian>(p.position.x).unwrap();
                self.xyz_writer.write_f32::<LittleEndian>(p.position.y).unwrap();
                self.xyz_writer.write_f32::<LittleEndian>(p.position.z).unwrap();
            }
            PositionEncoding::Uint8 => {
                let edge_length = self.bounding_cube.edge_length();
                let min = self.bounding_cube.min();
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
                let edge_length = self.bounding_cube.edge_length();
                let min = self.bounding_cube.min();
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
        }

        self.rgb_writer.write_u8(p.r).unwrap();
        self.rgb_writer.write_u8(p.g).unwrap();
        self.rgb_writer.write_u8(p.b).unwrap();
        self.num_written += 1;
    }

    pub fn num_written(&self) -> i64 {
        self.num_written
    }

    fn remove_all_files(&self) {
        // We are ignoring deletion errors here in case the file is already gone.
        let _ = fs::remove_file(&self.stem.with_extension(POSITION_EXT));
        let _ = fs::remove_file(&self.stem.with_extension(COLOR_EXT));
        let _ = fs::remove_file(&self.stem.with_extension(META_EXT));
    }
}


#[cfg(test)]
mod tests {
    use super::{ChildIndex, NodeId};

    #[test]
    fn test_parent_node_name() {
        assert_eq!(Some(NodeId::from_string("r12345".into())),
                   NodeId::from_string("r123456".into()).parent_id());
    }

    #[test]
    fn test_child_index() {
        assert_eq!(Some(ChildIndex(1)),
                   NodeId::from_string("r123451".into()).child_index());
        assert_eq!(Some(ChildIndex(7)),
                   NodeId::from_string("r123457".into()).child_index());
        assert_eq!(None, NodeId::from_string("r".into()).child_index());
    }
}
