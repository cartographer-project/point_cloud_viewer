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

use {InternalIterator, Point};
use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use errors::*;
use math::{clamp, Cube, CuboidLike, Vector3f, Zero};
use num;
use num_traits;
use proto;
use protobuf::{self, Message};
use std::{fmt, result};
use std::fs::{self, File};
use std::io::{BufReader, BufWriter, Cursor, Read};
use std::path::{Path, PathBuf};

pub const META_EXT: &'static str = "pb";
pub const POSITION_EXT: &'static str = "xyz";
pub const COLOR_EXT: &'static str = "rgb";

/// Represents a child of an octree Node.
#[derive(Debug, PartialEq, Eq)]
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
#[derive(Debug, Hash, Clone, Copy, PartialEq, Eq)]
pub struct NodeId {
    // The root is level = 0, its children 1 and so on.
    level: u8,
    // The index of this node. Multiple nodes can have the same index, but none can have the same
    // index and level.
    index: usize,
}

impl fmt::Display for NodeId {
    fn fmt(&self, formatter: &mut fmt::Formatter) -> result::Result<(), fmt::Error> {
        if self.level == 0 {
            "r".fmt(formatter)
        } else {
            write!(
                formatter,
                "r{index:0width$o}",
                index = self.index,
                width = self.level as usize
            )
        }
    }
}

impl NodeId {
    /// Construct a NodeId. No checking is done if this is a valid Id.
    pub fn from_str(name: &str) -> Self {
        let level = (name.len() - 1) as u8;
        let index = if level > 0 {
            usize::from_str_radix(&name[1..], 8).unwrap()
        } else {
            0
        };
        NodeId { level, index }
    }

    /// Returns the path on disk where the data for this node is saved.
    pub fn get_stem(&self, directory: &Path) -> PathBuf {
        directory.join(&self.to_string())
    }

    /// Returns the root node of the octree.
    fn root() -> Self {
        NodeId { index: 0, level: 0 }
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    fn get_child_id(&self, child_index: ChildIndex) -> Self {
        NodeId {
            level: self.level + 1,
            index: (self.index << 3) + child_index.0 as usize,
        }
    }

    /// The child index of this node in its parent.
    fn child_index(&self) -> Option<ChildIndex> {
        if self.level() == 0 {
            return None;
        }
        Some(ChildIndex(self.index as u8 & 7))
    }

    /// Returns the parents id or None if this is the root.
    fn parent_id(&self) -> Option<NodeId> {
        if self.level() == 0 {
            return None;
        }
        Some(NodeId {
            level: self.level - 1,
            index: (self.index >> 3),
        })
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    fn level(&self) -> usize {
        self.level as usize
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

#[derive(Debug)]
pub struct NodeMeta {
    pub num_points: i64,
    pub position_encoding: PositionEncoding,
    pub bounding_cube: Cube,
}

impl NodeMeta {
    pub fn from_disk(stem: &Path) -> Result<Self> {
        let meta_path = stem.with_extension(META_EXT);
        if !meta_path.exists() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let meta = {
            let mut data = Vec::new();
            File::open(&stem.with_extension(META_EXT))?.read_to_end(&mut data)?;
            protobuf::parse_from_reader::<proto::Node>(&mut Cursor::new(data))
                .chain_err(|| "Could not parse node protobuf.")?
        };

        Ok(NodeMeta {
            num_points: meta.num_points,
            position_encoding: PositionEncoding::from_proto(meta.position_encoding)?,
            // TODO(hrapp): Would be nice to have a from_proto and to_proto as a trait.
            bounding_cube: {
                let proto = meta.bounding_cube.unwrap();
                let min = proto.min.unwrap();
                Cube::new(Vector3f::new(min.x, min.y, min.z), proto.edge_length)
            },
        })
    }

    pub fn num_points_for_level_of_detail(&self, level_of_detail: i32) -> i64 {
        (self.num_points as f32 / level_of_detail as f32).ceil() as i64
    }
}

/// Streams points from our node on-disk representation.
pub struct NodeIterator {
    xyz_reader: BufReader<File>,
    rgb_reader: BufReader<File>,
    meta: NodeMeta,
}

impl NodeIterator {
    pub fn from_disk(directory: &Path, id: &NodeId) -> Result<Self> {
        let stem = id.get_stem(&directory);
        let meta = NodeMeta::from_disk(&stem)?;
        Ok(NodeIterator {
            xyz_reader: BufReader::new(File::open(&stem.with_extension(POSITION_EXT))?),
            rgb_reader: BufReader::new(File::open(&stem.with_extension(COLOR_EXT))?),
            meta: meta,
        })
    }
}

impl InternalIterator for NodeIterator {
    fn size_hint(&self) -> Option<usize> {
        Some(self.meta.num_points as usize)
    }

    fn for_each<F: FnMut(&Point)>(mut self, mut f: F) {
        let mut point = Point {
            position: Vector3f::zero(),
            r: 0,
            g: 0,
            b: 0,
        };

        let edge_length = self.meta.bounding_cube.edge_length();
        let min = self.meta.bounding_cube.min();
        for _ in 0..self.meta.num_points {
            // I tried pulling out this match by taking a function pointer to a 'decode_position'
            // function. This replaces a branch per point vs a function call per point and turned
            // out to be marginally slower.
            match self.meta.position_encoding {
                PositionEncoding::Float32 => {
                    point.position.x = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.x,
                        edge_length,
                    );
                    point.position.y = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.y,
                        edge_length,
                    );
                    point.position.z = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.z,
                        edge_length,
                    );
                }
                PositionEncoding::Uint8 => {
                    point.position.x =
                        fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.x, edge_length);
                    point.position.y =
                        fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.y, edge_length);
                    point.position.z =
                        fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.z, edge_length);
                }
                PositionEncoding::Uint16 => {
                    point.position.x = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.x,
                        edge_length,
                    );
                    point.position.y = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.y,
                        edge_length,
                    );
                    point.position.z = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.z,
                        edge_length,
                    );
                }
            }

            point.r = self.rgb_reader.read_u8().unwrap();
            point.g = self.rgb_reader.read_u8().unwrap();
            point.b = self.rgb_reader.read_u8().unwrap();
            f(&point);
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
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

    // TODO(sirver): Returning a Result here makes this function more expensive than needed - since
    // we require stack space for the full Result. This shuold be fixable to moving to failure.
    pub fn from_proto(proto: proto::Node_PositionEncoding) -> Result<Self> {
        match proto {
            proto::Node_PositionEncoding::INVALID => {
                Err(ErrorKind::InvalidInput("Invalid PositionEncoding".to_string()).into())
            }
            proto::Node_PositionEncoding::Uint8 => Ok(PositionEncoding::Uint8),
            proto::Node_PositionEncoding::Uint16 => Ok(PositionEncoding::Uint16),
            proto::Node_PositionEncoding::Float32 => Ok(PositionEncoding::Float32),
        }
    }

    pub fn to_proto(&self) -> proto::Node_PositionEncoding {
        match *self {
            PositionEncoding::Uint8 => proto::Node_PositionEncoding::Uint8,
            PositionEncoding::Uint16 => proto::Node_PositionEncoding::Uint16,
            PositionEncoding::Float32 => proto::Node_PositionEncoding::Float32,
        }
    }

    pub fn bytes_per_coordinate(&self) -> usize {
        match *self {
            PositionEncoding::Uint8 => 1,
            PositionEncoding::Uint16 => 2,
            PositionEncoding::Float32 => 4,
        }
    }
}

fn fixpoint_encode<T>(value: f32, min: f32, edge_length: f32) -> T
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let value =
        clamp((value - min) / edge_length, 0., 1.) * num::cast::<T, f32>(T::max_value()).unwrap();
    num::cast(value).unwrap()
}

fn fixpoint_decode<T>(value: T, min: f32, edge_length: f32) -> f32
where
    T: num_traits::PrimInt + num_traits::Bounded + num_traits::NumCast,
{
    let max: f32 = num::cast(T::max_value()).unwrap();
    let v: f32 = num::cast(value).unwrap();
    v / max * edge_length + min
}

fn encode(value: f32, min: f32, edge_length: f32) -> f32 {
    clamp((value - min) / edge_length, 0., 1.)
}

fn decode(value: f32, min: f32, edge_length: f32) -> f32 {
    value * edge_length + min
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
            let proto = {
                let mut proto = proto::Node::new();
                proto
                    .mut_bounding_cube()
                    .mut_min()
                    .set_x(self.bounding_cube.min().x);
                proto
                    .mut_bounding_cube()
                    .mut_min()
                    .set_y(self.bounding_cube.min().y);
                proto
                    .mut_bounding_cube()
                    .mut_min()
                    .set_z(self.bounding_cube.min().z);
                proto
                    .mut_bounding_cube()
                    .set_edge_length(self.bounding_cube.edge_length());
                proto.set_position_encoding(self.position_encoding.to_proto());
                proto.set_num_points(self.num_written);
                proto
            };

            let file = File::create(&self.stem.with_extension(META_EXT)).unwrap();
            let mut writer = BufWriter::new(file);
            proto.write_to_writer(&mut writer).unwrap();
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
        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();
        match self.position_encoding {
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
        assert_eq!(
            Some(NodeId::from_str("r12345")),
            NodeId::from_str("r123456").parent_id()
        );
    }

    #[test]
    fn test_child_index() {
        assert_eq!(
            Some(ChildIndex(1)),
            NodeId::from_str("r123451").child_index()
        );
        assert_eq!(
            Some(ChildIndex(7)),
            NodeId::from_str("r123457").child_index()
        );
        assert_eq!(None, NodeId::from_str("r").child_index());
    }
}
