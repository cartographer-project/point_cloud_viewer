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

// NOCOM(#sirver): what
#![allow(unused_imports)]

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use cgmath::{Vector3, Zero};
use color;
use errors::*;
use fnv::FnvHashMap;
use math::{clamp, Cube};
use num;
use num_traits;
use octree::{self, OctreeMeta};
use proto;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};
use std::{fmt, result};
use {InternalIterator, Point, NUM_POINTS_PER_BATCH};

pub const POSITION_EXT: &str = "xyz";
pub const COLOR_EXT: &str = "rgb";

fn extension_from_layer_name(name: &str) -> &str {
    // For backwards compatibility, color extension is changed.
    match name as &str {
        "color" => "rgb",
        other => other,
    }
}

/// Represents a child of an octree Node.
#[derive(Debug, PartialEq, Eq)]
pub struct ChildIndex(u8);

impl ChildIndex {
    pub fn from_u8(index: u8) -> Self {
        assert!(index < 8);
        ChildIndex(index)
    }

    /// Returns the ChildId of the child containing 'v'.
    pub fn from_bounding_cube(bounding_cube: &Cube, v: &Vector3<f32>) -> ChildIndex {
        // This is a bit flawed: it is not guaranteed that 'child_bounding_box.contains(&v)' is true
        // using this calculated index due to floating point precision.
        let center = bounding_cube.center();
        let gt_x = v.x > center.x;
        let gt_y = v.y > center.y;
        let gt_z = v.z > center.z;
        ChildIndex((gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8)
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

    pub fn from_level_index(level: u8, index: usize) -> Self {
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
    #[inline]
    pub fn get_child_id(&self, child_index: ChildIndex) -> Self {
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
    pub fn parent_id(&self) -> Option<NodeId> {
        if self.level() == 0 {
            return None;
        }
        Some(NodeId {
            level: self.level - 1,
            index: (self.index >> 3),
        })
    }

    /// Returns the level of this node in the octree, with 0 being the root.
    pub fn level(&self) -> usize {
        self.level as usize
    }

    /// Returns the index of this node at the current level.
    pub fn index(&self) -> usize {
        self.index as usize
    }

    /// Computes the bounding cube from a NodeID.
    pub fn find_bounding_cube(&self, root_bounding_cube: &Cube) -> Cube {
        let mut edge_length = root_bounding_cube.edge_length();
        let mut min = root_bounding_cube.min();
        for level in (0..self.level).rev() {
            edge_length /= 2.;
            // Reverse order: process from root to leaf nodes.
            let child_index = (self.index >> (3 * level)) & 7;
            let z = (child_index >> 0) & 1;
            let y = (child_index >> 1) & 1;
            let x = (child_index >> 2) & 1;
            min.x += x as f32 * edge_length;
            min.y += y as f32 * edge_length;
            min.z += z as f32 * edge_length;
        }
        Cube::new(min, edge_length)
    }

    // Get number of points from the file size of the color data.
    // Color data is required and always present.
    fn number_of_points(&self, directory: &Path) -> Result<i64> {
        // NOCOM(#sirver): This is broken now and should no longer be used.
        let file_meta_data_opt = fs::metadata(self.get_stem(directory).with_extension(COLOR_EXT));
        if file_meta_data_opt.is_err() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let file_size_bytes = file_meta_data_opt.unwrap().len();
        // color has 3 bytes per point
        Ok((file_size_bytes / 3) as i64)
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

    #[inline]
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

#[derive(Clone, Debug)]
pub struct NodeMeta {
    pub num_points: i64,
    pub position_encoding: PositionEncoding,
    pub bounding_cube: Cube,
    pub layers: FnvHashMap<String, octree::LayerKind>,
}

impl NodeMeta {
    pub fn num_points_for_level_of_detail(&self, level_of_detail: i32) -> i64 {
        (self.num_points as f32 / level_of_detail as f32).ceil() as i64
    }
}

/// All data required to read data from a written layer.
struct LayerReader {
    reader: BufReader<File>,
    kind: octree::LayerKind,
}

/// Streams points from our node on-disk representation.
pub struct NodeIterator {
    xyz_reader: BufReader<File>,
    layer_readers: FnvHashMap<String, LayerReader>,
    meta: NodeMeta,
}

impl NodeIterator {
    pub fn from_disk(octree_meta: &OctreeMeta, id: &NodeId) -> Result<Self> {
        let stem = id.get_stem(&octree_meta.directory);
        let num_points = id.number_of_points(&octree_meta.directory)?;
        let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);

        let mut layer_readers = FnvHashMap::default();
        for (name, kind) in &octree_meta.layers {
            let ext = extension_from_layer_name(&name);
            let reader = BufReader::new(File::open(&stem.with_extension(ext))?);
            layer_readers.insert(
                name.to_string(),
                LayerReader {
                    reader,
                    kind: *kind,
                },
            );
        }

        Ok(NodeIterator {
            xyz_reader: BufReader::new(File::open(&stem.with_extension(POSITION_EXT))?),
            layer_readers,
            meta: NodeMeta {
                bounding_cube: bounding_cube,
                position_encoding: position_encoding,
                num_points: num_points,
                layers: octree_meta.layers.clone(),
            },
        })
    }

    fn read_positions(&mut self, num_points_to_process: usize, position: &mut Vec<Vector3<f32>>) {
        let edge_length = self.meta.bounding_cube.edge_length();
        let min = self.meta.bounding_cube.min();
        match self.meta.position_encoding {
            PositionEncoding::Float32 => {
                for _ in 0..num_points_to_process {
                    let x = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.x,
                        edge_length,
                    );
                    let y = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.y,
                        edge_length,
                    );
                    let z = decode(
                        self.xyz_reader.read_f32::<LittleEndian>().unwrap(),
                        min.z,
                        edge_length,
                    );
                    position.push(Vector3::new(x, y, z));
                }
            }
            PositionEncoding::Uint8 => {
                for _ in 0..num_points_to_process {
                    let x = fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.x, edge_length);
                    let y = fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.y, edge_length);
                    let z = fixpoint_decode(self.xyz_reader.read_u8().unwrap(), min.z, edge_length);
                    position.push(Vector3::new(x, y, z));
                }
            }
            PositionEncoding::Uint16 => {
                for _ in 0..num_points_to_process {
                    let x = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.x,
                        edge_length,
                    );
                    let y = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.y,
                        edge_length,
                    );
                    let z = fixpoint_decode(
                        self.xyz_reader.read_u16::<LittleEndian>().unwrap(),
                        min.z,
                        edge_length,
                    );
                    position.push(Vector3::new(x, y, z));
                }
            }
        }
    }
}

impl InternalIterator for NodeIterator {
    fn size_hint(&self) -> Option<usize> {
        Some(self.meta.num_points as usize)
    }

    fn for_each_batch<F: FnMut(&octree::PointData)>(mut self, mut func: F) {
        let mut num_points_processed = 0;
        while num_points_processed < self.meta.num_points {
            let mut point_data = octree::PointData::default();
            let num_points_to_process =
                (self.meta.num_points - num_points_processed).min(NUM_POINTS_PER_BATCH);

            self.read_positions(num_points_to_process as usize, &mut point_data.position);

            for (name, layer_reader) in &mut self.layer_readers {
                let data = layer_reader
                    .kind
                    .read_from(num_points_to_process as usize, &mut layer_reader.reader)
                    .unwrap();
                point_data.layers.insert(name.to_string(), data);
            }
            num_points_processed += num_points_to_process;

            func(&point_data);
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PositionEncoding {
    Uint8,
    Uint16,
    Float32,
}

impl PositionEncoding {
    pub fn new(bounding_cube: &Cube, resolution: f64) -> PositionEncoding {
        let min_bits = (f64::from(bounding_cube.edge_length()) / resolution).log2() as u32 + 1;
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
    layer_writers: FnvHashMap<String, BufWriter<File>>,
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
        }

        // TODO(hrapp): Add some sanity checks that we do not have nodes with ridiculously low
        // amount of points laying around?
    }
}

impl NodeWriter {
    pub fn new(octree_meta: &OctreeMeta, node_id: &NodeId) -> Self {
        let stem = node_id.get_stem(&octree_meta.directory);
        let bounding_cube = node_id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));

        // NOCOM(#sirver): change write to lift
        let mut layer_writers = FnvHashMap::default();
        for name in octree_meta.layers.keys() {
            let ext = extension_from_layer_name(&name);
            let writer = BufWriter::new(File::create(&stem.with_extension(ext)).unwrap());
            layer_writers.insert(name.to_string(), writer);
        }
        let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);
        NodeWriter {
            bounding_cube,
            xyz_writer: BufWriter::new(File::create(&stem.with_extension(POSITION_EXT)).unwrap()),
            position_encoding,
            layer_writers,
            stem: stem,
            num_written: 0,
        }
    }

    // NOCOM(#sirver): should return an error
    /// Writers the point with the given 'index' from 'point_data' into this writer.
    pub fn write_point_with_index(&mut self, index: usize, point_data: &octree::PointData) {
        self.write_position(&point_data.position[index]);

        let mut layers_written = 0;
        for (name, mut writer) in &mut self.layer_writers {
            match point_data.layers.get(name) {
                None => {
                    // TODO(sirver): This should write a default value to make not every value for
                    // every point mandatory. As soon as we compress the node data before writing
                    // it to disk, this waste will not significantly matter anymore.
                }
                Some(data) => {
                    data.write_point_with_index_into(&mut writer, index).unwrap();
                    layers_written += 1;
                }
            }
        }
        // TODO(sirver): Handle this assert case by writing default values for not-provided layers.
        assert_eq!(
            layers_written,
            self.layer_writers.len(),
            "We did not receive data for every layer. \
             This will be handled in the future, but right now is not"
        );

        self.num_written += 1;
    }

    fn write_position(&mut self, position: &Vector3<f32>) {
        // NOCOM(#sirver): encoding should be applied to all float properties
        let edge_length = self.bounding_cube.edge_length();
        let min = self.bounding_cube.min();
        match self.position_encoding {
            PositionEncoding::Float32 => {
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_f32::<LittleEndian>(encode(position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Uint8 => {
                self.xyz_writer
                    .write_u8(fixpoint_encode(position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u8(fixpoint_encode(position.z, min.z, edge_length))
                    .unwrap();
            }
            PositionEncoding::Uint16 => {
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(position.x, min.x, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(position.y, min.y, edge_length))
                    .unwrap();
                self.xyz_writer
                    .write_u16::<LittleEndian>(fixpoint_encode(position.z, min.z, edge_length))
                    .unwrap();
            }
        }
    }

    pub fn num_written(&self) -> i64 {
        self.num_written
    }

    fn remove_all_files(&self) {
        // We are ignoring deletion errors here in case the file is already gone.
        let _ = fs::remove_file(&self.stem.with_extension(POSITION_EXT));

        for name in self.layer_writers.keys() {
            let ext = extension_from_layer_name(&name);
            let _ = fs::remove_file(&self.stem.with_extension(ext));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cgmath::Point3;

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

    #[test]
    fn test_bounding_box() {
        let root_bounding_cube = Cube::new(Point3::new(-5., -5., -5.), 10.);

        let bounding_cube = NodeId::from_str("r0").find_bounding_cube(&root_bounding_cube);
        assert_eq!(-5., bounding_cube.min().x);
        assert_eq!(-5., bounding_cube.min().y);
        assert_eq!(-5., bounding_cube.min().z);
        assert_eq!(5., bounding_cube.edge_length());

        let bounding_cube = NodeId::from_str("r13").find_bounding_cube(&root_bounding_cube);
        assert_eq!(-5., bounding_cube.min().x);
        assert_eq!(-2.5, bounding_cube.min().y);
        assert_eq!(2.5, bounding_cube.min().z);
        assert_eq!(2.5, bounding_cube.edge_length());
    }
}
