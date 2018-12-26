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

// NOCOM(#sirver): remove
#![allow(unused_imports)]

use byteorder::{LittleEndian, ReadBytesExt, WriteBytesExt};
use cgmath::{EuclideanSpace, Matrix4, Point3, Vector3, Vector4};
use collision::{Aabb, Aabb3, Contains, Discrete, Frustum, Relation};
use errors::*;
use fnv::FnvHashMap;
use math::Cube;
use proto;
use protobuf;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::fs::File;
use std::io::{self, BufReader, Cursor, Read};
use std::path::{Path, PathBuf};
use {InternalIterator, Point};

mod node;

pub use self::node::{
    ChildIndex, Node, NodeId, NodeIterator, NodeMeta, NodeWriter, PositionEncoding,
};

pub const CURRENT_VERSION: i32 = 9;

#[derive(Debug, Clone, Copy)]
pub enum LayerKind {
    // NOCOM(#sirver): explain resolution
    // A vector of 3 floats, the parameter is the 'resolution'.
    // NOCOM(#sirver): add resolution
    F32Vec3,

    // One float with its 'resolution'.
    // NOCOM(#sirver): add resolution
    F32,

    // A Vector of 3 u8.
    U8Vec3,
}

impl LayerKind {
    pub fn read_from(&self, num_points: usize, mut reader: impl io::Read) -> std::result::Result<LayerData, io::Error> {
        match *self {
            LayerKind::F32 => {
                let mut d = Vec::with_capacity(num_points);
                for _ in 0 .. num_points {
                    d.push(reader.read_f32::<LittleEndian>()?);
                }
                Ok(LayerData::F32(d))
            }
            LayerKind::F32Vec3 => {
                let mut d = Vec::with_capacity(num_points);
                for _ in 0 .. num_points {
                    let x = reader.read_f32::<LittleEndian>()?;
                    let y = reader.read_f32::<LittleEndian>()?;
                    let z = reader.read_f32::<LittleEndian>()?;
                    d.push(Vector3::new(x, y, z));
                }
                Ok(LayerData::F32Vec3(d))
            }
            LayerKind::U8Vec3 => {
                let mut d = Vec::with_capacity(num_points);
                for _ in 0 .. num_points {
                    let x = reader.read_u8()?;
                    let y = reader.read_u8()?;
                    let z = reader.read_u8()?;
                    d.push(Vector3::new(x, y, z));
                }
                Ok(LayerData::U8Vec3(d))
            }
        }
    }
}


#[derive(Debug, Clone)]
pub enum LayerData {
    // NOCOM(#sirver): explain resolution
    // A vector of 3 floats, the parameter is the 'resolution'.
    // NOCOM(#sirver): add resolution
    F32Vec3(Vec<cgmath::Vector3<f32>>),

    // One float with its 'resolution'.
    // NOCOM(#sirver): add resolution
    F32(Vec<f32>),

    // A Vector of 4 u8.
    U8Vec3(Vec<cgmath::Vector3<u8>>),
}

impl LayerData {
    // NOCOM(#sirver): this will be very slow
    pub fn push(&mut self, item: cgmath::Vector3<f32>) {
        match *self {
            LayerData::F32Vec3(ref mut data) => data.push(item),
            _ => panic!("Cannot push this data type onto this kind of layer."),
        }
    }

    pub fn extend(&mut self, other: LayerData) {
        use self::LayerData::*;
        match (self, other) {
            (F32Vec3(ref mut us), F32Vec3(ref them)) => us.extend(them),
            (F32(ref mut us), F32(ref them)) => us.extend(them),
            (U8Vec3(ref mut us), U8Vec3(ref them)) => us.extend(them),
            _ => panic!("Cannot extent this data type on this other one."),
        }
    }

    pub fn len(&self) -> usize {
        match self {
            LayerData::F32(data) => data.len(),
            LayerData::F32Vec3(data) => data.len(),
            LayerData::U8Vec3(data) => data.len(),
        }
    }

    pub fn write_point_with_index_into(&self, mut writer: impl io::Write, index: usize) -> std::result::Result<(), io::Error> {
        match self {
            LayerData::F32(data) => {
                writer.write_f32::<LittleEndian>(data[index])?;
            }
            LayerData::F32Vec3(data) => {
                let p = data[index];
                writer.write_f32::<LittleEndian>(p.x)?;
                writer.write_f32::<LittleEndian>(p.y)?;
                writer.write_f32::<LittleEndian>(p.z)?;
            }
            LayerData::U8Vec3(data) => {
                let p = data[index];
                writer.write_u8(p.x)?;
                writer.write_u8(p.y)?;
                writer.write_u8(p.z)?;
            }
        }
        Ok(())
    }

}

#[derive(Debug, Default)]
pub struct PointData {
    pub position: Vec<cgmath::Vector3<f32>>,
    pub layers: FnvHashMap<String, LayerData>,
}

#[derive(Debug)]
pub struct OctreeMeta {
    pub directory: PathBuf,
    pub resolution: f64,
    pub bounding_box: Aabb3<f32>,
    pub layers: FnvHashMap<String, LayerKind>,
}

// TODO(hrapp): something is funky here. "r" is smaller on screen than "r4" in many cases, though
// that is impossible.
fn project(m: &Matrix4<f32>, p: &Point3<f32>) -> Point3<f32> {
    let d = 1. / (m[0][3] * p.x + m[1][3] * p.y + m[2][3] * p.z + m[3][3]);
    Point3::new(
        (m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z + m[3][0]) * d,
        (m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z + m[3][1]) * d,
        (m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z + m[3][2]) * d,
    )
}

// This method projects world points through the matrix and returns a value proportional to the
// size of the screen. We do not know the width and height of the frustum in pixels here, so the
// unit of the value is not really well defined.
// Ideally we'd need to know the aspect ration of width/height to make the returned value truly
// proportional to the size on the screen, but this parameter would need to be passed through to
// all API calls. I decided on relying on a 'good enough' metric instead which did not require the
// parameter.
fn relative_size_on_screen(bounding_cube: &Cube, matrix: &Matrix4<f32>) -> f32 {
    // z is unused here.
    let min = bounding_cube.min();
    let max = bounding_cube.max();
    let mut rv = Aabb3::zero();
    for p in &[
        Point3::new(min.x, min.y, min.z),
        Point3::new(max.x, min.y, min.z),
        Point3::new(min.x, max.y, min.z),
        Point3::new(max.x, max.y, min.z),
        Point3::new(min.x, min.y, max.z),
        Point3::new(max.x, min.y, max.z),
        Point3::new(min.x, max.y, max.z),
        Point3::new(max.x, max.y, max.z),
    ] {
        rv = rv.grow(project(matrix, p));
    }
    (rv.max().x - rv.min().x) * (rv.max().y - rv.min().y)
}

#[derive(Debug)]
pub struct OnDiskOctree {
    meta: OctreeMeta,
    nodes: FnvHashMap<NodeId, NodeMeta>,
}

pub trait Octree: Send + Sync {
    fn get_visible_nodes(&self, projection_matrix: &Matrix4<f32>) -> Vec<NodeId>;
    fn get_node_data(&self, node_id: &NodeId) -> Result<NodeData>;
}

pub struct PointsInBoxIterator<'a> {
    octree_meta: &'a OctreeMeta,
    aabb: &'a Aabb3<f32>,
    intersecting_nodes: Vec<NodeId>,
}

impl<'a> InternalIterator for PointsInBoxIterator<'a> {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each_batch<F: FnMut(&PointData)>(self, _: F) {
        unimplemented!();
        // NOCOM(#sirver): needs reimplementation
        // for node_id in &self.intersecting_nodes {
        // // TODO(sirver): This crashes on error. We should bubble up an error.
        // let iterator = NodeIterator::from_disk(&self.octree_meta, node_id)
        // .expect("Could not read node points");
        // iterator.for_each_batch(|p| {
        // if !self.aabb.contains(&Point3::from_vec(p.position)) {
        // return;
        // }
        // f(p);
        // });
        // }
    }
}

pub struct PointsInFrustumIterator<'a> {
    octree_meta: &'a OctreeMeta,
    frustum_matrix: &'a Matrix4<f32>,
    intersecting_nodes: Vec<NodeId>,
}

impl<'a> InternalIterator for PointsInFrustumIterator<'a> {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each_batch<F: FnMut(&PointData)>(self, _: F) {
        unimplemented!();
        // NOCOM(#sirver): needs reimplementation
        // for node_id in &self.intersecting_nodes {
        // let iterator = NodeIterator::from_disk(&self.octree_meta, node_id)
        // .expect("Could not read node points");
        // iterator.for_each_batch(|p| {
        // if !contains(self.frustum_matrix, &Point3::from_vec(p.position)) {
        // return;
        // }
        // f(p);
        // });
        // }
    }
}

pub struct AllPointsIterator<'a> {
    octree_meta: &'a OctreeMeta,
    octree_nodes: &'a FnvHashMap<NodeId, NodeMeta>,
}

impl<'a> InternalIterator for AllPointsIterator<'a> {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each_batch<F: FnMut(&PointData)>(self, _: F) {
        // NOCOM(#sirver): needs reimplementation
        // let mut open_list = vec![NodeId::from_level_index(0, 0)];
        // while !open_list.is_empty() {
            // let current = open_list.pop().unwrap();
            // let iterator = NodeIterator::from_disk(&self.octree_meta, &current)
                // .expect("Could not read node points");
            // iterator.for_each(|p| { f(p) });
            // for child_index in 0..8 {
                // let child_id = current.get_child_id(ChildIndex::from_u8(child_index));
                // if self.octree_nodes.contains_key(&child_id) {
                    // open_list.push(child_id);
                // }
            // }
        // }
    }
}

// TODO(ksavinash9) update after https://github.com/rustgd/collision-rs/issues/101 is resolved.
fn contains(projection_matrix: &Matrix4<f32>, point: &Point3<f32>) -> bool {
    let v = Vector4::new(point.x, point.y, point.z, 1.);
    let clip_v = projection_matrix * v;
    return clip_v.x.abs() < clip_v.w
        && clip_v.y.abs() < clip_v.w
        && 0. < clip_v.z
        && clip_v.z < clip_v.w;
}

pub fn read_meta_proto<P: AsRef<Path>>(directory: P) -> Result<proto::Meta> {
    // We used to use JSON earlier.
    if directory.as_ref().join("meta.json").exists() {
        return Err(ErrorKind::InvalidVersion(3).into());
    }

    let mut data = Vec::new();
    File::open(&directory.as_ref().join("meta.pb"))?.read_to_end(&mut data)?;
    Ok(
        protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
            .chain_err(|| "Could not parse meta.pb")?,
    )
}

#[derive(Debug)]
pub struct NodeData {
    pub meta: node::NodeMeta,
    pub position: Vec<u8>,
    pub color: Vec<u8>,
}

impl OnDiskOctree {
    // TODO(sirver): This creates an object that is only partially usable.
    pub fn from_meta(meta_proto: proto::Meta, directory: PathBuf) -> Result<Self> {
        if meta_proto.version != CURRENT_VERSION {
            return Err(ErrorKind::InvalidVersion(meta_proto.version).into());
        }

        let bounding_box = {
            let bounding_box = meta_proto.bounding_box.unwrap();
            let min = bounding_box.min.unwrap();
            let max = bounding_box.max.unwrap();
            Aabb3::new(
                Point3::new(min.x, min.y, min.z),
                Point3::new(max.x, max.y, max.z),
            )
        };

        // NOCOM(#sirver): load the description from the proto.

        // The layers description was not in the proto. This is presumably and older octree that
        // still had a few hardcoded layers in it. Let's recreate its structure manually:
        let layers = {
            let mut layers = FnvHashMap::default();
            layers.insert("color".to_string(), LayerKind::U8Vec3);
            if directory.join("r.intensity").exists() {
                layers.insert("intensity".to_string(), LayerKind::F32);
            }
            layers
        };

        let meta = OctreeMeta {
            directory: directory.into(),
            resolution: meta_proto.resolution,
            bounding_box: bounding_box,
            layers: layers.clone(),
        };

        let mut nodes = FnvHashMap::default();
        for node_proto in meta_proto.nodes.iter() {
            let node_id = NodeId::from_level_index(
                node_proto.id.as_ref().unwrap().level as u8,
                node_proto.id.as_ref().unwrap().index as usize,
            );
            nodes.insert(
                node_id,
                NodeMeta {
                    num_points: node_proto.num_points,
                    position_encoding: PositionEncoding::from_proto(node_proto.position_encoding)?,
                    bounding_cube: node_id.find_bounding_cube(&Cube::bounding(&meta.bounding_box)),
                    // NOCOM(#sirver): this feels wrong: not every node should duplicated the
                    // knowledge about which layers are there or not from the octree base.
                    layers: layers.clone(),
                },
            );
        }
        Ok(OnDiskOctree { meta, nodes })
    }

    pub fn new<P: AsRef<Path>>(directory: P) -> Result<Self> {
        let directory = directory.as_ref().to_owned();
        let meta_proto = read_meta_proto(&directory)?;
        Self::from_meta(meta_proto, directory)
    }

    /// Returns the ids of all nodes that cut or are fully contained in 'aabb'.
    pub fn points_in_box<'a>(&'a self, aabb: &'a Aabb3<f32>) -> PointsInBoxIterator<'a> {
        let mut intersecting_nodes = Vec::new();
        let mut open_list = vec![Node::root_with_bounding_cube(Cube::bounding(
            &self.meta.bounding_box,
        ))];
        while !open_list.is_empty() {
            let current = open_list.pop().unwrap();
            if !aabb.intersects(&current.bounding_cube.to_aabb3()) {
                continue;
            }
            intersecting_nodes.push(current.id);
            for child_index in 0..8 {
                let child = current.get_child(ChildIndex::from_u8(child_index));
                if self.nodes.contains_key(&child.id) {
                    open_list.push(child);
                }
            }
        }
        PointsInBoxIterator {
            octree_meta: &self.meta,
            aabb,
            intersecting_nodes,
        }
    }

    pub fn points_in_frustum<'a>(
        &'a self,
        frustum_matrix: &'a Matrix4<f32>,
    ) -> PointsInFrustumIterator<'a> {
        let intersecting_nodes = self.get_visible_nodes(&frustum_matrix);
        PointsInFrustumIterator {
            octree_meta: &self.meta,
            frustum_matrix,
            intersecting_nodes,
        }
    }

    pub fn all_points<'a>(&'a self) -> AllPointsIterator<'a> {
        AllPointsIterator {
            octree_meta: &self.meta,
            octree_nodes: &self.nodes,
        }
    }

    pub fn bounding_box(&self) -> &Aabb3<f32> {
        &self.meta.bounding_box
    }
}

struct OpenNode {
    node: Node,
    relation: Relation,
    size_on_screen: f32,
}

impl Ord for OpenNode {
    fn cmp(&self, other: &OpenNode) -> Ordering {
        if self.size_on_screen == other.size_on_screen {
            return Ordering::Equal;
        }
        if self.size_on_screen < other.size_on_screen {
            Ordering::Less
        } else {
            Ordering::Greater
        }
    }
}

impl PartialOrd for OpenNode {
    fn partial_cmp(&self, other: &OpenNode) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl PartialEq for OpenNode {
    fn eq(&self, other: &OpenNode) -> bool {
        self.size_on_screen == other.size_on_screen
    }
}

impl Eq for OpenNode {}

#[inline]
fn maybe_push_node(
    v: &mut BinaryHeap<OpenNode>,
    nodes: &FnvHashMap<NodeId, NodeMeta>,
    relation: Relation,
    node: Node,
    projection_matrix: &Matrix4<f32>,
) {
    if !nodes.contains_key(&node.id) {
        return;
    }
    let size_on_screen = relative_size_on_screen(&node.bounding_cube, projection_matrix);
    v.push(OpenNode {
        node,
        relation,
        size_on_screen,
    });
}

impl Octree for OnDiskOctree {
    fn get_visible_nodes(&self, projection_matrix: &Matrix4<f32>) -> Vec<NodeId> {
        let frustum = Frustum::from_matrix4(*projection_matrix).unwrap();
        let mut open = BinaryHeap::new();
        maybe_push_node(
            &mut open,
            &self.nodes,
            Relation::Cross,
            Node::root_with_bounding_cube(Cube::bounding(&self.meta.bounding_box)),
            projection_matrix,
        );

        let mut visible = Vec::new();
        while let Some(current) = open.pop() {
            match current.relation {
                Relation::Cross => {
                    for child_index in 0..8 {
                        let child = current.node.get_child(ChildIndex::from_u8(child_index));
                        let child_relation = frustum.contains(&child.bounding_cube.to_aabb3());
                        if child_relation == Relation::Out {
                            continue;
                        }
                        maybe_push_node(
                            &mut open,
                            &self.nodes,
                            child_relation,
                            child,
                            projection_matrix,
                        );
                    }
                }
                Relation::In => {
                    // When the parent is fully in the frustum, so are the children.
                    for child_index in 0..8 {
                        maybe_push_node(
                            &mut open,
                            &self.nodes,
                            Relation::In,
                            current.node.get_child(ChildIndex::from_u8(child_index)),
                            projection_matrix,
                        );
                    }
                }
                Relation::Out => {
                    // This should never happen.
                    unreachable!();
                }
            };
            visible.push(current.node.id);
        }
        visible
    }

    fn get_node_data(&self, node_id: &NodeId) -> Result<NodeData> {
        let stem = node_id.get_stem(&self.meta.directory);

        // TODO(hrapp): If we'd randomize the points while writing, we could just read the
        // first N points instead of reading everything and skipping over a few.
        let position = {
            let mut xyz_reader =
                BufReader::new(File::open(&stem.with_extension(node::POSITION_EXT))?);
            let mut all_data = Vec::new();
            xyz_reader
                .read_to_end(&mut all_data)
                .chain_err(|| "Could not read position")?;
            all_data
        };

        let color = {
            let mut rgb_reader = BufReader::new(
                File::open(&stem.with_extension(node::COLOR_EXT))
                    .chain_err(|| "Could not read color")?,
            );
            let mut all_data = Vec::new();
            rgb_reader
                .read_to_end(&mut all_data)
                .chain_err(|| "Could not read color")?;
            all_data
        };

        Ok(NodeData {
            position: position,
            color: color,
            meta: self.nodes[node_id].clone(),
        })
    }
}
