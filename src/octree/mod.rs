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
use cgmath::{EuclideanSpace, Matrix4, Point3, Vector2};
use collision::{Aabb, Aabb3, Contains, Discrete, Frustum, Relation};
use std::collections::BinaryHeap;
use errors::*;
use fnv::FnvHashMap;
use math::Cube;
use proto;
use protobuf;
use std::fs::File;
use std::io::{BufReader, Cursor, Read};
use std::path::{Path, PathBuf};
use std::cmp::{Ordering};

mod node;

pub use self::node::{ChildIndex, Node, NodeId, NodeIterator, NodeMeta, NodeWriter,
                     PositionEncoding};

pub const CURRENT_VERSION: i32 = 9;

#[derive(Debug)]
pub struct OctreeMeta {
    pub directory: PathBuf,
    pub resolution: f64,
    pub bounding_box: Aabb3<f32>,
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

fn size_in_pixels(
    bounding_cube: &Cube,
    matrix: &Matrix4<f32>,
    width: i32,
    height: i32,
) -> Vector2<f32> {
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
    Vector2::new(
        (rv.max().x - rv.min().x) * (width as f32) / 2.,
        (rv.max().y - rv.min().y) * (height as f32) / 2.,
    )
}

#[derive(Debug)]
pub struct OnDiskOctree {
    meta: OctreeMeta,
    nodes: FnvHashMap<NodeId, NodeMeta>,
}

pub trait Octree: Send + Sync {
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4<f32>,
        width: i32,
        height: i32,
    ) -> Vec<NodeId>;

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

    fn for_each<F: FnMut(&Point)>(self, mut f: F) {
        for node_id in &self.intersecting_nodes {
            // TODO(sirver): This crashes on error. We should bubble up an error.
            let iterator = NodeIterator::from_disk(&self.octree_meta, node_id)
                .expect("Could not read node points");
            iterator.for_each(|p| {
                if !self.aabb.contains(&Point3::from_vec(p.position)) {
                    return;
                }
                f(p);
            });
        }
    }
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

        let meta = OctreeMeta {
            directory: directory.into(),
            resolution: meta_proto.resolution,
            bounding_box: bounding_box,
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
        let mut open_list = vec![
            Node::root_with_bounding_cube(Cube::bounding(&self.meta.bounding_box)),
        ];
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

    pub fn bounding_box(&self) -> &Aabb3<f32> {
        &self.meta.bounding_box
    }
}

struct OpenNode {
    node: Node,
    relation: Relation,
    pixels_sq: f32,
}

impl Ord for OpenNode {
    fn cmp(&self, other: &OpenNode) -> Ordering {
        if self.pixels_sq == other.pixels_sq {
            return Ordering::Equal;
        }
        if self.pixels_sq < other.pixels_sq {
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
        self.pixels_sq == other.pixels_sq
    }
}

impl Eq for OpenNode {
}

#[inline]
fn maybe_push_node(v: &mut BinaryHeap<OpenNode>, nodes: &FnvHashMap<NodeId, NodeMeta>, relation: Relation, node: Node, projection_matrix: &Matrix4<f32>, width: i32, height: i32) {
    if !nodes.contains_key(&node.id) {
        return;
    }
    let pixels = size_in_pixels(
        &node.bounding_cube,
        projection_matrix,
        width,
        height,
    );
    let visible_pixels = pixels.x * pixels.y;
    const MIN_PIXELS_SQ: f32 = 120.;
    const MIN_PIXELS_SIDE: f32 = 12.;
    if pixels.x < MIN_PIXELS_SIDE || pixels.y < MIN_PIXELS_SIDE
        || visible_pixels < MIN_PIXELS_SQ
    {
        return;
    }

    v.push(OpenNode {
        node, relation, pixels_sq: visible_pixels,
    });
}

impl Octree for OnDiskOctree {
    // TODO(sirver): This function becomes a bottleneck for large Octree's. It could be be
    // formulated as an iterator. To keep the order (approximately) right, the open list could be
    // replaced by a max heap looking at pixel size on screen.
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4<f32>,
        width: i32,
        height: i32,
    ) -> Vec<NodeId> {
        let frustum = Frustum::from_matrix4(*projection_matrix).unwrap();
        let mut open = BinaryHeap::new();
        maybe_push_node(&mut open,
                        &self.nodes,
                  Relation::Cross, Node::root_with_bounding_cube(Cube::bounding(&self.meta.bounding_box)),
                  projection_matrix, width, height
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
                        maybe_push_node(&mut open, &self.nodes, child_relation, child, projection_matrix, width, height);
                    }
                },
                Relation::In => {
                    // When the parent is fully in the frustum, so are the children.
                    for child_index in 0..8 {
                        maybe_push_node(&mut open, &self.nodes, Relation::In, current.node.get_child(ChildIndex::from_u8(child_index)), projection_matrix, width, height);
                    }
                },
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
            let mut rgb_reader = BufReader::new(File::open(&stem.with_extension(node::COLOR_EXT))
                .chain_err(|| "Could not read color")?);
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
