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

use crate::errors::*;
use crate::math::{clamp, Cube};
use crate::proto;
use crate::{InternalIterator, Point};
use cgmath::{EuclideanSpace, Matrix4, Point3, Vector4};
use collision::{Aabb, Aabb3, Contains, Discrete, Frustum, Relation};
use fnv::FnvHashMap;
use protobuf;
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap};
use std::fs::{self, File};
use std::io::{BufReader, Cursor, Read};
use std::path::{Path, PathBuf};

mod node;

pub use self::node::{
    ChildIndex, Node, NodeId, NodeIterator, NodeLayer, NodeMeta, NodeWriter, PositionEncoding,
};

pub const CURRENT_VERSION: i32 = 9;

#[derive(Clone, Debug)]
pub struct OctreeMeta {
    pub resolution: f64,
    pub bounding_box: Aabb3<f32>,
}

// TODO(hrapp): something is funky here. "r" is smaller on screen than "r4" in many cases, though
// that is impossible.
fn project(m: &Matrix4<f32>, p: &Point3<f32>) -> Point3<f32> {
    let q = m * Point3::to_homogeneous(*p);
    Point3::from_homogeneous(q / q.w)
}

fn clip_point_to_hemicube(p: &Point3<f32>) -> Point3<f32> {
    Point3::new(clamp(p.x, -1., 1.), clamp(p.y, -1., 1.), clamp(p.z, 0., 1.))
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
    let mut rv = Aabb3::new(
        clip_point_to_hemicube(&project(matrix, &min)),
        clip_point_to_hemicube(&project(matrix, &max)),
    );
    for p in &[
        Point3::new(max.x, min.y, min.z),
        Point3::new(min.x, max.y, min.z),
        Point3::new(max.x, max.y, min.z),
        Point3::new(min.x, min.y, max.z),
        Point3::new(max.x, min.y, max.z),
        Point3::new(min.x, max.y, max.z),
    ] {
        rv = rv.grow(clip_point_to_hemicube(&project(matrix, p)));
    }
    (rv.max().x - rv.min().x) * (rv.max().y - rv.min().y)
}

pub trait OctreeDataProvider: Send + Sync {
    fn meta_proto(&self) -> Result<proto::Meta>;
    fn data(
        &self,
        node_id: &NodeId,
        node_layers: Vec<NodeLayer>,
    ) -> Result<HashMap<NodeLayer, Box<dyn Read>>>;
}

pub struct Octree {
    data_provider: Box<dyn OctreeDataProvider>,
    meta: OctreeMeta,
    nodes: FnvHashMap<NodeId, NodeMeta>,
}

pub struct PointsInBoxIterator<'a> {
    octree: &'a Octree,
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
            let iterator = NodeIterator::from_data_provider(
                &*self.octree.data_provider,
                &self.octree.meta,
                node_id,
                self.octree.nodes[node_id].num_points,
            )
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

pub struct PointsInFrustumIterator<'a> {
    octree: &'a Octree,
    frustum_matrix: &'a Matrix4<f32>,
    intersecting_nodes: Vec<NodeId>,
}

impl<'a> InternalIterator for PointsInFrustumIterator<'a> {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each<F: FnMut(&Point)>(self, mut f: F) {
        for node_id in &self.intersecting_nodes {
            let iterator = NodeIterator::from_data_provider(
                &*self.octree.data_provider,
                &self.octree.meta,
                node_id,
                self.octree.nodes[node_id].num_points,
            )
            .expect("Could not read node points");
            iterator.for_each(|p| {
                if !contains(self.frustum_matrix, &Point3::from_vec(p.position)) {
                    return;
                }
                f(p);
            });
        }
    }
}

pub struct AllPointsIterator<'a> {
    octree: &'a Octree,
    octree_nodes: &'a FnvHashMap<NodeId, NodeMeta>,
}

impl<'a> InternalIterator for AllPointsIterator<'a> {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each<F: FnMut(&Point)>(self, mut f: F) {
        let mut open_list = vec![NodeId::from_level_index(0, 0)];
        while !open_list.is_empty() {
            let current = open_list.pop().unwrap();
            let iterator = NodeIterator::from_data_provider(
                &*self.octree.data_provider,
                &self.octree.meta,
                &current,
                self.octree.nodes[&current].num_points,
            )
            .expect("Could not read node points");
            iterator.for_each(|p| f(p));
            for child_index in 0..8 {
                let child_id = current.get_child_id(ChildIndex::from_u8(child_index));
                if self.octree_nodes.contains_key(&child_id) {
                    open_list.push(child_id);
                }
            }
        }
    }
}

// TODO(ksavinash9) update after https://github.com/rustgd/collision-rs/issues/101 is resolved.
fn contains(projection_matrix: &Matrix4<f32>, point: &Point3<f32>) -> bool {
    let v = Vector4::new(point.x, point.y, point.z, 1.);
    let clip_v = projection_matrix * v;
    clip_v.x.abs() < clip_v.w && clip_v.y.abs() < clip_v.w && 0. < clip_v.z && clip_v.z < clip_v.w
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

impl Octree {
    // TODO(sirver): This creates an object that is only partially usable.
    pub fn from_data_provider(data_provider: Box<OctreeDataProvider>) -> Result<Self> {
        let meta_proto = data_provider.meta_proto()?;
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
            resolution: meta_proto.resolution,
            bounding_box,
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

        Ok(Octree {
            meta,
            nodes,
            data_provider,
        })
    }

    pub fn get_visible_nodes(&self, projection_matrix: &Matrix4<f32>) -> Vec<NodeId> {
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

    pub fn get_node_data(&self, node_id: &NodeId) -> Result<NodeData> {
        // TODO(hrapp): If we'd randomize the points while writing, we could just read the
        // first N points instead of reading everything and skipping over a few.
        let mut position_color_reads = self
            .data_provider
            .data(node_id, vec![NodeLayer::Position, NodeLayer::Color])?;

        let mut get_data = |node_layer: &NodeLayer, err: &str| -> Result<Vec<u8>> {
            let mut reader =
                BufReader::new(position_color_reads.remove(node_layer).ok_or_else(|| err)?);
            let mut all_data = Vec::new();
            reader.read_to_end(&mut all_data).chain_err(|| err)?;
            Ok(all_data)
        };
        let position = get_data(&NodeLayer::Position, "Could not read position")?;
        let color = get_data(&NodeLayer::Color, "Could not read color")?;

        Ok(NodeData {
            position,
            color,
            meta: self.nodes[node_id].clone(),
        })
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
            octree: &self,
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
            octree: &self,
            frustum_matrix,
            intersecting_nodes,
        }
    }

    pub fn all_points(&self) -> AllPointsIterator {
        AllPointsIterator {
            octree: &self,
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
        if self.size_on_screen > other.size_on_screen {
            Ordering::Greater
        } else if self.size_on_screen < other.size_on_screen {
            Ordering::Less
        } else {
            Ordering::Equal
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

pub struct OnDiskOctreeDataProvider {
    pub directory: PathBuf,
}

impl OnDiskOctreeDataProvider {
    /// Returns the path on disk where the data for this node is saved.
    pub fn stem(&self, node_id: &NodeId) -> PathBuf {
        self.directory.join(node_id.to_string())
    }

    // Get number of points from the file size of the color data.
    // Color data is required and always present.
    pub fn number_of_points(&self, node_id: &NodeId) -> Result<i64> {
        let stem = self.stem(node_id);
        let file_meta_data_opt = fs::metadata(stem.with_extension(NodeLayer::Color.extension()));
        if file_meta_data_opt.is_err() {
            return Err(ErrorKind::NodeNotFound.into());
        }

        let file_size_bytes = file_meta_data_opt.unwrap().len();
        // color has 3 bytes per point
        Ok((file_size_bytes / 3) as i64)
    }
}

impl OctreeDataProvider for OnDiskOctreeDataProvider {
    fn meta_proto(&self) -> Result<proto::Meta> {
        read_meta_proto(&self.directory)
    }

    fn data(
        &self,
        node_id: &NodeId,
        node_layers: Vec<NodeLayer>,
    ) -> Result<HashMap<NodeLayer, Box<dyn Read>>> {
        let stem = self.stem(node_id);
        let mut readers = HashMap::<NodeLayer, Box<dyn Read>>::new();
        for node_layer in node_layers {
            let file = match File::open(&stem.with_extension(node_layer.extension())) {
                Err(ref err) if err.kind() == ::std::io::ErrorKind::NotFound => {
                    return Err(ErrorKind::NodeNotFound.into());
                }
                e => e,
            }?;
            readers.insert(node_layer.to_owned(), Box::new(file));
        }
        Ok(readers)
    }
}

pub fn octree_from_directory(directory: impl Into<PathBuf>) -> Result<Octree> {
    let data_provider = OnDiskOctreeDataProvider {
        directory: directory.into(),
    };
    Octree::from_data_provider(Box::new(data_provider))
}
