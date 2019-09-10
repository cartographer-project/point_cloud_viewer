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
use crate::math::Cube;
use crate::proto;
use crate::read_write::PositionEncoding;
use crate::CURRENT_VERSION;
use cgmath::{Matrix4, Point3};
use collision::{Aabb, Aabb3, Relation};
use fnv::FnvHashMap;
use num::clamp;
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::io::{BufReader, Read};
use std::path::PathBuf;

use crate::batch_iterator::{PointCloud, PointQuery};

mod generation;
pub use self::generation::{build_octree, build_octree_from_file};

mod factory;
pub use self::factory::OctreeFactory;

mod node;
pub use self::node::{to_node_proto, ChildIndex, Node, NodeId, NodeMeta};

mod octree_iterator;
pub use self::octree_iterator::{FilteredPointsIterator, NodeIdsIterator};

pub use crate::data_provider::{DataProvider, OnDiskDataProvider};

#[cfg(test)]
mod octree_test;

#[derive(Clone, Debug)]
pub struct OctreeMeta {
    pub resolution: f64,
    pub bounding_box: Aabb3<f64>,
}

pub fn to_meta_proto(octree_meta: &OctreeMeta, nodes: Vec<proto::OctreeNode>) -> proto::Meta {
    let mut octree_proto = proto::OctreeMeta::new();
    octree_proto
        .mut_bounding_box()
        .mut_min()
        .set_x(octree_meta.bounding_box.min().x);
    octree_proto
        .mut_bounding_box()
        .mut_min()
        .set_y(octree_meta.bounding_box.min().y);
    octree_proto
        .mut_bounding_box()
        .mut_min()
        .set_z(octree_meta.bounding_box.min().z);
    octree_proto
        .mut_bounding_box()
        .mut_max()
        .set_x(octree_meta.bounding_box.max().x);
    octree_proto
        .mut_bounding_box()
        .mut_max()
        .set_y(octree_meta.bounding_box.max().y);
    octree_proto
        .mut_bounding_box()
        .mut_max()
        .set_z(octree_meta.bounding_box.max().z);
    octree_proto.set_resolution(octree_meta.resolution);

    let octree_nodes = ::protobuf::RepeatedField::<proto::OctreeNode>::from_vec(nodes);
    octree_proto.set_nodes(octree_nodes);

    let mut meta = proto::Meta::new();
    meta.set_version(CURRENT_VERSION);
    meta.set_octree(octree_proto);
    meta
}

// TODO(hrapp): something is funky here. "r" is smaller on screen than "r4" in many cases, though
// that is impossible.
fn project(m: &Matrix4<f64>, p: &Point3<f64>) -> Point3<f64> {
    let q = m * Point3::to_homogeneous(*p);
    Point3::from_homogeneous(q / q.w)
}

fn clip_point_to_hemicube(p: &Point3<f64>) -> Point3<f64> {
    Point3::new(clamp(p.x, -1., 1.), clamp(p.y, -1., 1.), clamp(p.z, 0., 1.))
}

// This method projects world points through the matrix and returns a value proportional to the
// size of the screen. We do not know the width and height of the frustum in pixels here, so the
// unit of the value is not really well defined.
// Ideally we'd need to know the aspect ration of width/height to make the returned value truly
// proportional to the size on the screen, but this parameter would need to be passed through to
// all API calls. I decided on relying on a 'good enough' metric instead which did not require the
// parameter.
fn relative_size_on_screen(bounding_cube: &Cube, matrix: &Matrix4<f64>) -> f64 {
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

pub struct Octree {
    data_provider: Box<dyn DataProvider>,
    meta: OctreeMeta,
    nodes: FnvHashMap<NodeId, NodeMeta>,
}

#[derive(Debug)]
pub struct NodeData {
    pub meta: NodeMeta,
    pub position: Vec<u8>,
    pub color: Vec<u8>,
}

fn bounding_box_to_aabb(bounding_box: &proto::AxisAlignedCuboid) -> Aabb3<f64> {
    let min = bounding_box.min.clone().unwrap_or_else(|| {
        let deprecated_min = bounding_box.deprecated_min.clone().unwrap(); // Version 9
        let mut v = proto::Vector3d::new();
        v.set_x(f64::from(deprecated_min.x));
        v.set_y(f64::from(deprecated_min.y));
        v.set_z(f64::from(deprecated_min.z));
        v
    });
    let max = bounding_box.max.clone().unwrap_or_else(|| {
        let deprecated_max = bounding_box.deprecated_max.clone().unwrap(); // Version 9
        let mut v = proto::Vector3d::new();
        v.set_x(f64::from(deprecated_max.x));
        v.set_y(f64::from(deprecated_max.y));
        v.set_z(f64::from(deprecated_max.z));
        v
    });
    Aabb3::new(
        Point3::new(min.x, min.y, min.z),
        Point3::new(max.x, max.y, max.z),
    )
}

impl Octree {
    // TODO(sirver): This creates an object that is only partially usable.
    pub fn from_data_provider(data_provider: Box<dyn DataProvider>) -> Result<Self> {
        let bounding_box;
        let meta;
        let nodes_proto;

        let meta_proto = data_provider.meta_proto()?;
        match meta_proto.version {
            9 | 10 | 11 => {
                println!(
                    "Data is an older octree version: {}, current would be {}. \
                     If feasible, try upgrading this octree using `upgrade_octree`.",
                    meta_proto.version, CURRENT_VERSION
                );
                bounding_box = bounding_box_to_aabb(meta_proto.get_deprecated_bounding_box());
                meta = OctreeMeta {
                    resolution: meta_proto.deprecated_resolution,
                    bounding_box,
                };
                nodes_proto = meta_proto.get_deprecated_nodes();
            }
            CURRENT_VERSION => {
                if !meta_proto.has_octree() {
                    return Err(ErrorKind::InvalidInput("No octree meta found".to_string()).into());
                }
                let octree_meta = meta_proto.get_octree();
                bounding_box = bounding_box_to_aabb(octree_meta.get_bounding_box());
                meta = OctreeMeta {
                    resolution: octree_meta.resolution,
                    bounding_box,
                };
                nodes_proto = octree_meta.get_nodes();
            }
            _ => return Err(ErrorKind::InvalidVersion(meta_proto.version).into()),
        }

        let mut nodes = FnvHashMap::default();

        for node_proto in nodes_proto.iter() {
            let node_id = NodeId::from_proto(node_proto.id.as_ref().unwrap());
            nodes.insert(
                node_id,
                NodeMeta {
                    num_points: node_proto.num_points,
                    position_encoding: PositionEncoding::from_proto(node_proto.position_encoding)?,
                    bounding_cube: node_id.find_bounding_cube(&Cube::bounding(&bounding_box)),
                },
            );
        }

        Ok(Octree {
            meta,
            nodes,
            data_provider,
        })
    }

    pub fn to_meta_proto(&self) -> proto::Meta {
        let nodes: Vec<proto::OctreeNode> = self
            .nodes
            .iter()
            .map(|(id, node_meta)| {
                to_node_proto(&id, node_meta.num_points, &node_meta.position_encoding)
            })
            .collect();
        to_meta_proto(&self.meta, nodes)
    }

    pub fn get_visible_nodes(&self, projection_matrix: &Matrix4<f64>) -> Vec<NodeId> {
        let frustum = collision::Frustum::from_matrix4(*projection_matrix).unwrap();
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
            if !current.empty {
                visible.push(current.node.id);
            }
        }
        visible
    }

    pub fn get_node_data(&self, node_id: &NodeId) -> Result<NodeData> {
        // TODO(hrapp): If we'd randomize the points while writing, we could just read the
        // first N points instead of reading everything and skipping over a few.
        let mut position_color_reads = self
            .data_provider
            .data(&node_id.to_string(), &["position", "color"])?;

        let mut get_data = |node_attribute: &str, err: &str| -> Result<Vec<u8>> {
            let mut reader = BufReader::new(
                position_color_reads
                    .remove(node_attribute)
                    .ok_or_else(|| err)?,
            );
            let mut all_data = Vec::new();
            reader.read_to_end(&mut all_data).chain_err(|| err)?;
            Ok(all_data)
        };
        let position = get_data("position", "Could not read position")?;
        let color = get_data("color", "Could not read color")?;

        Ok(NodeData {
            position,
            color,
            meta: self.nodes[node_id].clone(),
        })
    }

    /// return the bounding box saved in meta
    pub fn bounding_box(&self) -> &Aabb3<f64> {
        &self.meta.bounding_box
    }
}

impl PointCloud for Octree {
    type Id = NodeId;
    type PointsIter = FilteredPointsIterator;
    fn nodes_in_location(&self, query: &PointQuery) -> Vec<Self::Id> {
        let container = query.get_point_culling();
        let filter_func = move |node_id: &NodeId, octree: &Octree| -> bool {
            let current = &octree.nodes[&node_id];
            container.intersects_aabb3(&current.bounding_cube.to_aabb3())
        };
        NodeIdsIterator::new(&self, filter_func).collect()
    }

    fn points_in_node<'a>(&'a self, query: &PointQuery, node_id: NodeId) -> FilteredPointsIterator {
        let container = query.get_point_culling();
        FilteredPointsIterator::new(&self, node_id, container)
    }
}

struct OpenNode {
    node: Node,
    relation: Relation,
    size_on_screen: f64,
    empty: bool,
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
    projection_matrix: &Matrix4<f64>,
) {
    if let Some(meta) = nodes.get(&node.id) {
        let size_on_screen = relative_size_on_screen(&node.bounding_cube, projection_matrix);
        v.push(OpenNode {
            node,
            relation,
            size_on_screen,
            empty: meta.num_points == 0,
        });
    }
}

//  TODO(catevita): refactor function for octree factory
pub fn octree_from_directory(directory: impl Into<PathBuf>) -> Result<Box<Octree>> {
    let data_provider = OnDiskDataProvider {
        directory: directory.into(),
    };
    let octree = Octree::from_data_provider(Box::new(data_provider))?;
    Ok(Box::new(octree))
}
