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

use errors::*;
use math::{Aabb, Aabb3f, Cube, Matrix4f, Point3f, Vector2f};
use num_traits::Zero;
use proto;
use collision::{Relation, Frustum};
use protobuf;
use std::cmp;
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufReader, Cursor, Read};
use std::path::{Path, PathBuf};

mod node;

pub use self::node::{ChildIndex, Node, NodeId, NodeIterator, NodeMeta, NodeWriter,
                     PositionEncoding};

pub const CURRENT_VERSION: i32 = 9;

#[derive(Debug)]
pub struct VisibleNode {
    pub id: NodeId,
    pub level_of_detail: i32,
    pixels: Vector2f,
}

impl VisibleNode {
    pub fn new(id: NodeId, level_of_detail: i32) -> VisibleNode {
        VisibleNode {
            id,
            level_of_detail,
            pixels: Vector2f::zero(),
        }
    }
}

#[derive(Debug)]
pub struct NodesToBlob {
    pub id: NodeId,
    pub level_of_detail: i32,
}

// TODO(hrapp): something is funky here. "r" is smaller on screen than "r4" in many cases, though
// that is impossible.
fn project(m: &Matrix4f, p: &Point3f) -> Point3f {
    let d = 1. / (m[0][3] * p.x + m[1][3] * p.y + m[2][3] * p.z + m[3][3]);
    Point3f::new(
        (m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z + m[3][0]) * d,
        (m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z + m[3][1]) * d,
        (m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z + m[3][2]) * d,
    )
}

fn size_in_pixels(bounding_cube: &Cube, matrix: &Matrix4f, width: i32, height: i32) -> Vector2f {
    // z is unused here.
    let min = bounding_cube.min();
    let max = bounding_cube.max();
    let mut rv = Aabb3f::zero();
    for p in &[
        Point3f::new(min.x, min.y, min.z),
        Point3f::new(max.x, min.y, min.z),
        Point3f::new(min.x, max.y, min.z),
        Point3f::new(max.x, max.y, min.z),
        Point3f::new(min.x, min.y, max.z),
        Point3f::new(max.x, min.y, max.z),
        Point3f::new(min.x, max.y, max.z),
        Point3f::new(max.x, max.y, max.z),
    ] {
        rv = rv.grow(project(matrix, p));
    }
    Vector2f::new(
        (rv.max().x - rv.min().x) * (width as f32) / 2.,
        (rv.max().y - rv.min().y) * (height as f32) / 2.,
    )
}

#[derive(Debug)]
pub struct OnDiskOctree {
    directory: PathBuf,
    // Maps from node id to NodeMeta: bounding_box, number of points, PositionEncoding.
    nodes: HashMap<NodeId, NodeMeta>,
    bounding_box: Aabb3f,
}

#[derive(Debug, PartialEq)]
pub enum UseLod {
    No,
    Yes,
}

pub trait Octree: Send + Sync {
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4f,
        width: i32,
        height: i32,
        use_lod: UseLod,
    ) -> Vec<VisibleNode>;

    fn get_node_data(&self, node_id: &NodeId, level_of_detail: i32) -> Result<NodeData>;
}

#[derive(Debug)]
pub struct NodeData {
    pub meta: node::NodeMeta,
    pub position: Vec<u8>,
    pub color: Vec<u8>,
}

impl OnDiskOctree {
    pub fn new<P: AsRef<Path>>(directory: P) -> Result<Self> {
        let directory = directory.as_ref();
        // We used to use JSON earlier.
        if directory.join("meta.json").exists() {
            return Err(ErrorKind::InvalidVersion(3).into());
        }

        let mut meta = {
            let mut data = Vec::new();
            File::open(&directory.join("meta.pb"))?.read_to_end(&mut data)?;
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
                .chain_err(|| "Could not parse meta.pb")?
        };

        if meta.version != CURRENT_VERSION {
            return Err(ErrorKind::InvalidVersion(meta.version).into());
        }

        let bounding_box = {
            let bounding_box = meta.bounding_box.clone().unwrap();
            let min = bounding_box.min.unwrap();
            let max = bounding_box.max.unwrap();
            Aabb3f::new(
                Point3f::new(min.x, min.y, min.z),
                Point3f::new(max.x, max.y, max.z),
            )
        };

        let mut nodes = HashMap::new();
        for meta in meta.mut_nodes().iter() {
            nodes.insert(
                NodeId::from_str(&meta.id), 
                NodeMeta {
                    num_points: meta.num_points,
                    position_encoding: PositionEncoding::from_proto(meta.position_encoding)?,
                    bounding_cube: {
                        let proto = meta.bounding_cube.clone().unwrap();
                        let min = proto.min.unwrap();
                        Cube::new(Point3f::new(min.x, min.y, min.z), proto.edge_length)
                    },
                }
            );
        }

        Ok(OnDiskOctree {
            directory: directory.into(),
            nodes: nodes,
            bounding_box: bounding_box,
        })
    }
}

impl Octree for OnDiskOctree {
    fn get_visible_nodes(
        &self,
        projection_matrix: &Matrix4f,
        width: i32,
        height: i32,
        use_lod: UseLod,
    ) -> Vec<VisibleNode> {
        let frustum = Frustum::from_matrix4(*projection_matrix).unwrap();
        let mut open = vec![
            Node::root_with_bounding_cube(Cube::bounding(&self.bounding_box)),
        ];

        let mut visible = Vec::new();
        while !open.is_empty() {
            let node_to_explore = open.pop().unwrap();
            let meta = self.nodes.get(&node_to_explore.id);
            if meta.is_none() || frustum.contains(&node_to_explore.bounding_cube.to_aabb3()) == Relation::Out {
                continue;
            }
            let num_points = meta.unwrap().num_points;

            let pixels = size_in_pixels(
                &node_to_explore.bounding_cube,
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
                continue;
            }

            let level_of_detail = match use_lod {
                UseLod::No => 1,
                UseLod::Yes => {
                    // Simple heuristic: keep one point for every four pixels.
                    cmp::max(1, ((num_points as f32) / (visible_pixels / 4.)) as i32)
                }
            };

            for child_index in 0..8 {
                open.push(node_to_explore.get_child(ChildIndex::from_u8(child_index)))
            }

            visible.push(VisibleNode {
                id: node_to_explore.id,
                level_of_detail: level_of_detail,
                pixels: pixels,
            });
        }

        visible.sort_by(|a, b| {
            let size_a = a.pixels.x * a.pixels.y;
            let size_b = b.pixels.x * b.pixels.y;
            size_b.partial_cmp(&size_a).unwrap()
        });
        visible
    }

    fn get_node_data(&self, node_id: &NodeId, level_of_detail: i32) -> Result<NodeData> {
        let stem = node_id.get_stem(&self.directory);
        let meta = {
            let mut meta = self.nodes[node_id].clone();
            meta.num_points = meta.num_points_for_level_of_detail(level_of_detail);
            meta
        };

        // TODO(hrapp): If we'd randomize the points while writing, we could just read the
        // first N points instead of reading everything and skipping over a few.
        let position = {
            let mut xyz_reader =
                BufReader::new(File::open(&stem.with_extension(node::POSITION_EXT))?);
            let mut all_data = Vec::new();
            xyz_reader
                .read_to_end(&mut all_data)
                .chain_err(|| "Could not read position")?;

            let mut position = Vec::new();
            let bytes_per_point = meta.position_encoding.bytes_per_coordinate() * 3;
            position.reserve(bytes_per_point * meta.num_points as usize);
            for (idx, chunk) in all_data.chunks(bytes_per_point).enumerate() {
                if idx % level_of_detail as usize != 0 {
                    continue;
                }
                position.extend(chunk);
            }
            position
        };

        let color = {
            let mut rgb_reader = BufReader::new(File::open(&stem.with_extension(node::COLOR_EXT))
                .chain_err(|| "Could not read color")?);
            let mut all_data = Vec::new();
            rgb_reader
                .read_to_end(&mut all_data)
                .chain_err(|| "Could not read color")?;
            let mut color = Vec::new();
            color.reserve(3 * meta.num_points as usize);
            for (idx, chunk) in all_data.chunks(3).enumerate() {
                if idx % level_of_detail as usize != 0 {
                    continue;
                }
                color.extend(chunk);
            }
            color
        };

        Ok(NodeData {
            position: position,
            color: color,
            meta: meta,
        })
    }
}
