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

use byteorder::{LittleEndian, ByteOrder};
use errors::*;
use json;
use math::{CuboidLike, Cuboid, Cube, Matrix4f, Vector3f, Vector2f, Frustum};
use point_stream::PointStream;
use std::cmp;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::fmt;
use std::result;
use walkdir;

pub const CURRENT_VERSION: i32 = 3;

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
    pub fn get_on_disk_path(&self, directory: &Path) -> PathBuf {
        directory.join(&self.name)
    }

    /// Returns the root node of the octree.
    fn root() -> Self {
        NodeId::from_string("r".to_string())
    }

    /// Returns the NodeId for the corresponding 'child_index'.
    fn get_child_id(&self, child_index: u8) -> Self {
        assert!(child_index < 8);
        let mut child_name = self.name.clone();
        child_name.push((child_index + '0' as u8) as char);
        NodeId::from_string(child_name)
    }

    /// The child index of this node in its parent.
    // TODO(hrapp): Child index should be a newtype or an enum.
    // TODO(hrapp): Should return None if this is the root.
    fn child_index(&self) -> u8 {
        let last_char = self.name.split_at(self.name.len() - 1).1.bytes().last().unwrap() as char;
        match last_char {
            '0' => 0,
            '1' => 1,
            '2' => 2,
            '3' => 3,
            '4' => 4,
            '5' => 5,
            '6' => 6,
            '7' => 7,
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
pub enum BytesPerCoordinate {
    One,
    Two,
    Four,
}

pub fn required_bytes(bounding_cube: &Cube, resolution: f64) -> BytesPerCoordinate {
    let min_bits = (bounding_cube.edge_length() as f64 / resolution).log2() as u32 + 1;
    match min_bits {
        0...8 => BytesPerCoordinate::One,
        9...16 => BytesPerCoordinate::Two,
        _ => BytesPerCoordinate::Four,
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

    pub fn get_child(&self, child_index: u8) -> Node {
        let child_bounding_cube = {
            assert!(child_index < 8);
            let half_edge_length = self.bounding_cube.edge_length() / 2.;
            let mut min = self.bounding_cube.min();
            if (child_index & 0b001) != 0 {
                min.z += half_edge_length;
            }

            if (child_index & 0b010) != 0 {
                min.y += half_edge_length;
            }

            if (child_index & 0b100) != 0 {
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
    pub fn get_child_id_containing_point(&self, v: &Vector3f) -> u8 {
        // This is a bit flawed: it is not guaranteed that 'child_bounding_box.contains(&v)' is true
        // using this calculated index due to floating point precision.
        let center = self.bounding_cube.center();
        let gt_x = v.x > center.x;
        let gt_y = v.y > center.y;
        let gt_z = v.z > center.z;
        (gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8
    }

    // TODO(hrapp): This function could use some testing.
    pub fn parent(&self) -> Option<Node> {
        let maybe_parent_id = self.id.parent_id();
        if maybe_parent_id.is_none() {
            return None;
        }

        let parent_cube = {
            let child_index = self.id.child_index();
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
pub struct VisibleNode {
    pub id: NodeId,
    pub level_of_detail: i32,
    pixels: Vector2f,
}

#[derive(Debug)]
pub struct NodesToBlob {
    pub id: NodeId,
    pub level_of_detail: i32,
}

// TODO(hrapp): something is funky here. "r" is smaller on screen than "r4" in many cases, though
// that is impossible.
fn project(m: &Matrix4f, p: &Vector3f) -> Vector3f {
    let d = 1. / (m[0][3] * p.x + m[1][3] * p.y + m[2][3] * p.z + m[3][3]);
    Vector3f::new((m[0][0] * p.x + m[1][0] * p.y + m[2][0] * p.z + m[3][0]) * d,
                  (m[0][1] * p.x + m[1][1] * p.y + m[2][1] * p.z + m[3][1]) * d,
                  (m[0][2] * p.x + m[1][2] * p.y + m[2][2] * p.z + m[3][2]) * d)
}

fn size_in_pixels(bounding_cube: &Cube, matrix: &Matrix4f, width: i32, height: i32) -> Vector2f {
    // z is unused here.
    let min = bounding_cube.min();
    let max = bounding_cube.max();
    let mut rv = Cuboid::new();
    for p in &[Vector3f::new(min.x, min.y, min.z),
               Vector3f::new(max.x, min.y, min.z),
               Vector3f::new(min.x, max.y, min.z),
               Vector3f::new(max.x, max.y, min.z),
               Vector3f::new(min.x, min.y, max.z),
               Vector3f::new(max.x, min.y, max.z),
               Vector3f::new(min.x, max.y, max.z),
               Vector3f::new(max.x, max.y, max.z)] {
        rv.update(&project(matrix, &p));
    }
    Vector2f::new((rv.max().x - rv.min().x) * (width as f32) / 2.,
                  (rv.max().y - rv.min().y) * (height as f32) / 2.)
}

#[derive(Debug)]
pub struct Octree {
    directory: PathBuf,
    // Maps from node id to number of points.
    nodes: HashMap<NodeId, u64>,
    bounding_cube: Cube,
}

#[derive(Debug)]
pub enum UseLod {
    No,
    Yes,
}

impl Octree {
    pub fn new(directory: PathBuf) -> Result<Self> {
        let meta = {
            let mut content = String::new();
            File::open(&directory.join("meta.json"))?.read_to_string(&mut content)?;
            json::parse(&content)?
        };

        match meta["version"].as_i32() {
            None => return Err(ErrorKind::InvalidVersion(-1).into()),
            Some(v) if v != CURRENT_VERSION => return Err(ErrorKind::InvalidVersion(v).into()),
            _ => (), // Correct version.
        }

        let bounding_cube =
            Cube::new(Vector3f::new(meta["bounding_cube"]["min_x"].as_f32().unwrap(),
                                    meta["bounding_cube"]["min_y"].as_f32().unwrap(),
                                    meta["bounding_cube"]["min_z"].as_f32().unwrap()),
                      meta["bounding_cube"]["edge_length"].as_f32().unwrap());

        let mut nodes = HashMap::new();
        for entry in walkdir::WalkDir::new(&directory).into_iter().filter_map(|e| e.ok()) {
            let path = entry.path();
            if path.file_name().is_none() {
                continue;
            }
            let file_name = path.file_name().unwrap().to_string_lossy();
            if !file_name.starts_with("r") {
                continue;
            }
            let num_points = fs::metadata(path).unwrap().len() / 15;
            nodes.insert(NodeId::from_string(file_name.to_string()), num_points);
        }

        Ok(Octree {
            directory: directory.into(),
            nodes: nodes,
            bounding_cube: bounding_cube,
        })
    }

    pub fn get_visible_nodes(&self,
                             projection_matrix: &Matrix4f,
                             width: i32,
                             height: i32,
                             use_lod: UseLod)
                             -> Vec<VisibleNode> {
        let frustum = Frustum::from_matrix(projection_matrix);
        let mut open = vec![Node {
                                id: NodeId::root(),
                                bounding_cube: self.bounding_cube.clone(),
                            }];

        let mut visible = Vec::new();
        while !open.is_empty() {
            let node_to_explore = open.pop().unwrap();
            let maybe_num_points = self.nodes.get(&node_to_explore.id);
            if maybe_num_points.is_none() || !frustum.intersects(&node_to_explore.bounding_cube) {
                continue;
            }
            let num_points = *maybe_num_points.unwrap();

            let pixels = size_in_pixels(&node_to_explore.bounding_cube,
                                        projection_matrix,
                                        width,
                                        height);
            let visible_pixels = pixels.x * pixels.y;
            const MIN_PIXELS_SQ: f32 = 120.;
            const MIN_PIXELS_SIDE: f32 = 12.;
            if pixels.x < MIN_PIXELS_SIDE || pixels.y < MIN_PIXELS_SIDE ||
               visible_pixels < MIN_PIXELS_SQ {
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
                open.push(node_to_explore.get_child(child_index))
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

    pub fn get_nodes_as_binary_blob(&self, nodes: &[NodesToBlob]) -> Result<(usize, Vec<u8>)> {
        const NUM_BYTES_PER_POINT: usize = 4 * 3 + 4;

        let mut num_points = 0;
        let mut rv = Vec::new();
        for node in nodes {
            let points: Vec<_> = PointStream::from_blob(&node.id.get_on_disk_path(&self.directory))
                ?
                .collect();
            let num_points_for_lod =
                (points.len() as f32 / node.level_of_detail as f32).ceil() as usize;

            num_points += num_points_for_lod;
            let mut pos = rv.len();
            rv.resize(pos + 4 + NUM_BYTES_PER_POINT * num_points_for_lod, 0u8);
            LittleEndian::write_u32(&mut rv[pos..],
                                    (num_points_for_lod * NUM_BYTES_PER_POINT) as u32);
            pos += 4;

            // Put positions.
            for (idx, p) in points.iter().enumerate() {
                if idx % node.level_of_detail as usize != 0 {
                    continue;
                }
                LittleEndian::write_f32(&mut rv[pos..], p.position.x);
                pos += 4;
                LittleEndian::write_f32(&mut rv[pos..], p.position.y);
                pos += 4;
                LittleEndian::write_f32(&mut rv[pos..], p.position.z);
                pos += 4;
            }

            // Put colors.
            for (idx, p) in points.iter().enumerate() {
                if idx % node.level_of_detail as usize != 0 {
                    continue;
                }
                rv[pos] = p.r;
                pos += 1;
                rv[pos] = p.g;
                pos += 1;
                rv[pos] = p.b;
                pos += 1;
                rv[pos] = 255;
                pos += 1;
            }
        }
        assert_eq!(4 * nodes.len() + NUM_BYTES_PER_POINT * num_points, rv.len());
        Ok((num_points, rv))
    }
}

#[cfg(test)]
mod tests {
    use super::NodeId;

    #[test]
    fn test_parent_node_name() {
        assert_eq!(Some(NodeId::from_string("r12345".into())),
                   NodeId::from_string("r123456".into()).parent_id());
    }
}
