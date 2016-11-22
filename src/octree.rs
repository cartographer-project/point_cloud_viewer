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
use json;
use math::{BoundingBox, Matrix4f, Vector3f, Vector2f, Frustum};
use {Point, ply};
use std::cmp;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{Read, BufReader};
use std::path::{Path, PathBuf};
use walkdir;

pub struct PointStream {
    data: BufReader<File>,
    num_points_read: i64,
    pub num_total_points: i64,
}

impl Iterator for PointStream {
    type Item = Point;

    fn next(&mut self) -> Option<Self::Item> {
        if self.num_points_read >= self.num_total_points {
            return None;
        }
        let point = ply::read_point(&mut self.data);
        self.num_points_read += 1;
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.num_total_points as usize, Some(self.num_total_points as usize))
    }
}

impl PointStream {
    pub fn from_ply(ply_file: &Path) -> Self {
        let (file, num_total_points) = ply::open(ply_file);
        Self::from_reader_and_count(BufReader::new(file), num_total_points)
    }

    pub fn from_blob(blob_path: &Path) -> Self {
        let num_total_points = fs::metadata(blob_path).unwrap().len() as i64 / 15;
        let file = File::open(blob_path).unwrap();
        Self::from_reader_and_count(BufReader::new(file), num_total_points)
    }

    fn from_reader_and_count(data: BufReader<File>, num_total_points: i64) -> Self {
        PointStream {
            data: data,
            num_total_points: num_total_points,
            num_points_read: 0,
        }
    }
}

pub fn node_path(directory: &Path, name: &str) -> PathBuf {
    directory.join(name)
}

pub fn parent_node_name(name: &str) -> &str {
    if name.is_empty() {
        name
    } else {
        name.split_at(name.len() - 1).0
    }
}

pub fn child_node_name(parent: &str, child_index: u8) -> String {
    assert!(child_index < 8);
    format!("{}{}", parent, child_index)
}

pub fn get_child_bounding_box(parent: &BoundingBox, child_index: u8) -> BoundingBox {
    assert!(child_index < 8);

    let mut bounding_box = parent.clone();
    let half_size_x = (bounding_box.max.x - bounding_box.min.x) / 2.;
    let half_size_y = (bounding_box.max.y - bounding_box.min.y) / 2.;
    let half_size_z = (bounding_box.max.z - bounding_box.min.z) / 2.;

    if (child_index & 0b001) != 0 {
        bounding_box.min.z += half_size_z;
    } else {
        bounding_box.max.z -= half_size_z;
    }

    if (child_index & 0b010) != 0 {
        bounding_box.min.y += half_size_y;
    } else {
        bounding_box.max.y -= half_size_y;
    }

    if (child_index & 0b100) != 0 {
        bounding_box.min.x += half_size_x;
    } else {
        bounding_box.max.x -= half_size_x;
    }
    bounding_box
}

#[derive(Debug)]
struct NodeToExplore {
    name: String,
    bounding_box: BoundingBox,
}

#[derive(Debug)]
pub struct VisibleNode {
    pub name: String,
    pub level_of_detail: i32,
    pixels: Vector2f,
}

#[derive(Debug)]
pub struct NodesToBlob {
    pub name: String,
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

fn size_in_pixels(bb: &BoundingBox, matrix: &Matrix4f, width: i32, height: i32) -> Vector2f {
    // z is unused here.
    let mut rv = BoundingBox::new();
    for p in &[Vector3f::new(bb.min.x, bb.min.y, bb.min.z),
               Vector3f::new(bb.max.x, bb.min.y, bb.min.z),
               Vector3f::new(bb.min.x, bb.max.y, bb.min.z),
               Vector3f::new(bb.max.x, bb.max.y, bb.min.z),
               Vector3f::new(bb.min.x, bb.min.y, bb.max.z),
               Vector3f::new(bb.max.x, bb.min.y, bb.max.z),
               Vector3f::new(bb.min.x, bb.max.y, bb.max.z),
               Vector3f::new(bb.max.x, bb.max.y, bb.max.z)] {
        rv.update(&project(matrix, &p));
    }
    Vector2f::new((rv.max.x - rv.min.x) * (width as f32) / 2.,
                  (rv.max.y - rv.min.y) * (height as f32) / 2.)
}

#[derive(Debug)]
pub struct Octree {
    directory: PathBuf,
    // Maps from node name to number of points.
    nodes: HashMap<String, u64>,
    bounding_box: BoundingBox,
}

#[derive(Debug)]
pub enum UseLod {
    No,
    Yes,
}

impl Octree {
    pub fn new(directory: PathBuf) -> Self {
        let bounding_box = {
            let mut content = String::new();
            File::open(&directory.join("meta.json")).unwrap().read_to_string(&mut content).unwrap();
            let meta = json::parse(&content).unwrap();
            BoundingBox {
                min: Vector3f::new(meta["bounding_box"]["min_x"].as_f32().unwrap(),
                                   meta["bounding_box"]["min_y"].as_f32().unwrap(),
                                   meta["bounding_box"]["min_z"].as_f32().unwrap()),
                max: Vector3f::new(meta["bounding_box"]["max_x"].as_f32().unwrap(),
                                   meta["bounding_box"]["max_y"].as_f32().unwrap(),
                                   meta["bounding_box"]["max_z"].as_f32().unwrap()),
            }
        };

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
            nodes.insert(file_name.to_string(), num_points);
        }

        Octree {
            directory: directory.into(),
            nodes: nodes,
            bounding_box: bounding_box,
        }
    }

    pub fn get_visible_nodes(&self,
                             projection_matrix: &Matrix4f,
                             width: i32,
                             height: i32,
                             use_lod: UseLod)
                             -> Vec<VisibleNode> {
        let frustum = Frustum::from_matrix(projection_matrix);
        let mut open = vec![NodeToExplore {
                                name: "r".into(),
                                bounding_box: self.bounding_box.clone(),
                            }];

        let mut visible = Vec::new();
        while !open.is_empty() {
            let node_to_explore = open.pop().unwrap();
            let maybe_num_points = self.nodes.get(&node_to_explore.name);
            if maybe_num_points.is_none() || !frustum.intersects(&node_to_explore.bounding_box) {
                continue;
            }
            let num_points = *maybe_num_points.unwrap();

            let pixels = size_in_pixels(&node_to_explore.bounding_box,
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
                open.push(NodeToExplore {
                    name: child_node_name(&node_to_explore.name, child_index),
                    bounding_box: get_child_bounding_box(&node_to_explore.bounding_box,
                                                         child_index as u8),
                })
            }

            visible.push(VisibleNode {
                name: node_to_explore.name,
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

    pub fn get_nodes_as_binary_blob(&self, nodes: &[NodesToBlob]) -> (usize, Vec<u8>) {
        let mut num_points = 0;
        let mut rv = Vec::new();
        for node in nodes {
            let points: Vec<_> = PointStream::from_blob(&node_path(&self.directory, &node.name))
                .collect();
            let num_points_for_lod =
                (points.len() as f32 / node.level_of_detail as f32).ceil() as usize;

            num_points += num_points_for_lod;
            let mut pos = rv.len();
            rv.resize(pos + 4 + 24 * num_points_for_lod, 0u8);
            LittleEndian::write_u32(&mut rv[pos..], num_points_for_lod as u32 * 24);
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
                LittleEndian::write_f32(&mut rv[pos..], p.r as f32 / 255.);
                pos += 4;
                LittleEndian::write_f32(&mut rv[pos..], p.g as f32 / 255.);
                pos += 4;
                LittleEndian::write_f32(&mut rv[pos..], p.b as f32 / 255.);
                pos += 4;
            }
        }
        assert_eq!(4 * nodes.len() + 24 * num_points, rv.len());
        (num_points, rv)
    }
}

#[cfg(test)]
mod tests {
    use super::parent_node_name;

    #[test]
    fn test_parent_node_name() {
        assert_eq!("r12345", parent_node_name("r123456"));
    }
}
