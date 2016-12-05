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

#[macro_use]
extern crate nom;
extern crate clap;
extern crate byteorder;
extern crate point_viewer;
extern crate scoped_pool;
#[macro_use]
extern crate json;

use point_viewer::Point;
use point_viewer::math::{Vector3f, BoundingBox};
use point_viewer::octree;
use point_viewer::pts::PtsPointStream;

use std::collections::HashSet;
use scoped_pool::{Scope, Pool};
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};
use byteorder::{LittleEndian, WriteBytesExt};
use std::sync::mpsc;


#[derive(Debug)]
struct NodeWriter {
    writer: BufWriter<File>,
    path: PathBuf,
    num_points: i64,
}

impl Drop for NodeWriter {
    fn drop(&mut self) {
        // If we did not write anything into this node, it should not exist.
        if self.num_points == 0 {
            // We are ignoring deletion errors here in case the file is already gone.
            let _ = fs::remove_file(&self.path);
        }

        // TODO(hrapp): Add some sanity checks that we do not have nodes with ridiculously low
        // amount of points laying around?
    }
}

impl NodeWriter {
    fn new(path: PathBuf) -> Self {
        NodeWriter {
            writer: BufWriter::new(File::create(&path).unwrap()),
            path: path,
            num_points: 0,
        }
    }

    pub fn write(&mut self, p: &Point) {
        self.writer.write_f32::<LittleEndian>(p.position.x).unwrap();
        self.writer.write_f32::<LittleEndian>(p.position.y).unwrap();
        self.writer.write_f32::<LittleEndian>(p.position.z).unwrap();
        self.writer.write_u8(p.r).unwrap();
        self.writer.write_u8(p.g).unwrap();
        self.writer.write_u8(p.b).unwrap();
        self.num_points += 1;
    }
}

struct SplittedNode {
    name: String,
    bounding_box: BoundingBox,
    num_points: i64,
}

fn split<PointIterator: Iterator<Item = Point>>(output_directory: &Path,
                                                name: &str,
                                                bounding_box: &BoundingBox,
                                                stream: PointIterator)
                                                -> Vec<SplittedNode> {
    let mut children: Vec<Option<NodeWriter>> = vec![None, None, None, None, None, None, None,
                                                     None];
    println!("Splitting {}...", name);
    for p in stream {
        let child_index = get_child_index(&bounding_box, &p.position);
        if children[child_index as usize].is_none() {
            children[child_index as usize] =
                Some(NodeWriter::new(octree::node_path(output_directory,
                                               &octree::child_node_name(name, child_index as u8))));
        }
        children[child_index as usize].as_mut().unwrap().write(&p);
    }

    // Remove the node file on disk. This is only save some disk space during processing - all
    // nodes will be rewritten by subsampling the children in the second step anyways. We also
    // ignore file removing error. For example, we never write out the root, so it cannot be
    // removed.
    let _ = fs::remove_file(octree::node_path(output_directory, name));
    let mut rv = Vec::new();
    for (child_index, c) in children.into_iter().enumerate() {
        if c.is_none() {
            continue;
        }
        let c = c.unwrap();

        rv.push(SplittedNode {
            name: octree::child_node_name(name, child_index as u8),
            num_points: c.num_points,
            bounding_box: octree::get_child_bounding_box(&bounding_box, child_index as u8),
        });
    }
    rv
}

fn get_child_index(bounding_box: &BoundingBox, v: &Vector3f) -> u8 {
    let center = bounding_box.center();
    let gt_x = v.x > center.x;
    let gt_y = v.y > center.y;
    let gt_z = v.z > center.z;
    (gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8
}

fn split_node<'a, 'b: 'a, PointIterator: Iterator<Item = Point>>(scope: &Scope<'a>,
                                                                 output_directory: &'b Path,
                                                                 node: SplittedNode,
                                                                 stream: PointIterator,
                                                                 tx: mpsc::Sender<String>) {
    let children = split(output_directory, &node.name, &node.bounding_box, stream);
    let (leaf_nodes, split_nodes): (Vec<_>, Vec<_>) = children.into_iter()
        .partition(|n| n.num_points < 100000);

    for node in leaf_nodes {
        tx.send(node.name).unwrap();
    }

    for child in split_nodes {
        let tx_clone = tx.clone();
        scope.recurse(move |scope| {
            let stream = octree::PointStream::from_blob(&octree::node_path(output_directory,
                                                                           &child.name),
                                                        octree::ShowProgress::No);
            split_node(scope, output_directory, child, stream, tx_clone);
        });
    }
}

fn subsample_children_into(output_directory: &Path, node_name: &str) {
    let mut parent = NodeWriter::new(octree::node_path(output_directory, node_name));

    println!("Creating {} from subsampling children...", node_name);
    for i in 0..8 {
        let child_name = octree::child_node_name(node_name, i);
        let path = octree::node_path(output_directory, &child_name);
        if !path.exists() {
            continue;
        }
        let points: Vec<_> = octree::PointStream::from_blob(&path, octree::ShowProgress::No)
            .collect();
        let mut child = NodeWriter::new(octree::node_path(output_directory, &child_name));
        for (idx, p) in points.into_iter().enumerate() {
            if idx % 8 == 0 {
                parent.write(&p);
            } else {
                child.write(&p);
            }
        }

    }
}

#[derive(Debug)]
enum InputFile {
    Ply(PathBuf),
    Pts(PathBuf),
}

fn make_stream(i: &InputFile) -> Box<Iterator<Item = Point>> {
    match *i {
        InputFile::Ply(ref filename) => {
            Box::new(octree::PointStream::from_ply(filename, octree::ShowProgress::Yes))
        }
        InputFile::Pts(ref filename) => Box::new(PtsPointStream::new(filename)),
    }
}

fn main() {
    let matches = clap::App::new("build_octree")
        .args(&[clap::Arg::with_name("output_directory")
                    .help("Output directory to write the octree into.")
                    .long("output_directory")
                    .required(true)
                    .takes_value(true),
                clap::Arg::with_name("input")
                    .help("PLY/PTS file to parse for the points.")
                    .index(1)
                    .required(true)])
        .get_matches();

    let output_directory = &PathBuf::from(matches.value_of("output_directory").unwrap());

    let input = {
        let filename = PathBuf::from(matches.value_of("input").unwrap());
        match filename.extension().and_then(|s| s.to_str()) {
            Some("ply") => InputFile::Ply(filename.clone()),
            Some("pts") => InputFile::Pts(filename.clone()),
            other => panic!("Unknown input file format: {:?}", other),
        }
    };

    println!("Determining bounding box...");
    let mut num_total_points = 0i64;
    let bounding_box = {
        let mut r = BoundingBox::new();
        for p in make_stream(&input) {
            r.update(&p.position);
            num_total_points += 1;
        }
        r.make_cubic();
        r
    };

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);
    let meta = object!{
        "version" => 1,
        "bounding_box" => object!{
            "min_x" => bounding_box.min.x,
            "min_y" => bounding_box.min.y,
            "min_z" => bounding_box.min.z,
            "max_x" => bounding_box.max.x,
            "max_y" => bounding_box.max.y,
            "max_z" => bounding_box.max.z
        }
    };
    File::create(&output_directory.join("meta.json"))
        .unwrap()
        .write_all(&meta.pretty(4).as_bytes())
        .unwrap();

    println!("Creating octree structure.");
    let pool = Pool::new(10);

    let (tx, rx) = mpsc::channel();
    pool.scoped(move |scope| {
        let root_stream = make_stream(&input);
        let root = SplittedNode {
            name: "r".into(),
            bounding_box: bounding_box,
            num_points: num_total_points,
        };
        split_node(scope, output_directory, root, root_stream, tx.clone());
    });

    let mut leaf_nodes: Vec<_> = rx.into_iter().collect();
    // Sort by length of node name, longest first. A node with the same length name as another are
    // on the same tree level and can be subsampled in parallel.
    leaf_nodes.sort_by(|a, b| b.len().cmp(&a.len()));

    while !leaf_nodes.is_empty() {
        let current_length = leaf_nodes[0].len();
        let res = leaf_nodes.into_iter().partition(|n| n.len() == current_length);
        leaf_nodes = res.1;

        let mut parent_names = HashSet::new();
        for node in res.0 {
            let parent_name = octree::parent_node_name(&node);
            if parent_name.is_empty() || parent_names.contains(parent_name) {
                continue;
            }
            parent_names.insert(parent_name.to_string());

            let grand_parent = octree::parent_node_name(&parent_name);
            if !grand_parent.is_empty() {
                leaf_nodes.push(grand_parent.to_string());
            }
        }

        pool.scoped(move |scope| {
            for parent_name in parent_names {
                scope.execute(move || {
                    subsample_children_into(output_directory, &parent_name);
                });
            }
        });
    }
}
