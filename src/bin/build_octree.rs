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
extern crate pbr;
#[macro_use]
extern crate json;

use point_viewer::Point;
use point_viewer::math::{Vector3f, CuboidLike, Cube, Cuboid};
use point_viewer::octree;
use point_viewer::pts::PtsPointStream;

use byteorder::{LittleEndian, WriteBytesExt};
use pbr::ProgressBar;
use scoped_pool::{Scope, Pool};
use std::collections::HashSet;
use std::fs::{self, File};
use std::io::{BufWriter, Write, Stdout};
use std::path::{Path, PathBuf};
use std::sync::mpsc;

const UPDATE_COUNT: i64 = 100000;
const MAX_POINTS_PER_NODE: i64 = 100000;

#[derive(Debug)]
struct NodeWriter {
    writer: BufWriter<File>,
    path: PathBuf,
    num_points: i64,
    bounding_cube: Cube,
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
    fn new(path: PathBuf, bounding_cube: Cube, resolution: f64) -> Self {
        let size = bounding_cube.size();
        // NOCOM(#hrapp): use this to fix code encode the (x,y,z).
        println!("#hrapp size: {:#?}", size);
        let cnt = size.x as f64 / resolution;
        println!("#hrapp cnt: {:#?}", cnt);
        println!("#hrapp cnt.log2(): {:#?}", cnt.log2());

        NodeWriter {
            writer: BufWriter::new(File::create(&path).unwrap()),
            path: path,
            num_points: 0,
            bounding_cube: bounding_cube,
        }
    }

    pub fn write(&mut self, p: &Point) {
        // NOCOM(#hrapp): bring back and make this pass.
        // assert!(self.bounding_cube.contains(&p.position));
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
    bounding_cube: Cube,
    num_points: i64,
}

fn split<PointIterator: Iterator<Item = Point>>(output_directory: &Path,
                                                resolution: f64,
                                                name: &str,
                                                bounding_cube: &Cube,
                                                stream: PointIterator)
                                                -> Vec<SplittedNode> {
    let mut children: Vec<Option<NodeWriter>> = vec![None, None, None, None, None, None, None,
                                                     None];
    match stream.size_hint().1 {
        Some(size) => {
            println!("Splitting {} which has {} points ({:.2}x MAX_POINTS_PER_NODE).",
                     name,
                     size,
                     size as f64 / MAX_POINTS_PER_NODE as f64)
        }
        None => println!("Splitting {} which has an unknown number of points.", name),
    };

    for p in stream {
        let child_index = get_child_index(&bounding_cube, &p.position);
        if children[child_index as usize].is_none() {
            children[child_index as usize] =
                Some(NodeWriter::new(
                        octree::node_path(output_directory,
                                               &octree::child_node_name(name, child_index as u8)),
                        octree::get_child_bounding_cube(&bounding_cube, child_index),
                        resolution));
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
            bounding_cube: octree::get_child_bounding_cube(&bounding_cube, child_index as u8),
        });
    }
    rv
}

fn get_child_index(bounding_cube: &Cube, v: &Vector3f) -> u8 {
    let center = bounding_cube.center();
    let gt_x = v.x > center.x;
    let gt_y = v.y > center.y;
    let gt_z = v.z > center.z;
    (gt_x as u8) << 2 | (gt_y as u8) << 1 | gt_z as u8
}

struct NodeToCreateBySubsamplingChildren {
    name: String,
    bounding_cube: Cube,
}

fn split_node<'a, 'b: 'a, PointIterator>(scope: &Scope<'a>,
                                         output_directory: &'b Path,
                                         resolution: f64,
                                         node: SplittedNode,
                                         stream: PointIterator,
                                         leaf_nodes_sender: mpsc::Sender<NodeToCreateBySubsamplingChildren>)
    where PointIterator: Iterator<Item = Point>
{
    let children = split(output_directory, resolution, &node.name, &node.bounding_cube, stream);
    let (leaf_nodes, split_nodes): (Vec<_>, Vec<_>) = children.into_iter()
        .partition(|n| n.num_points < MAX_POINTS_PER_NODE);

    for child in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let stream = octree::PointStream::from_blob(&octree::node_path(output_directory,
                                                                           &child.name));
            split_node(scope,
                       output_directory,
                       resolution,
                       child,
                       stream,
                       leaf_nodes_sender_clone);
        });
    }

    for node in leaf_nodes {
        leaf_nodes_sender.send(NodeToCreateBySubsamplingChildren {
            name: node.name,
            bounding_cube: node.bounding_cube,
        }).unwrap();
    }
}

fn subsample_children_into(output_directory: &Path, node: NodeToCreateBySubsamplingChildren, resolution: f64) {
    let mut parent = NodeWriter::new(
        octree::node_path(output_directory, &node.name),
        node.bounding_cube.clone(),
        resolution);

    println!("Creating {} from subsampling children.", &node.name);
    for i in 0..8 {
        let child_name = octree::child_node_name(&node.name, i);
        let path = octree::node_path(output_directory, &child_name);
        if !path.exists() {
            continue;
        }
        let points: Vec<_> = octree::PointStream::from_blob(&path).collect();
        let mut child = NodeWriter::new(
            octree::node_path(output_directory, &child_name),
            octree::get_child_bounding_cube(&node.bounding_cube, i as u8),
            resolution);
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

fn make_stream(input: &InputFile)
               -> (Box<Iterator<Item = Point>>, Option<pbr::ProgressBar<Stdout>>) {
    let stream: Box<Iterator<Item = Point>> = match *input {
        InputFile::Ply(ref filename) => Box::new(octree::PointStream::from_ply(filename)),
        InputFile::Pts(ref filename) => Box::new(PtsPointStream::new(filename)),
    };

    let progress_bar = match stream.size_hint().1 {
        Some(size) => Some(ProgressBar::new(size as u64)),
        None => None,
    };
    (stream, progress_bar)
}

/// Returns the bounding_cube and the number of the points in 'input'.
fn find_bounding_cube(input: &InputFile) -> (Cube, i64) {
    let mut num_points = 0i64;
    let mut bounding_cube = Cuboid::new();
    let (stream, mut progress_bar) = make_stream(input);
    if let Some(ref mut progress_bar) = progress_bar {
        progress_bar.message("Determining bounding box: ");
    };

    for p in stream {
        bounding_cube.update(&p.position);
        num_points += 1;
        if num_points % UPDATE_COUNT == 0 {
            if let Some(ref mut progress_bar) = progress_bar {
                progress_bar.add(UPDATE_COUNT as u64);
            }
        }
    }
    progress_bar.map(|mut f| f.finish());
    (bounding_cube.to_cube(), num_points)
}

fn main() {
    let matches = clap::App::new("build_octree")
        .args(&[clap::Arg::with_name("output_directory")
                    .help("Output directory to write the octree into.")
                    .long("output_directory")
                    .required(true)
                    .takes_value(true),
                clap::Arg::with_name("resolution")
                    .help("Minimal precision that this point cloud should have. This decides on the number of bits used to encode each node.")
                    .long("resolution")
                    .default_value("0.001"),
                clap::Arg::with_name("input")
                    .help("PLY/PTS file to parse for the points.")
                    .index(1)
                    .required(true)])
        .get_matches();

    let output_directory = &PathBuf::from(matches.value_of("output_directory").unwrap());
    let resolution = matches.value_of("resolution").unwrap().parse::<f64>().expect("resolution could not be parsed as float.");

    let input = {
        let filename = PathBuf::from(matches.value_of("input").unwrap());
        match filename.extension().and_then(|s| s.to_str()) {
            Some("ply") => InputFile::Ply(filename.clone()),
            Some("pts") => InputFile::Pts(filename.clone()),
            other => panic!("Unknown input file format: {:?}", other),
        }
    };


    let (bounding_cube, num_points) = find_bounding_cube(&input);

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);
    let meta = object!{
        "version" => octree::CURRENT_VERSION,
        "resolution" => resolution,
        "bounding_cube" => object!{
            "min_x" => bounding_cube.min().x,
            "min_y" => bounding_cube.min().y,
            "min_z" => bounding_cube.min().z,
            "edge_length" => bounding_cube.edge_length()
        }
    };
    File::create(&output_directory.join("meta.json"))
        .unwrap()
        .write_all(&meta.pretty(4).as_bytes())
        .unwrap();

    println!("Creating octree structure.");
    let pool = Pool::new(10);

    let (leaf_nodes_sender, leaf_nodes_receiver) = mpsc::channel();
    pool.scoped(move |scope| {
        let (root_stream, _) = make_stream(&input);
        let root = SplittedNode {
            name: "r".into(),
            bounding_cube: bounding_cube,
            num_points: num_points,
        };
        split_node(scope,
                   output_directory,
                   resolution,
                   root,
                   root_stream,
                   leaf_nodes_sender.clone());
    });

    let mut leaf_nodes: Vec<_> = leaf_nodes_receiver.into_iter().collect();

    // Sort by length of node name, longest first. A node with the same length name as another are
    // on the same tree level and can be subsampled in parallel.
    leaf_nodes.sort_by(|a, b| b.name.len().cmp(&a.name.len()));

    while !leaf_nodes.is_empty() {
        let current_length = leaf_nodes[0].name.len();
        let res = leaf_nodes.into_iter().partition(|n| n.name.len() == current_length);
        leaf_nodes = res.1;

        let mut parent_names = HashSet::new();
        let mut subsample_nodes = Vec::new();
        for node in res.0 {
            let parent_name = octree::parent_node_name(&node.name);
            if parent_name.is_empty() || parent_names.contains(parent_name) {
                continue;
            }
            parent_names.insert(parent_name.to_string());

            let parent_bounding_cube = octree::get_parent_bounding_cube(&node.bounding_cube, octree::get_child_index(&node.name));

            let grand_parent = octree::parent_node_name(&parent_name);
            if !grand_parent.is_empty() {
                let grand_parent_bounding_cube =
                    octree::get_parent_bounding_cube(&parent_bounding_cube,
                                                    octree::get_child_index(&parent_name));
                leaf_nodes.push(NodeToCreateBySubsamplingChildren {
                    name: grand_parent.to_string(),
                    bounding_cube: grand_parent_bounding_cube,
                })
            }

            subsample_nodes.push(NodeToCreateBySubsamplingChildren {
                name: parent_name.to_string(),
                bounding_cube: parent_bounding_cube,
            });
        }

        pool.scoped(move |scope| {
            for node in subsample_nodes {
                scope.execute(move || {
                    subsample_children_into(output_directory, node, resolution);
                });
            }
        });
    }
}
