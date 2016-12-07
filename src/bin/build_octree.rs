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

use point_viewer::errors::*;
use point_viewer::math::{CuboidLike, Cube, Cuboid};
use point_viewer::octree;
use point_viewer::Point;
use point_viewer::point_stream::PointStream;
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
    fn new(output_directory: &Path, node: &octree::Node, resolution: f64) -> Self {
        // TODO: use this to fix-code encode the (x,y,z).
        // let required_bytes = octree::required_bytes(bounding_cube, resolution);

        let path = node.id.get_on_disk_path(output_directory);
        NodeWriter {
            writer: BufWriter::new(File::create(&path).unwrap()),
            path: path,
            num_points: 0,
        }
    }

    pub fn write(&mut self, p: &Point) {
        // Note that due to floating point rounding errors while calculating bounding boxes, it
        // could be here that 'p' is not quite the bounding box of our node.
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
    node: octree::Node,
    num_points: i64,
}

fn split<PointIterator: Iterator<Item = Point>>(output_directory: &Path,
                                                resolution: f64,
                                                node: &octree::Node,
                                                stream: PointIterator)
                                                -> Vec<SplittedNode> {
    let mut children: Vec<Option<NodeWriter>> = vec![None, None, None, None, None, None, None,
                                                     None];
    match stream.size_hint().1 {
        Some(size) => {
            println!("Splitting {} which has {} points ({:.2}x MAX_POINTS_PER_NODE).",
                     node.id,
                     size,
                     size as f64 / MAX_POINTS_PER_NODE as f64)
        }
        None => {
            println!("Splitting {} which has an unknown number of points.",
                     node.id)
        }
    };

    for p in stream {
        let child_index = node.get_child_id_containing_point(&p.position);
        let array_index = child_index.as_u8() as usize;
        if children[array_index].is_none() {
            children[array_index] =
                Some(NodeWriter::new(output_directory,
                                     &node.get_child(child_index),
                                     resolution));
        }
        children[array_index].as_mut().unwrap().write(&p);
    }

    // Remove the node file on disk. This is only save some disk space during processing - all
    // nodes will be rewritten by subsampling the children in the second step anyways. We also
    // ignore file removing error. For example, we never write out the root, so it cannot be
    // removed.
    let _ = fs::remove_file(node.id.get_on_disk_path(output_directory));

    let mut rv = Vec::new();
    for (child_index, c) in children.into_iter().enumerate() {
        if c.is_none() {
            continue;
        }
        let c = c.unwrap();

        rv.push(SplittedNode {
            node: node.get_child(octree::ChildIndex::from_u8(child_index as u8)),
            num_points: c.num_points,
        });
    }
    rv
}

fn split_node<'a, 'b, Points>(scope: &Scope<'a>,
                              output_directory: &'b Path,
                              resolution: f64,
                              splitted_node: SplittedNode,
                              stream: Points,
                              leaf_nodes_sender: mpsc::Sender<octree::Node>)
    where 'b: 'a,
          Points: Iterator<Item = Point>
{
    let children = split(output_directory, resolution, &splitted_node.node, stream);
    let (leaf_nodes, split_nodes): (Vec<_>, Vec<_>) = children.into_iter()
        .partition(|n| n.num_points < MAX_POINTS_PER_NODE);

    for child in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let blob_path = child.node.id.get_on_disk_path(output_directory);
            let stream = PointStream::from_blob(&blob_path)
                .chain_err(|| format!("Could not open {:?}", blob_path))
                .unwrap();
            split_node(scope,
                       output_directory,
                       resolution,
                       child,
                       stream,
                       leaf_nodes_sender_clone);
        });
    }

    for splitted_node in leaf_nodes {
        leaf_nodes_sender.send(splitted_node.node).unwrap();
    }
}

fn subsample_children_into(output_directory: &Path,
                           node: octree::Node,
                           resolution: f64)
                           -> Result<()> {
    let mut parent_writer = NodeWriter::new(output_directory, &node, resolution);
    println!("Creating {} from subsampling children.", &node.id);
    for i in 0..8 {
        let child = node.get_child(octree::ChildIndex::from_u8(i));
        let path = child.id.get_on_disk_path(output_directory);
        if !path.exists() {
            continue;
        }
        let points: Vec<_> = PointStream::from_blob(&path)
            .chain_err(|| format!("Could not find {:?}", path))?
            .collect();
        let mut child_writer = NodeWriter::new(output_directory, &child, resolution);
        for (idx, p) in points.into_iter().enumerate() {
            if idx % 8 == 0 {
                parent_writer.write(&p);
            } else {
                child_writer.write(&p);
            }
        }
    }
    Ok(())
}

#[derive(Debug)]
enum InputFile {
    Ply(PathBuf),
    Pts(PathBuf),
}

fn make_stream(input: &InputFile)
               -> (Box<Iterator<Item = Point>>, Option<pbr::ProgressBar<Stdout>>) {
    let stream: Box<Iterator<Item = Point>> = match *input {
        InputFile::Ply(ref filename) => Box::new(PointStream::from_ply(filename)),
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
                    .help("Minimal precision that this point cloud should have. This decides \
                           on the number of bits used to encode each node.")
                    .long("resolution")
                    .default_value("0.001"),
                clap::Arg::with_name("input")
                    .help("PLY/PTS file to parse for the points.")
                    .index(1)
                    .required(true)])
        .get_matches();

    let output_directory = &PathBuf::from(matches.value_of("output_directory").unwrap());
    let resolution = matches.value_of("resolution")
        .unwrap()
        .parse::<f64>()
        .expect("resolution could not be parsed as float.");

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
            node: octree::Node::root_with_bounding_cube(bounding_cube),
            num_points: num_points,
        };
        split_node(scope,
                   output_directory,
                   resolution,
                   root,
                   root_stream,
                   leaf_nodes_sender.clone());
    });

    let mut deepest_level = 0usize;
    let mut leaf_nodes = Vec::<octree::Node>::new();
    for leaf_node in leaf_nodes_receiver.into_iter() {
        deepest_level = std::cmp::max(deepest_level, leaf_node.level());
        leaf_nodes.push(leaf_node);
    }

    // We start on the deepest level and work our way up the tree.
    for current_level in (0..deepest_level + 1).rev() {
        // All nodes on the same level can be subsampled in parallel.
        let res = leaf_nodes.into_iter().partition(|n| n.level() == current_level);
        leaf_nodes = res.1;

        let mut parent_ids = HashSet::new();
        let mut subsample_nodes = Vec::new();
        for node in res.0 {
            let maybe_parent = node.parent();
            if maybe_parent.is_none() {
                continue;
            }
            let parent = maybe_parent.unwrap();
            if parent_ids.contains(&parent.id) {
                continue;
            }
            parent_ids.insert(parent.id.clone());

            let maybe_grand_parent = parent.parent();
            if let Some(grand_parent) = maybe_grand_parent {
                leaf_nodes.push(grand_parent);
            }
            subsample_nodes.push(parent);
        }

        pool.scoped(move |scope| {
            for node in subsample_nodes {
                scope.execute(move || {
                    subsample_children_into(output_directory, node, resolution).unwrap();
                });
            }
        });
    }
}
