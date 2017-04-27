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

extern crate byteorder;
extern crate clap;
extern crate pbr;
extern crate point_viewer;
extern crate protobuf;
extern crate scoped_pool;

use pbr::ProgressBar;
use point_viewer::errors::*;
use point_viewer::math::{CuboidLike, Cube, Cuboid};
use point_viewer::octree;
use point_viewer::Point;
use point_viewer::proto;
use point_viewer::pts::PtsIterator;
use point_viewer::ply::PlyIterator;
use protobuf::core::Message;
use scoped_pool::{Scope, Pool};
use std::collections::HashSet;
use std::fs::{self, File};
use std::io::Stdout;
use std::path::{Path, PathBuf};
use std::sync::mpsc;

const UPDATE_COUNT: i64 = 100000;
const MAX_POINTS_PER_NODE: i64 = 100000;

struct SplittedNode {
    node: octree::Node,
    num_points: i64,
}

fn split<PointIterator: Iterator<Item = Point>>(output_directory: &Path,
                                                resolution: f64,
                                                node: &octree::Node,
                                                stream: PointIterator)
                                                -> Vec<SplittedNode> {
    let mut children: Vec<Option<octree::NodeWriter>> = vec![None, None, None, None, None, None,
                                                             None, None];
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
            children[array_index] = Some(octree::NodeWriter::new(output_directory,
                                                                 &node.get_child(child_index),
                                                                 resolution));
        }
        children[array_index].as_mut().unwrap().write(&p);
    }

    // Remove the node file on disk by reopening the node and immediately dropping it again without
    // writing a point. This only saves some disk space during processing - all nodes will be
    // rewritten by subsampling the children in the second step anyways. We also ignore file
    // removing error. For example, we never write out the root, so it cannot be removed.
    octree::NodeWriter::new(output_directory, &node, resolution);

    let mut rv = Vec::new();
    for (child_index, c) in children.into_iter().enumerate() {
        if c.is_none() {
            continue;
        }
        let c = c.unwrap();

        rv.push(SplittedNode {
            node: node.get_child(octree::ChildIndex::from_u8(child_index as u8)),
            num_points: c.num_written(),
        });
    }
    rv
}

fn split_node<'a, 'b: 'a, Points>(scope: &Scope<'a>,
                                  output_directory: &'b Path,
                                  resolution: f64,
                                  splitted_node: SplittedNode,
                                  stream: Points,
                                  leaf_nodes_sender: mpsc::Sender<octree::Node>)
    where Points: Iterator<Item = Point>
{
    let children = split(output_directory, resolution, &splitted_node.node, stream);
    let (leaf_nodes, split_nodes): (Vec<_>, Vec<_>) = children.into_iter()
        .partition(|n| n.num_points < MAX_POINTS_PER_NODE);

    for child in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let stream = octree::NodeIterator::from_disk(output_directory, &child.node.id).unwrap();
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
    let mut parent_writer = octree::NodeWriter::new(output_directory, &node, resolution);
    println!("Creating {} from subsampling children.", &node.id);
    for i in 0..8 {
        let child = node.get_child(octree::ChildIndex::from_u8(i));
        let node_iterator = match octree::NodeIterator::from_disk(output_directory, &child.id) {
            Ok(node_iterator) => node_iterator,
            Err(Error(ErrorKind::NodeNotFound, _)) => continue,
            Err(err) => return Err(err),
        };

        // We read all points into memory, because the new node writer will rewrite this child's
        // file(s).
        let points = node_iterator.collect::<Vec<Point>>().into_iter();

        let mut child_writer = octree::NodeWriter::new(output_directory, &child, resolution);
        for (idx, p) in points.enumerate() {
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
        InputFile::Ply(ref filename) => Box::new(PlyIterator::new(filename).unwrap()),
        InputFile::Pts(ref filename) => Box::new(PtsIterator::new(filename)),
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
            progress_bar.as_mut().map(|pb| pb.add(UPDATE_COUNT as u64));
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

    let mut meta = proto::Meta::new();
    meta.mut_bounding_cube().mut_min().set_x(bounding_cube.min().x);
    meta.mut_bounding_cube().mut_min().set_y(bounding_cube.min().y);
    meta.mut_bounding_cube().mut_min().set_z(bounding_cube.min().z);
    meta.mut_bounding_cube().set_edge_length(bounding_cube.edge_length());
    meta.set_resolution(resolution);
    meta.set_version(octree::CURRENT_VERSION);
    let mut meta_pb = File::create(&output_directory.join("meta.pb")).unwrap();
    meta.write_to_writer(&mut meta_pb).unwrap();

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
