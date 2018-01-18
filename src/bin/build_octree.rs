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
use point_viewer::{InternalIterator, Point};
use point_viewer::errors::*;
use point_viewer::math::{Cuboid, CuboidLike};
use point_viewer::octree;
use point_viewer::ply::PlyIterator;
use point_viewer::proto;
use point_viewer::pts::PtsIterator;
use protobuf::Message;
use scoped_pool::{Pool, Scope};
use std::collections::HashSet;
use std::fs::{self, File};
use std::io::{BufWriter, Stdout};
use std::path::{Path, PathBuf};
use std::sync::mpsc;

const UPDATE_COUNT: i64 = 100000;
const MAX_POINTS_PER_NODE: i64 = 100000;

struct SplittedNode {
    node: octree::Node,
    num_points: i64,
}

fn split<P>(
    output_directory: &Path,
    resolution: f64,
    node: &octree::Node,
    stream: P,
) -> Vec<SplittedNode>
where
    P: InternalIterator,
{
    let mut children: Vec<Option<octree::NodeWriter>> =
        vec![None, None, None, None, None, None, None, None];
    match stream.size_hint() {
        Some(size) => println!(
            "Splitting {} which has {} points ({:.2}x MAX_POINTS_PER_NODE).",
            node.id,
            size,
            size as f64 / MAX_POINTS_PER_NODE as f64
        ),
        None => println!(
            "Splitting {} which has an unknown number of points.",
            node.id
        ),
    };

    stream.for_each(|p| {
        let child_index = node.get_child_id_containing_point(&p.position);
        let array_index = child_index.as_u8() as usize;
        if children[array_index].is_none() {
            children[array_index] = Some(octree::NodeWriter::new(
                output_directory,
                &node.get_child(child_index),
                resolution,
            ));
        }
        children[array_index].as_mut().unwrap().write(&p);
    });

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

fn should_split_node(node: &SplittedNode, resolution: f64) -> bool {
    if node.num_points <= MAX_POINTS_PER_NODE {
        return false;
    }
    if node.node.bounding_cube.edge_length() as f64 <= resolution {
        // TODO(hrapp): If the data has billion of points in this small spot, performance will
        // greatly suffer if we display it. Drop points?
        println!(
            "Node {} which has {} points ({:.2}x MAX_POINTS_PER_NODE) \
             is too small to be split, keeping all points.",
            node.node.id,
            node.num_points,
            node.num_points as f64 / MAX_POINTS_PER_NODE as f64
        );
        return false;
    }
    true
}

fn split_node<'a, 'b: 'a, P>(
    scope: &Scope<'a>,
    output_directory: &'b Path,
    resolution: f64,
    splitted_node: SplittedNode,
    stream: P,
    leaf_nodes_sender: mpsc::Sender<octree::Node>,
) where
    P: InternalIterator,
{
    let children = split(output_directory, resolution, &splitted_node.node, stream);
    let (leaf_nodes, split_nodes): (Vec<_>, Vec<_>) = children
        .into_iter()
        .partition(|n| !should_split_node(n, resolution));

    for child in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let stream = octree::NodeIterator::from_disk(output_directory, &child.node.id).unwrap();
            split_node(
                scope,
                output_directory,
                resolution,
                child,
                stream,
                leaf_nodes_sender_clone,
            );
        });
    }

    for splitted_node in leaf_nodes {
        leaf_nodes_sender.send(splitted_node.node).unwrap();
    }
}

fn subsample_children_into(
    output_directory: &Path,
    node: &octree::Node,
    resolution: f64,
) -> Result<()> {
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
        let mut points = Vec::with_capacity(node_iterator.size_hint().unwrap());
        node_iterator.for_each(|p| points.push((*p).clone()));

        let mut child_writer = octree::NodeWriter::new(output_directory, &child, resolution);
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

enum InputFileIterator {
    Ply(PlyIterator),
    Pts(PtsIterator),
}

impl InternalIterator for InputFileIterator {
    fn size_hint(&self) -> Option<usize> {
        match *self {
            InputFileIterator::Ply(ref p) => p.size_hint(),
            InputFileIterator::Pts(ref p) => p.size_hint(),
        }
    }

    fn for_each<F: FnMut(&Point)>(self, f: F) {
        match self {
            InputFileIterator::Ply(p) => p.for_each(f),
            InputFileIterator::Pts(p) => p.for_each(f),
        }
    }
}

fn make_stream(input: &InputFile) -> (InputFileIterator, Option<pbr::ProgressBar<Stdout>>) {
    let stream = match *input {
        InputFile::Ply(ref filename) => InputFileIterator::Ply(PlyIterator::new(filename).unwrap()),
        InputFile::Pts(ref filename) => InputFileIterator::Pts(PtsIterator::new(filename)),
    };

    let progress_bar = match stream.size_hint() {
        Some(size) => Some(ProgressBar::new(size as u64)),
        None => None,
    };
    (stream, progress_bar)
}

/// Returns the bounding_box and the number of the points in 'input'.
fn find_bounding_box(input: &InputFile) -> (Cuboid, i64) {
    let mut num_points = 0i64;
    let mut bounding_box = Cuboid::new();
    let (stream, mut progress_bar) = make_stream(input);
    progress_bar
        .as_mut()
        .map(|pb| pb.message("Determining bounding box: "));

    stream.for_each(|p: &Point| {
        bounding_box.update(&p.position);
        num_points += 1;
        if num_points % UPDATE_COUNT == 0 {
            progress_bar.as_mut().map(|pb| pb.add(UPDATE_COUNT as u64));
        }
    });
    progress_bar.map(|mut f| f.finish());
    (bounding_box, num_points)
}

fn main() {
    let matches = clap::App::new("build_octree")
        .args(&[
            clap::Arg::with_name("output_directory")
                .help("Output directory to write the octree into.")
                .long("output_directory")
                .required(true)
                .takes_value(true),
            clap::Arg::with_name("resolution")
                .help(
                    "Minimal precision that this point cloud should have. This decides \
                     on the number of bits used to encode each node.",
                )
                .long("resolution")
                .default_value("0.001"),
            clap::Arg::with_name("input")
                .help("PLY/PTS file to parse for the points.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let output_directory = &PathBuf::from(matches.value_of("output_directory").unwrap());
    let resolution = matches
        .value_of("resolution")
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

    let (bounding_box, num_points) = find_bounding_box(&input);

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);

    let meta = {
        let mut meta = proto::Meta::new();
        meta.mut_bounding_box()
            .mut_min()
            .set_x(bounding_box.min().x);
        meta.mut_bounding_box()
            .mut_min()
            .set_y(bounding_box.min().y);
        meta.mut_bounding_box()
            .mut_min()
            .set_z(bounding_box.min().z);
        meta.mut_bounding_box()
            .mut_max()
            .set_x(bounding_box.max().x);
        meta.mut_bounding_box()
            .mut_max()
            .set_y(bounding_box.max().y);
        meta.mut_bounding_box()
            .mut_max()
            .set_z(bounding_box.max().z);
        meta.set_resolution(resolution);
        meta.set_version(octree::CURRENT_VERSION);
        meta
    };

    let mut buf_writer = BufWriter::new(File::create(&output_directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();

    println!("Creating octree structure.");
    let pool = Pool::new(10);

    let (leaf_nodes_sender, leaf_nodes_receiver) = mpsc::channel();
    pool.scoped(move |scope| {
        let (root_stream, _) = make_stream(&input);
        let root = SplittedNode {
            node: octree::Node::root_with_bounding_cube(bounding_box.into_cube()),
            num_points: num_points,
        };
        split_node(
            scope,
            output_directory,
            resolution,
            root,
            root_stream,
            leaf_nodes_sender.clone(),
        );
    });

    let mut deepest_level = 0usize;
    let mut nodes_to_subsample = Vec::<octree::Node>::new();
    for leaf_node in leaf_nodes_receiver.into_iter() {
        deepest_level = std::cmp::max(deepest_level, leaf_node.level());
        nodes_to_subsample.push(leaf_node);
    }

    // We start on the deepest level and work our way up the tree.
    for current_level in (0..deepest_level + 1).rev() {
        // All nodes on the same level can be subsampled in parallel.
        let res = nodes_to_subsample
            .into_iter()
            .partition(|n| n.level() == current_level);
        nodes_to_subsample = res.1;

        let mut parent_ids = HashSet::new();
        let mut subsample_nodes = Vec::new();
        for node in res.0 {
            let maybe_parent = node.parent();
            if maybe_parent.is_none() {
                // Only the root has no parents.
                assert_eq!(node.level(), 0);
                continue;
            }
            let parent = maybe_parent.unwrap();
            if parent_ids.contains(&parent.id) {
                continue;
            }
            parent_ids.insert(parent.id);
            subsample_nodes.push(parent);
        }

        pool.scoped(|scope| {
            for node in &subsample_nodes {
                scope.execute(move || {
                    subsample_children_into(output_directory, node, resolution).unwrap();
                });
            }
        });

        // The nodes that were just now created through sub-sampling will be required to create
        // their parents.
        nodes_to_subsample.extend(subsample_nodes.into_iter());
    }
}
