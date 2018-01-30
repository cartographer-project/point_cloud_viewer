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
extern crate cgmath;
extern crate clap;
extern crate collision;
extern crate pbr;
extern crate point_viewer;
extern crate protobuf;
extern crate scoped_pool;

use cgmath::{EuclideanSpace, Point3};
use collision::{Aabb, Aabb3};
use pbr::ProgressBar;
use point_viewer::{InternalIterator, Point};
use point_viewer::errors::*;
use point_viewer::math::Cube;
use point_viewer::octree;
use point_viewer::ply::PlyIterator;
use point_viewer::proto;
use point_viewer::pts::PtsIterator;
use protobuf::Message;
use scoped_pool::{Pool, Scope};
use std::collections::{HashMap, HashSet};
use std::fs::{self, File};
use std::io::{BufWriter, Stdout};
use std::path::PathBuf;
use std::sync::mpsc;

const UPDATE_COUNT: i64 = 100000;
const MAX_POINTS_PER_NODE: i64 = 100000;

// Return a list a leaf nodes and a list of nodes to be splitted further.
fn split<P>(
    octree_meta: &octree::OctreeMeta,
    node_id: &octree::NodeId,
    stream: P,
) -> (Vec<octree::NodeId>, Vec<octree::NodeId>)
where
    P: InternalIterator,
{
    let mut children: Vec<Option<octree::NodeWriter>> =
        vec![None, None, None, None, None, None, None, None];
    match stream.size_hint() {
        Some(size) => println!(
            "Splitting {} which has {} points ({:.2}x MAX_POINTS_PER_NODE).",
            node_id,
            size,
            size as f64 / MAX_POINTS_PER_NODE as f64
        ),
        None => println!(
            "Splitting {} which has an unknown number of points.",
            node_id
        ),
    };

    let bounding_cube = node_id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
    stream.for_each(|p| {
        let child_index = octree::ChildIndex::from_bounding_cube(&bounding_cube, &p.position);
        let array_index = child_index.as_u8() as usize;
        if children[array_index].is_none() {
            children[array_index] = Some(octree::NodeWriter::new(
                &octree_meta,
                &node_id.get_child_id(child_index),
            ));
        }
        children[array_index].as_mut().unwrap().write(p);
    });

    // Remove the node file on disk by reopening the node and immediately dropping it again without
    // writing a point. This only saves some disk space during processing - all nodes will be
    // rewritten by subsampling the children in the second step anyways. We also ignore file
    // removing error. For example, we never write out the root, so it cannot be removed.
    octree::NodeWriter::new(&octree_meta, &node_id);

    let mut leaf_nodes = Vec::new();
    let mut split_nodes = Vec::new();
    for (child_index, c) in children.into_iter().enumerate() {
        if c.is_none() {
            continue;
        }
        let c = c.unwrap();
        let child_id = node_id.get_child_id(octree::ChildIndex::from_u8(child_index as u8));

        if should_split_node(&child_id, c.num_written(), octree_meta) {
            split_nodes.push(child_id);
        } else {
            leaf_nodes.push(child_id);
        }
    }
    (leaf_nodes, split_nodes)
}

fn should_split_node(
    id: &octree::NodeId,
    num_points: i64,
    octree_meta: &octree::OctreeMeta,
) -> bool {
    if num_points <= MAX_POINTS_PER_NODE {
        return false;
    }
    let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
    if f64::from(bounding_cube.edge_length()) <= octree_meta.resolution {
        // TODO(hrapp): If the data has billion of points in this small spot, performance will
        // greatly suffer if we display it. Drop points?
        println!(
            "Node {} which has {} points ({:.2}x MAX_POINTS_PER_NODE) \
             is too small to be split, keeping all points.",
            id,
            num_points,
            num_points as f64 / MAX_POINTS_PER_NODE as f64
        );
        return false;
    }
    true
}

fn split_node<'a, 'b: 'a, P>(
    scope: &Scope<'a>,
    octree_meta: &'a octree::OctreeMeta,
    node_id: &octree::NodeId,
    stream: P,
    leaf_nodes_sender: &mpsc::Sender<octree::NodeId>,
) where
    P: InternalIterator,
{
    let (leaf_nodes, split_nodes) = split(octree_meta, &node_id, stream);
    for child_id in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let stream = octree::NodeIterator::from_disk(&octree_meta, &child_id).unwrap();
            split_node(
                scope,
                octree_meta,
                &child_id,
                stream,
                &leaf_nodes_sender_clone,
            );
        });
    }

    for id in leaf_nodes {
        leaf_nodes_sender.send(id).unwrap();
    }
}

fn subsample_children_into(
    octree_meta: &octree::OctreeMeta,
    node_id: &octree::NodeId,
    nodes_sender: &mpsc::Sender<(octree::NodeId, i64)>,
) -> Result<()> {
    let mut parent_writer = octree::NodeWriter::new(&octree_meta, &node_id);
    println!("Creating {} from subsampling children.", node_id);
    for i in 0..8 {
        let child_id = node_id.get_child_id(octree::ChildIndex::from_u8(i));
        let node_iterator = match octree::NodeIterator::from_disk(&octree_meta, &child_id) {
            Ok(node_iterator) => node_iterator,
            Err(Error(ErrorKind::NodeNotFound, _)) => continue,
            Err(err) => return Err(err),
        };

        // We read all points into memory, because the new node writer will rewrite this child's
        // file(s).
        let mut points = Vec::with_capacity(node_iterator.size_hint().unwrap());
        node_iterator.for_each(|p| points.push((*p).clone()));

        let mut child_writer = octree::NodeWriter::new(&octree_meta, &child_id);
        for (idx, p) in points.into_iter().enumerate() {
            if idx % 8 == 0 {
                parent_writer.write(&p);
            } else {
                child_writer.write(&p);
            }
        }
        // Update child.
        nodes_sender
            .send((child_id, child_writer.num_written()))
            .unwrap();
    }
    // Make sure the root node is also tracked as an existing node.
    if node_id.level() == 0 {
        nodes_sender
            .send((*node_id, parent_writer.num_written()))
            .unwrap();
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
fn find_bounding_box(input: &InputFile) -> Aabb3<f32> {
    let mut num_points = 0i64;
    let mut bounding_box = Aabb3::zero();
    let (stream, mut progress_bar) = make_stream(input);
    progress_bar
        .as_mut()
        .map(|pb| pb.message("Determining bounding box: "));

    stream.for_each(|p: &Point| {
        bounding_box = bounding_box.grow(Point3::from_vec(p.position));
        num_points += 1;
        if num_points % UPDATE_COUNT == 0 {
            progress_bar.as_mut().map(|pb| pb.add(UPDATE_COUNT as u64));
        }
    });
    progress_bar.map(|mut f| f.finish());
    bounding_box
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

    let bounding_box = find_bounding_box(&input);

    let octree_meta = &octree::OctreeMeta {
        bounding_box,
        resolution,
        directory: output_directory.clone(),
    };

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);

    let mut meta = {
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

    println!("Creating octree structure.");
    let pool = Pool::new(10);

    let (leaf_nodes_sender, leaf_nodes_receiver) = mpsc::channel();
    pool.scoped(move |scope| {
        let (root_stream, _) = make_stream(&input);
        let root_node = octree::Node::root_with_bounding_cube(Cube::bounding(&bounding_box));
        split_node(
            scope,
            octree_meta,
            &root_node.id,
            root_stream,
            &leaf_nodes_sender,
        );
    });

    let mut nodes_to_subsample = Vec::new();
    let mut deepest_level = 0usize;
    for id in leaf_nodes_receiver {
        deepest_level = std::cmp::max(deepest_level, id.level());
        nodes_to_subsample.push(id);
    }
    let mut finished_nodes = HashMap::new();

    // sub sampling returns the list of finished nodes including all meta data
    // We start on the deepest level and work our way up the tree.
    for current_level in (0..deepest_level + 1).rev() {
        // All nodes on the same level can be subsampled in parallel.
        let res = nodes_to_subsample
            .into_iter()
            .partition(|n| n.level() == current_level);
        nodes_to_subsample = res.1;

        let mut parent_ids = HashSet::new();
        let mut subsample_nodes = Vec::new();
        for id in res.0 {
            let maybe_parent_id = id.parent_id();
            if maybe_parent_id.is_none() {
                // Only the root has no parents.
                assert_eq!(id.level(), 0);
                continue;
            }
            let parent_id = maybe_parent_id.unwrap();
            if parent_ids.contains(&parent_id) {
                continue;
            }
            parent_ids.insert(parent_id);
            subsample_nodes.push(parent_id);
        }

        let (finished_nodes_sender, finished_nodes_receiver) = mpsc::channel();
        pool.scoped(|scope| {
            for id in &subsample_nodes {
                let finished_nodes_sender_clone = finished_nodes_sender.clone();
                scope.execute(move || {
                    subsample_children_into(octree_meta, id, &finished_nodes_sender_clone).unwrap();
                });
            }
        });

        drop(finished_nodes_sender);
        for (id, num_points) in finished_nodes_receiver {
            if num_points > 0 {
                finished_nodes.insert(id, num_points);
            }
        }

        // The nodes that were just now created through sub-sampling will be required to create
        // their parents.
        nodes_to_subsample.extend(subsample_nodes.into_iter());
    }

    // Add all non-zero node meta data to meta.pb
    for (id, num_points) in finished_nodes {
        let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        let position_encoding =
            octree::PositionEncoding::new(&bounding_cube, octree_meta.resolution);

        let mut proto = proto::Node::new();
        proto.mut_id().set_level(id.level() as i32);
        proto.mut_id().set_index(id.index() as i64);
        proto.set_num_points(num_points);
        proto
            .mut_bounding_cube()
            .mut_min()
            .set_x(bounding_cube.min().x);
        proto
            .mut_bounding_cube()
            .mut_min()
            .set_y(bounding_cube.min().y);
        proto
            .mut_bounding_cube()
            .mut_min()
            .set_z(bounding_cube.min().z);
        proto
            .mut_bounding_cube()
            .set_edge_length(bounding_cube.edge_length());
        proto.set_position_encoding(position_encoding.to_proto());
        meta.mut_nodes().push(proto);
    }

    let mut buf_writer = BufWriter::new(File::create(&output_directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}
