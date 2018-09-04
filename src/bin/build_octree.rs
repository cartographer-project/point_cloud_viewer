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
extern crate fnv;
extern crate lru_cache;
extern crate pbr;
extern crate point_viewer;
extern crate protobuf;
extern crate scoped_pool;
extern crate rand;

use cgmath::{EuclideanSpace, Point3};
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use lru_cache::LruCache;
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
use std::fs::{self, File};
use std::io::{BufWriter, Stdout};
use std::path::PathBuf;
use std::sync::{Arc, Mutex, Once, ONCE_INIT, mpsc};
use std::io;
use std::io::prelude::*;
use std::{mem, thread};
use rand::Rng;
use std::collections::HashMap;

const MAX_NODES_IN_CACHE: usize = 1000;
const UPDATE_COUNT: i64 = 100000;
const MAX_POINTS_PER_NODE: i64 = 10000;

// Generate random number in range (0, range)
fn random_num_in_range(range: i64) -> usize {
    rand::thread_rng().gen_range(0, range) as usize
}

#[derive(Debug)]
pub struct OctreeBuilder<'a> {
    // Octree Meta for storing bounding box, resolution and output directory
    octree_meta: &'a octree::OctreeMeta,
    // Hashmap for storing points seen by node
    points_seen_in_node: HashMap<octree::NodeId, i64>,
    // Lru Cache for storing points in node
    cache: LruCache<octree::NodeId, Vec<point_viewer::Point>>,
    // Cache hits
    hits: usize,
    // Cache misses
    misses: usize,
}

impl<'a> OctreeBuilder<'a> {
    pub fn new(meta: &'a octree::OctreeMeta) -> Self {
        OctreeBuilder {
            octree_meta: meta,
            points_seen_in_node: HashMap::new(),
            cache: LruCache::new(MAX_NODES_IN_CACHE),
            hits: 0,
            misses: 0,
        }
    }

    // insert or update Lru Cache
    pub fn insert_or_update_cache(
        & mut self,
        node_id:  &octree::NodeId,
        points: &Vec<point_viewer::Point>
    ) {
        match self.cache.remove(&node_id) {
            Some(points) => {
                println!("Removed points in cache for node id {}", node_id);
                match self.cache.insert(*node_id, points) {
                    Some(points) => println!("Insert node id {} in cache", node_id),
                    None => panic!("Cannot insert node id {} in cache", node_id),
                }
            },
            None => {
                println!("Didn't read points from cache for node id {}", node_id);
                self.cache.insert(*node_id, points.to_vec());
            },
        }
    }

    // Read from cache or disk
    pub fn read_cache_or_disk(
        & mut self,
        node_id:  &octree::NodeId,
    ) -> Vec<point_viewer::Point> {
        let points = match self.cache.get_mut(&node_id) {
            Some(cached_points) => {
                println!("Found points in cache for {}", node_id);
                cached_points.clone()
            },
            // Case: The current node is empty
            None => {
                println!("Reading points from cache {}", node_id);
                let node_iterator = octree::NodeIterator::from_disk(&self.octree_meta, &node_id).unwrap();
                let mut pd = &mut Vec::with_capacity(node_iterator.size_hint().unwrap());
                node_iterator.for_each(|p| pd.push((*p).clone()));
                pd.clone()
                // Insert in Cache
            },
        };
        points
    }

    // Sample the points in node using reservoir sampling algorithm
    fn reservoir_sample_node(
        & mut self,
        node_id:  &octree::NodeId,
        bounding_cube: &Cube,
        point: &point_viewer::Point,
    ) {
        let child_index = octree::ChildIndex::from_bounding_cube(&bounding_cube, &point.position);
        let child_id = node_id.get_child_id(child_index);
        let child_bounding_cube = child_id.find_bounding_cube(&bounding_cube);
        *self.points_seen_in_node.entry(*node_id).or_insert(0) += 1;

        // Case 1: New node with 0 points
        if !self.points_seen_in_node.contains_key(&node_id) {
            let mut node_writer = octree::NodeWriter::new(&self.octree_meta, &node_id);
            node_writer.write(&point);
            return;
        }

        let mut points = self.read_cache_or_disk(&node_id);
        let mut node_writer = octree::NodeWriter::new(&self.octree_meta, &node_id);
        let points_seen = self.points_seen_in_node[&node_id];

        // Case 1: The current node has less than or equal to MAX_POINTS_PER_NODE points
        if points_seen < MAX_POINTS_PER_NODE as i64 {
            points.push(point.clone());
            self.insert_or_update_cache(&node_id, &points);
            for p in points {
                node_writer.write(&p);
            }
            return;
        }  else {
            // Case 2: The current node has less than or equal to MAX_POINTS_PER_NODE points
            let random_index: usize = random_num_in_range(points_seen);
            if random_index >= MAX_POINTS_PER_NODE as usize {
                // Not needed to read file from disk
                self.insert_or_update_cache(&node_id, &points);
                for p in points {
                    node_writer.write(&p);
                }
                self.reservoir_sample_node(&child_id, &child_bounding_cube, &point);
            } else {
                let evicted_point = points[random_index].clone();
                points[random_index] = point.clone();
                let evicted_child_index = octree::ChildIndex::from_bounding_cube(&bounding_cube, &evicted_point.position);
                let evicted_child_id = node_id.get_child_id(evicted_child_index);
                let evicted_child_bounding_cube = evicted_child_id.find_bounding_cube(&bounding_cube);
                self.insert_or_update_cache(&node_id, &points);
                for p in points {
                    node_writer.write(&p);
                }
                self.reservoir_sample_node(&child_id, &evicted_child_bounding_cube, &evicted_point);
            }
        }
    }
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

/// Returns the bounding box containing all points
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

    println!("Creating octree structure.");

    let mut octree_builder = OctreeBuilder::new(octree_meta);
    let root_node = octree::Node::root_with_bounding_cube(Cube::bounding(&bounding_box));
    let bounding_cube = Cube::bounding(&bounding_box);
    let (root_stream, _) = make_stream(&input);
    root_stream.for_each(|point| {
        octree_builder.reservoir_sample_node(&root_node.id, &bounding_cube, &point);
    });

    println!("Octree build completed!");
}
