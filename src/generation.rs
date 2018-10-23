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

use cgmath::{EuclideanSpace, Point3};
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use lru_cache::LruCache;
use pbr::ProgressBar;
use InternalIterator;
use Point;
use errors::*;
use math::Cube;
use octree;
use ply::PlyIterator;
use proto;
use pts::PtsIterator;
use protobuf::Message;
use scoped_pool::{Pool, Scope};
use std::fs::{self, File};
use std::io::{BufWriter, Stdout};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, Once, ONCE_INIT, mpsc};
use std::io;
use std::io::prelude::*;
use std::{mem, thread};
use std::mem::{size_of};
use rand::{self, Rng};
use std::collections::HashMap;

const MAX_NODES_IN_CACHE: usize = 100000;
const UPDATE_COUNT: i64 = 1000;
const MAX_POINTS_PER_NODE: i64 = 100000;

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
    cache: LruCache<octree::NodeId, Vec<Point>>,
}

impl<'a> OctreeBuilder<'a> {
    pub fn new(meta: &'a octree::OctreeMeta) -> Self {
        OctreeBuilder {
            octree_meta: meta,
            points_seen_in_node: HashMap::new(),
            cache: LruCache::new(MAX_NODES_IN_CACHE),
        }
    }

    // insert or update Lru Cache
    pub fn insert_or_update_cache(
        & mut self,
        node_id:  &octree::NodeId,
        points: &Vec<Point>
    ) {
        self.cache.remove(&node_id);
        self.cache.insert(*node_id, points.to_vec());
    }

    // Read from cache or disk
    pub fn read_cache_or_disk(
        & mut self,
        node_id:  &octree::NodeId,
    ) -> Vec<Point> {
        // match self.cache.get_mut(node_id) {
        //     Some(points) => println!("Cached points {}: {}", node_id, count),
        //     None => println!("{} is unreviewed.", node_id)
        // }
        if self.cache.contains_key(&node_id) {
            let mut cached_points = self.cache.get_mut(&node_id).unwrap();
            return cached_points.to_vec();
        }

        let node_iterator = octree::NodeIterator::from_disk(&self.octree_meta, &node_id).unwrap();
        let mut points = &mut Vec::with_capacity(node_iterator.size_hint().unwrap());
        node_iterator.for_each(|p| points.push((*p).clone()));
        self.cache.remove(&node_id);
        self.cache.insert(*node_id, points.to_vec());
        points.clone()
    }

    // Sample the points in node using reservoir sampling algorithm
    fn reservoir_sample_node(
        & mut self,
        node_id:  &octree::NodeId,
        bounding_cube: &Cube,
        point: &Point,
    ) {
        let child_index = octree::ChildIndex::from_bounding_cube(&bounding_cube, &point.position);
        let child_id = node_id.get_child_id(child_index);
        let child_bounding_cube = child_id.find_bounding_cube(&bounding_cube);

        // Case 1: New node with 0 points
        if !self.points_seen_in_node.contains_key(&node_id) {
            *self.points_seen_in_node.entry(*node_id).or_insert(1);
            let mut node_writer = octree::NodeWriter::new(&self.octree_meta, &node_id);
            self.cache.insert(*node_id, vec![point.clone()]);
            node_writer.write(&point);
            return;
        }

        *self.points_seen_in_node.entry(*node_id).or_insert(1) += 1;

        let mut points = self.read_cache_or_disk(&node_id);
        let points_seen = self.points_seen_in_node[&node_id];

        // Case 2: The current node has less than or equal to MAX_POINTS_PER_NODE points
        if points_seen <= MAX_POINTS_PER_NODE as i64 {
            self.cache.get_mut(&node_id).unwrap().push(point.clone());
            return;
        }  else {
            // Case 3: The current node has less than or equal to MAX_POINTS_PER_NODE points
            let random_index: usize = random_num_in_range(points_seen);
            if random_index >= MAX_POINTS_PER_NODE as usize {
                self.reservoir_sample_node(&child_id, &child_bounding_cube, &point);
            } else {
                if !self.cache.contains_key(&node_id) {
                    let node_iterator = octree::NodeIterator::from_disk(&self.octree_meta, &node_id).unwrap();
                    let mut points = Vec::with_capacity(node_iterator.size_hint().unwrap());
                    node_iterator.for_each(|p| points.push((p).clone()));
                    self.cache.insert(*node_id, points);
                }
                let mut points = self.read_cache_or_disk(&node_id);
                let evicted_point = mem::replace(&mut points[random_index], point.clone());
                let evicted_child_index = octree::ChildIndex::from_bounding_cube(&bounding_cube, &evicted_point.position);
                let evicted_child_id = node_id.get_child_id(evicted_child_index);
                let evicted_child_bounding_cube = evicted_child_id.find_bounding_cube(&bounding_cube);
                self.insert_or_update_cache(&node_id, &points);
                self.reservoir_sample_node(&evicted_child_id, &evicted_child_bounding_cube, &evicted_point);
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

fn make_stream(input: &InputFile) -> (InputFileIterator, Option<ProgressBar<Stdout>>) {
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

pub fn build_octree_from_file(output_directory: impl AsRef<Path>, resolution: f64, filename: impl AsRef<Path>) {
    // TODO(ksavinash9): This function should return a Result.
    let input = {
        match filename.as_ref().extension().and_then(|s| s.to_str()) {
            Some("ply") => InputFile::Ply(filename.as_ref().to_path_buf()),
            Some("pts") => InputFile::Pts(filename.as_ref().to_path_buf()),
            other => panic!("Unknown input file format: {:?}", other),
        }
    };
    let bounding_box = find_bounding_box(&input);
    let (stream, _) = make_stream(&input);
    build_octree(output_directory, resolution, bounding_box, stream)
}

pub fn build_octree(output_directory: impl AsRef<Path>, resolution: f64, bounding_box: Aabb3<f32>, input: impl InternalIterator) {
    // TODO(ksavinash9): This function should return a Result.
    let octree_meta = &octree::OctreeMeta {
        bounding_box,
        resolution,
        directory: output_directory.as_ref().to_path_buf(),
    };

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory.as_ref());

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

    let mut octree_builder = OctreeBuilder::new(octree_meta);
    let root_node = octree::Node::root_with_bounding_cube(Cube::bounding(&bounding_box));
    let bounding_cube = Cube::bounding(&bounding_box);

    let mut progress_bar = match input.size_hint() {
        Some(size) => Some(ProgressBar::new(size as u64)),
        None => None,
    };
    let mut num_points = 0;
    progress_bar
        .as_mut()
        .map(|pb| pb.message("Reservoir Sampling Points: "));

    input.for_each(|point| {
        octree_builder.reservoir_sample_node(&root_node.id, &bounding_cube, &point);
        num_points += 1;
        if num_points % UPDATE_COUNT == 0 {
            progress_bar.as_mut().map(|pb| pb.add(UPDATE_COUNT as u64));
        }
    });
    progress_bar.map(|mut f| f.finish());

    progress_bar = Some(ProgressBar::new(octree_builder.cache.len() as u64));
    for (node_id, points) in octree_builder.cache {
        let mut node_writer = octree::NodeWriter::new(octree_meta, &node_id);
        for p in points {
            node_writer.write(&p);
        }
        progress_bar.as_mut().map(|f| f.inc());
    }
    println!("Octree build completed!");
}
