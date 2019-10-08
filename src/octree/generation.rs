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

use crate::data_provider::OnDiskDataProvider;
use crate::errors::*;
use crate::math::Cube;
use crate::octree::{self, to_meta_proto, to_node_proto, ChildIndex, NodeId, OctreeMeta};
use crate::proto;
use crate::read_write::{
    attempt_increasing_rlimit_to_max, Encoding, NodeIterator, NodeWriter, OpenMode, PlyIterator,
    PositionEncoding, RawNodeWriter,
};
use crate::{NumberOfPoints, PointsBatch, NUM_POINTS_PER_BATCH};
use cgmath::{EuclideanSpace, Point3, Vector3};
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use pbr::ProgressBar;
use protobuf::Message;
use scoped_pool::{Pool, Scope};
use std::cmp;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;
use std::sync::mpsc;
use std::time::Duration;

const MAX_POINTS_PER_NODE: i64 = 100_000;

impl RawNodeWriter {
    fn from_data_provider(
        octree_data_provider: &OnDiskDataProvider,
        octree_meta: &OctreeMeta,
        node_id: &NodeId,
    ) -> Self {
        let path = octree_data_provider.stem(&node_id.to_string());
        let bounding_cube = node_id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
        let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);
        let min = bounding_cube.min();
        RawNodeWriter::new(
            path,
            Encoding::ScaledToCube(
                Vector3::new(min.x, min.y, min.z),
                bounding_cube.edge_length(),
                position_encoding,
            ),
            OpenMode::Truncate,
        )
    }
}

// Return a list a leaf nodes and a list of nodes to be splitted further.
fn split<P>(
    octree_data_provider: &OnDiskDataProvider,
    octree_meta: &octree::OctreeMeta,
    node_id: &octree::NodeId,
    stream: P,
) -> (Vec<octree::NodeId>, Vec<octree::NodeId>)
where
    P: Iterator<Item = PointsBatch> + NumberOfPoints,
{
    let mut children: Vec<Option<RawNodeWriter>> =
        vec![None, None, None, None, None, None, None, None];
    let size = stream.num_points();
    println!(
        "Splitting {} which has {} points ({:.2}x MAX_POINTS_PER_NODE).",
        node_id,
        size,
        size as f64 / MAX_POINTS_PER_NODE as f64
    );

    let bounding_cube = node_id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
    stream.for_each(|batch| {
        let child_indices: Vec<_> = batch
            .position
            .iter()
            .map(|p| octree::ChildIndex::from_bounding_cube(&bounding_cube, &p))
            .collect();
        for (array_index, child_writer) in children.iter_mut().enumerate() {
            let mut child_batch = batch.clone();
            let keep: Vec<_> = child_indices
                .iter()
                .map(|i| i.as_u8() == array_index as u8)
                .collect();
            child_batch.retain(&keep);
            if !child_batch.position.is_empty() {
                if child_writer.is_none() {
                    *child_writer = Some(RawNodeWriter::from_data_provider(
                        octree_data_provider,
                        octree_meta,
                        &node_id.get_child_id(ChildIndex::from_u8(array_index as u8)),
                    ));
                }
                child_writer.as_mut().unwrap().write(&child_batch).unwrap();
            }
        }
    });

    // Remove the node file on disk by reopening the node and immediately dropping it again without
    // writing a point. This only saves some disk space during processing - all nodes will be
    // rewritten by subsampling the children in the second step anyways. We also ignore file
    // removing error. For example, we never write out the root, so it cannot be removed.
    RawNodeWriter::from_data_provider(octree_data_provider, octree_meta, node_id);

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
    if bounding_cube.edge_length() <= octree_meta.resolution {
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

fn split_node<'a, P>(
    scope: &Scope<'a>,
    octree_data_provider: &'a OnDiskDataProvider,
    octree_meta: &'a octree::OctreeMeta,
    node_id: &octree::NodeId,
    stream: P,
    leaf_nodes_sender: &mpsc::Sender<octree::NodeId>,
) where
    P: Iterator<Item = PointsBatch> + NumberOfPoints,
{
    let (leaf_nodes, split_nodes) = split(octree_data_provider, octree_meta, node_id, stream);
    for child_id in split_nodes {
        let leaf_nodes_sender_clone = leaf_nodes_sender.clone();
        scope.recurse(move |scope| {
            let stream = NodeIterator::from_data_provider(
                octree_data_provider,
                octree_meta.encoding_for_node(child_id),
                &child_id,
                octree_data_provider
                    .number_of_points(&child_id.to_string())
                    .unwrap() as usize,
                NUM_POINTS_PER_BATCH,
            )
            .unwrap();
            split_node(
                scope,
                octree_data_provider,
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
    octree_data_provider: &OnDiskDataProvider,
    octree_meta: &octree::OctreeMeta,
    node_id: &octree::NodeId,
    nodes_sender: &mpsc::Sender<(octree::NodeId, i64)>,
) -> Result<()> {
    let mut parent_writer =
        RawNodeWriter::from_data_provider(octree_data_provider, octree_meta, node_id);
    for i in 0..8 {
        let child_id = node_id.get_child_id(octree::ChildIndex::from_u8(i));
        let num_points = match octree_data_provider.number_of_points(&child_id.to_string()) {
            Ok(num_points) => num_points,
            Err(Error(ErrorKind::NodeNotFound, _)) => continue,
            Err(err) => return Err(err),
        };
        let mut node_iterator = NodeIterator::from_data_provider(
            octree_data_provider,
            octree_meta.encoding_for_node(child_id),
            &child_id,
            num_points as usize,
            NUM_POINTS_PER_BATCH,
        )?;

        // We read all points into memory, because the new node writer will rewrite this child's
        // file(s).
        let mut batch = node_iterator.next().unwrap();
        node_iterator.for_each(|mut b| batch.append(&mut b).unwrap());
        let (keep_parent, keep_child): (Vec<bool>, Vec<bool>) = (0..batch.position.len())
            .map(|i| {
                let in_parent = i % 8 == 0;
                (in_parent, !in_parent)
            })
            .unzip();
        let mut parent_batch = batch.clone();
        parent_batch.retain(&keep_parent);
        let mut child_batch = batch;
        child_batch.retain(&keep_child);

        let mut child_writer =
            RawNodeWriter::from_data_provider(octree_data_provider, octree_meta, &child_id);
        parent_writer.write(&parent_batch)?;
        child_writer.write(&child_batch)?;

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

/// Returns the bounding box containing all points
fn find_bounding_box(filename: impl AsRef<Path>) -> Aabb3<f64> {
    let mut bounding_box = None;
    let stream = PlyIterator::from_file(filename, NUM_POINTS_PER_BATCH).unwrap();
    let mut progress_bar = ProgressBar::new(stream.num_points() as u64);
    progress_bar.set_max_refresh_rate(Some(Duration::from_secs(2)));
    progress_bar.message("Determining bounding box: ");

    stream.for_each(|batch| {
        for position in batch.position {
            let p3 = Point3::from_vec(position);
            let b = bounding_box.get_or_insert(Aabb3::new(p3, p3));
            *b = b.grow(p3);
            progress_bar.inc();
        }
    });
    progress_bar.finish();
    bounding_box.unwrap_or_else(Aabb3::zero)
}

pub fn build_octree_from_file(
    pool: &Pool,
    output_directory: impl AsRef<Path>,
    resolution: f64,
    filename: impl AsRef<Path>,
) {
    let bounding_box = find_bounding_box(filename.as_ref());
    let stream = PlyIterator::from_file(filename, NUM_POINTS_PER_BATCH).unwrap();
    build_octree(pool, output_directory, resolution, bounding_box, stream)
}

pub fn build_octree(
    pool: &Pool,
    output_directory: impl AsRef<Path>,
    resolution: f64,
    bounding_box: Aabb3<f64>,
    input: impl Iterator<Item = PointsBatch> + NumberOfPoints,
) {
    attempt_increasing_rlimit_to_max();

    // TODO(ksavinash9): This function should return a Result.
    let octree_meta = &octree::OctreeMeta {
        bounding_box,
        resolution,
    };
    let octree_data_provider = OnDiskDataProvider {
        directory: output_directory.as_ref().to_path_buf(),
    };
    let octree_data_provider = &octree_data_provider;

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory.as_ref());

    println!("Creating octree structure.");

    let (leaf_nodes_sender, leaf_nodes_receiver) = mpsc::channel();
    pool.scoped(move |scope| {
        let root_node = octree::Node::root_with_bounding_cube(Cube::bounding(&bounding_box));
        split_node(
            scope,
            octree_data_provider,
            octree_meta,
            &root_node.id,
            input,
            &leaf_nodes_sender,
        );
    });

    let mut nodes_to_subsample = Vec::new();
    let mut deepest_level = 0u8;
    for id in leaf_nodes_receiver {
        deepest_level = cmp::max(deepest_level, id.level());
        nodes_to_subsample.push(id);
    }
    let mut finished_nodes = FnvHashMap::default();

    // sub sampling returns the list of finished nodes including all meta data
    // We start on the deepest level and work our way up the tree.
    for current_level in (1..=deepest_level).rev() {
        // All nodes on the same level can be subsampled in parallel.
        let res = nodes_to_subsample
            .into_iter()
            .partition(|n| n.level() == current_level);
        nodes_to_subsample = res.1;

        // Unwrap is safe, since we stop at current_level = 1, so the root can never appear.
        let parent_ids: FnvHashSet<_> = res
            .0
            .into_iter()
            .map(|id| id.parent_id().unwrap())
            .collect();
        let mut progress_bar = ProgressBar::new(parent_ids.len() as u64);
        progress_bar.message(&format!("Building level {}: ", current_level - 1));

        let (finished_nodes_sender, finished_nodes_receiver) = mpsc::channel();
        let (progress_tx, progress_rx) = mpsc::channel();
        pool.scoped(|scope| {
            scope.execute(|| {
                for (id, num_points) in finished_nodes_receiver {
                    finished_nodes.insert(id, num_points);
                }
            });

            scope.execute(|| {
                for _ in progress_rx {
                    progress_bar.inc();
                }
            });

            for id in &parent_ids {
                let finished_nodes_sender_clone = finished_nodes_sender.clone();
                let progress_tx_clone = progress_tx.clone();
                scope.execute(move || {
                    subsample_children_into(
                        octree_data_provider,
                        octree_meta,
                        id,
                        &finished_nodes_sender_clone,
                    )
                    .unwrap();
                    progress_tx_clone.send(()).unwrap();
                });
            }
            drop(finished_nodes_sender);
            drop(progress_tx);
        });
        progress_bar.finish();

        // The nodes that were just now created through sub-sampling will be required to create
        // their parents.
        nodes_to_subsample.extend(parent_ids.into_iter());
    }

    // Add all non-zero node meta data to meta.pb
    let nodes: Vec<proto::OctreeNode> = finished_nodes
        .iter()
        .map(|(id, num_points)| {
            let bounding_cube = id.find_bounding_cube(&Cube::bounding(&octree_meta.bounding_box));
            let position_encoding = PositionEncoding::new(&bounding_cube, octree_meta.resolution);
            to_node_proto(&id, *num_points, &position_encoding)
        })
        .collect();
    let meta = to_meta_proto(&octree_meta, nodes);

    let mut buf_writer =
        BufWriter::new(File::create(&output_directory.as_ref().join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::Color;
    use crate::{AttributeData, Point};
    use cgmath::Vector3;
    use num_integer::div_ceil;
    use num_traits::identities::Zero;
    use std::collections::BTreeMap;
    use tempdir::TempDir;

    struct Points {
        points: Vec<Point>,
        point_count: usize,
    }

    impl Points {
        fn new(points: Vec<Point>) -> Self {
            Points {
                points,
                point_count: 0,
            }
        }
    }

    impl NumberOfPoints for Points {
        fn num_points(&self) -> usize {
            self.points.len()
        }
    }

    impl Iterator for Points {
        type Item = PointsBatch;

        fn next(&mut self) -> Option<PointsBatch> {
            if self.point_count == self.points.len() {
                return None;
            }
            let batch_size =
                std::cmp::min(NUM_POINTS_PER_BATCH, self.points.len() - self.point_count);
            let mut position = Vec::with_capacity(batch_size);
            let mut color = Vec::with_capacity(batch_size);
            let mut intensity = None;
            for _ in 0..batch_size {
                let point = &self.points[self.point_count];
                position.push(point.position);
                color.push(Vector3::new(
                    point.color.red,
                    point.color.green,
                    point.color.blue,
                ));
                if let Some(i) = point.intensity {
                    if intensity.is_none() {
                        intensity = Some(Vec::with_capacity(batch_size));
                    }
                    intensity.as_mut().unwrap().push(i);
                }
                self.point_count += 1;
            }
            let mut attributes = BTreeMap::new();
            attributes.insert("color".to_string(), AttributeData::U8Vec3(color));
            if let Some(intensity) = intensity {
                attributes.insert("intensity".to_string(), AttributeData::F32(intensity));
            }
            Some(PointsBatch {
                position,
                attributes,
            })
        }

        fn size_hint(&self) -> (usize, Option<usize>) {
            let num_batches = div_ceil(self.points.len(), NUM_POINTS_PER_BATCH);
            (num_batches, Some(num_batches))
        }
    }
    #[test]
    fn test_generation() {
        let default_point = Point {
            position: Vector3::zero(),
            color: Color {
                red: 255,
                green: 0,
                blue: 0,
                alpha: 255,
            },
            intensity: None,
        };
        let mut points = vec![default_point; 100_001];
        points[100_000].position = Vector3::new(2.0, 0.0, 0.0);
        let mut bounding_box = Aabb3::zero();
        for point in &points {
            bounding_box = bounding_box.grow(Point3::from_vec(point.position));
        }
        let pool = scoped_pool::Pool::new(10);
        let tmp_dir = TempDir::new("octree").unwrap();
        build_octree(&pool, tmp_dir, 1.0, bounding_box, Points::new(points));
    }
}
