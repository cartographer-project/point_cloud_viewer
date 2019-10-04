#![feature(test)]

extern crate test;

use lazy_static::lazy_static;
use point_viewer::octree::build_octree;
use point_viewer::read_write::{Encoding, NodeWriter, OpenMode, RawNodeWriter, S2Splitter};
use protobuf::Message;
use random_points::{Batched, RandomPointsOnEarth};
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use tempdir::TempDir;
use test::Bencher;

mod random_points;

#[derive(Debug)]
struct Arguments {
    // Minimal precision that this point cloud should have.
    // This decides on the number of bits used to encode each node.
    resolution: f64,
    // Width of the area to generate points for (used for local x and y), in meters.
    width: f64,
    // Height of the area to generate points for, in meters
    height: f64,
    // The number of threads used to shard octree building.
    num_threads: usize,
    // The number of points in the test point cloud.
    num_points: usize,
    // The batch size used for building the S2 point cloud.
    batch_size: usize,
}

impl Default for Arguments {
    fn default() -> Arguments {
        Arguments {
            resolution: 0.001,
            width: 200.0,
            height: 20.0,
            num_threads: 10,
            num_points: 1_000_000,
            batch_size: 5000,
        }
    }
}

fn make_octree(args: &Arguments, dir: &Path) {
    let mut points_oct = RandomPointsOnEarth::new(args.width, args.height, args.num_points);
    let bbox = points_oct.bbox();
    let batches_oct = Batched::new(points_oct, args.batch_size);
    let pool = scoped_pool::Pool::new(args.num_threads);

    build_octree(&pool, dir, args.resolution, bbox, batches_oct);
}

fn make_s2_cells(args: &Arguments, dir: &Path) {
    let points_s2 = RandomPointsOnEarth::new(args.width, args.height, args.num_points);
    let mut s2_writer: S2Splitter<RawNodeWriter> =
        S2Splitter::new(dir, Encoding::Plain, OpenMode::Truncate);
    Batched::new(points_s2, args.batch_size)
        .try_for_each(|batch| s2_writer.write(&batch))
        .expect("Writing failed");
    let meta = s2_writer.get_meta().to_proto();
    let mut meta_writer = BufWriter::new(File::create(dir.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut meta_writer).unwrap();
}

lazy_static! {
    static ref OCTREE_DIR: TempDir = {
        let temp_dir = TempDir::new("octree").unwrap();
        let args = Arguments::default();
        make_octree(&args, temp_dir.path());
        temp_dir
    };
    static ref S2_DIR: TempDir = {
        let temp_dir = TempDir::new("s2").unwrap();
        let args = Arguments::default();
        make_s2_cells(&args, temp_dir.path());
        temp_dir
    };
}

#[bench]
fn bench_octree_building_multithreaded(b: &mut Bencher) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    b.iter(|| {
        let temp_dir = TempDir::new("octree").unwrap();
        make_octree(&args, temp_dir.path());
    });
}

#[bench]
fn bench_s2_building_singlethreaded(b: &mut Bencher) {
    let mut args = Arguments::default();
    args.num_points = 100_000;
    b.iter(|| {
        let temp_dir = TempDir::new("s2").unwrap();
        make_s2_cells(&args, temp_dir.path());
    });
}

#[test]
fn num_points_in_octree() {
    let path_buf = OCTREE_DIR.path().to_owned();
    use point_viewer::data_provider::{DataProvider, OnDiskDataProvider};
    let data_provider = OnDiskDataProvider {
        directory: path_buf,
    };
    let meta = data_provider.meta_proto().unwrap();
    assert!(meta.has_octree());
    let num_points: i64 = meta
        .get_octree()
        .get_nodes()
        .iter()
        .map(|n| n.num_points)
        .sum();
    assert_eq!(num_points, Arguments::default().num_points as i64);
}

#[test]
fn num_points_in_s2() {
    let path_buf = S2_DIR.path().to_owned();
    use point_viewer::data_provider::{DataProvider, OnDiskDataProvider};
    let data_provider = OnDiskDataProvider {
        directory: path_buf,
    };
    let meta = data_provider.meta_proto().unwrap();
    assert!(meta.has_s2());
    let num_points: u64 = meta.get_s2().get_cells().iter().map(|c| c.num_points).sum();
    assert_eq!(num_points, Arguments::default().num_points as u64);
}
