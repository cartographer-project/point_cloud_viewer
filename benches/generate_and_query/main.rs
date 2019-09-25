#![feature(test)]

extern crate test;

use lazy_static::lazy_static;
use point_viewer::data_provider::{DataProvider, OnDiskDataProvider};
use point_viewer::iterator::{PointCloud, PointLocation, PointQuery};
use point_viewer::octree::{build_octree, Octree};
use point_viewer::read_write::{Encoding, NodeWriter, OpenMode, RawNodeWriter, S2Splitter};
use point_viewer::s2_cells::S2Cells;
use point_viewer::Point;
use protobuf::Message;
use random_points::{Batched, RandomPointsOnEarth, SEED};
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
    let mut points_oct = RandomPointsOnEarth::new(args.width, args.height, SEED);
    let pool = scoped_pool::Pool::new(args.num_threads);

    build_octree(
        &pool,
        dir,
        args.resolution,
        points_oct.bbox(),
        points_oct.take(args.num_points),
    );
}

fn make_s2_cells(args: &Arguments, dir: &Path) {
    let points_s2 = RandomPointsOnEarth::new(args.width, args.height, SEED);
    let mut s2_writer: S2Splitter<RawNodeWriter> =
        S2Splitter::new(dir, Encoding::Plain, OpenMode::Truncate);
    let points_s2 = points_s2.take(args.num_points);
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
fn num_points_in_octree_meta() {
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
fn num_points_in_s2_meta() {
    let path_buf = S2_DIR.path().to_owned();
    let data_provider = OnDiskDataProvider {
        directory: path_buf,
    };
    let meta = data_provider.meta_proto().unwrap();
    assert!(meta.has_s2());
    let num_points: u64 = meta.get_s2().get_cells().iter().map(|c| c.num_points).sum();
    assert_eq!(num_points, Arguments::default().num_points as u64);
}

fn query_and_sort<C>(point_cloud: &C, query: &PointQuery) -> Vec<Point>
where
    C: PointCloud,
{
    let mut points = Vec::new();
    for node_id in point_cloud.nodes_in_location(query).into_iter() {
        let points_iter = point_cloud.points_in_node(query, node_id).unwrap();
        points.extend(points_iter);
    }
    points.sort_unstable_by(|p1, p2| {
        let x_order = p1.position.x.partial_cmp(&p2.position.x).unwrap();
        let y_order = p1.position.y.partial_cmp(&p2.position.y).unwrap();
        let z_order = p1.position.z.partial_cmp(&p2.position.z).unwrap();
        x_order.then(y_order).then(z_order)
    });
    points
}

fn get_s2_cells() -> S2Cells {
    let path_buf = S2_DIR.path().to_owned();
    let data_provider = OnDiskDataProvider {
        directory: path_buf,
    };
    S2Cells::from_data_provider(Box::new(data_provider)).unwrap()
}

fn get_octree() -> Octree {
    let path_buf = OCTREE_DIR.path().to_owned();
    let data_provider = OnDiskDataProvider {
        directory: path_buf,
    };
    Octree::from_data_provider(Box::new(data_provider)).unwrap()
}

#[test]
fn check_box_query_equality() {
    let s2 = get_s2_cells();
    let oct = get_octree();

    let bbox = RandomPointsOnEarth::new(4.0, 4.0, SEED).bbox();
    let query = PointQuery {
        location: PointLocation::Aabb(bbox),
        global_from_local: None,
    };
    let points_s2 = query_and_sort(&s2, &query);
    let points_oct = query_and_sort(&oct, &query);
    assert_eq!(points_s2.len(), points_oct.len());
    for (p_s2, p_oct) in points_s2.iter().zip(points_oct.iter()) {
        println!("Aye");
        assert!(p_s2.position.eq(&p_oct.position), "s2 point: {:?}, octree point: {:?}", p_s2.position, p_oct.position)
    }
}
