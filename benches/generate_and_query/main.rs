#![feature(test)]

extern crate test;

use cgmath::{InnerSpace, PerspectiveFov, Rad, Vector2, Vector3};
use lazy_static::lazy_static;
use num_integer::div_ceil;
use point_viewer::data_provider::{DataProvider, OnDiskDataProvider};
use point_viewer::iterator::{PointCloud, PointLocation, PointQuery};
use point_viewer::math::{Frustum, OrientedBeam};
use point_viewer::octree::{build_octree, Octree};
use point_viewer::read_write::{Encoding, NodeWriter, OpenMode, RawNodeWriter, S2Splitter};
use point_viewer::s2_cells::S2Cells;
use protobuf::Message;
use random_points::{Batched, RandomPointsOnEarth};
use std::cmp::Ordering;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;
use tempdir::TempDir;
use test::Bencher;

mod random_points;

const RESOLUTION: f64 = 0.001;

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
    // The seed used for generating point clouds
    seed: u64,
}

impl Default for Arguments {
    fn default() -> Arguments {
        Arguments {
            resolution: RESOLUTION,
            width: 200.0,
            height: 20.0,
            num_threads: 10,
            num_points: 1_000_000,
            batch_size: 5000,
            seed: 80_293_751_232,
        }
    }
}

struct IndexedPoint {
    idx: usize,
    pos: Vector3<f64>,
}

fn make_octree(args: &Arguments, dir: &Path) {
    let points_oct = RandomPointsOnEarth::new(args.width, args.height, args.num_points, args.seed);
    let bbox = points_oct.bbox();
    let batches_oct = Batched::new(points_oct, args.batch_size);
    let pool = scoped_pool::Pool::new(args.num_threads);

    build_octree(&pool, dir, args.resolution, bbox, batches_oct, &["color"]);
}

fn make_s2_cells(args: &Arguments, dir: &Path) {
    let points_s2 = RandomPointsOnEarth::new(args.width, args.height, args.num_points, args.seed);
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

fn setup() -> (Arguments, S2Cells, Octree, RandomPointsOnEarth) {
    let args = Arguments::default();
    let s2_path_buf = S2_DIR.path().to_owned();
    let s2_data_provider = OnDiskDataProvider {
        directory: s2_path_buf,
    };
    let s2 = S2Cells::from_data_provider(Box::new(s2_data_provider)).unwrap();
    let oct_path_buf = OCTREE_DIR.path().to_owned();
    let oct_data_provider = OnDiskDataProvider {
        directory: oct_path_buf,
    };
    let oct = Octree::from_data_provider(Box::new(oct_data_provider)).unwrap();
    let points = RandomPointsOnEarth::new(10.0, 10.0, args.num_points, args.seed);
    (args, s2, oct, points)
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

fn cmp_points(p1: &IndexedPoint, p2: &IndexedPoint) -> std::cmp::Ordering {
    p1.idx.cmp(&p2.idx)
}

fn query_and_sort<C>(point_cloud: &C, query: &PointQuery, batch_size: usize) -> Vec<IndexedPoint>
where
    C: PointCloud,
{
    let mut points = Vec::new();
    for node_id in point_cloud.nodes_in_location(query).into_iter() {
        let points_iter = point_cloud
            .points_in_node(query, node_id, batch_size)
            .unwrap();
        points.extend(points_iter.flat_map(|batch| {
            let color: &Vec<Vector3<u8>> = batch.get_attribute_vec("color").unwrap();
            color
                .iter()
                .zip(batch.position.iter())
                .map(|(c, p)| {
                    let idx = ((c.x as usize) << 16) + ((c.y as usize) << 8) + c.z as usize;
                    IndexedPoint { idx, pos: *p }
                })
                .collect::<Vec<IndexedPoint>>()
        }));
    }
    points.sort_unstable_by(cmp_points);
    assert!(!points.is_empty());
    points
}

// Checks that the points are equal up to a precision of the default resolution
fn assert_points_equal(points_s2: &[IndexedPoint], points_oct: &[IndexedPoint]) {
    let args = Arguments::default();
    let mut num_skipped = 0;
    // If a point is allowed to be displaced by up to args.resolution in each dimension,
    // the distance from the true position may be up to 2 * resolution * sqrt(3).
    // We take 2 * resolution, since we can get errors when encoding and decoding.
    let threshold = 3.0_f64.sqrt() * 2.0 * args.resolution;
    let mut s2_iter = points_s2.iter().enumerate();
    let mut oct_iter = points_oct.iter().enumerate();
    let mut s2_next = s2_iter.next();
    let mut oct_next = oct_iter.next();

    while let (Some((count_s2, p_s2)), Some((count_oct, p_oct))) = (s2_next, oct_next) {
        match cmp_points(p_s2, p_oct) {
            Ordering::Equal => {
                let distance = (p_s2.pos - p_oct.pos).magnitude();
                assert!(
                    distance <= threshold,
                    "Inequality between s2 point [{}]: {:?} and octree point [{}]: {:?}, distance {}",
                    count_s2,
                    p_s2.pos,
                    count_oct,
                    p_oct.pos,
                    distance
                );
                s2_next = s2_iter.next();
                oct_next = oct_iter.next();
            }
            Ordering::Less => {
                num_skipped += 1;
                s2_next = s2_iter.next();
            }
            Ordering::Greater => {
                num_skipped += 1;
                oct_next = oct_iter.next();
            }
        }
    }
    assert!(
        num_skipped <= div_ceil(std::cmp::min(points_s2.len(), points_oct.len()), 100),
        "More than 1% point index mismatches."
    );
}

#[test]
fn check_all_query_equality() {
    let (args, s2, oct, _points) = setup();
    let query = PointQuery {
        attributes: vec!["color"],
        location: PointLocation::AllPoints,
        global_from_local: None,
    };
    let points_s2 = query_and_sort(&s2, &query, args.batch_size);
    let points_oct = query_and_sort(&oct, &query, args.batch_size);
    assert_points_equal(&points_s2, &points_oct);
}

#[test]
fn check_box_query_equality() {
    let (args, s2, oct, points) = setup();
    let bbox = points.bbox();
    let query = PointQuery {
        attributes: vec!["color"],
        location: PointLocation::Aabb(bbox),
        global_from_local: None,
    };
    let points_s2 = query_and_sort(&s2, &query, args.batch_size);
    let points_oct = query_and_sort(&oct, &query, args.batch_size);
    assert_points_equal(&points_s2, &points_oct);
}

#[test]
fn check_frustum_query_equality() {
    let (args, s2, oct, points) = setup();
    let ecef_from_local = points.ecef_from_local().clone();
    let perspective = PerspectiveFov {
        fovy: Rad(1.2),
        aspect: 1.0,
        near: 0.1,
        far: 10.0,
    }
    .to_perspective();
    let frustum = Frustum::new(ecef_from_local, perspective);
    let query = PointQuery {
        attributes: vec!["color"],
        location: PointLocation::Frustum(frustum),
        global_from_local: None,
    };
    let points_s2 = query_and_sort(&s2, &query, args.batch_size);
    let points_oct = query_and_sort(&oct, &query, args.batch_size);
    assert_points_equal(&points_s2, &points_oct);
}

#[test]
fn check_beam_query_equality() {
    let (args, s2, oct, points) = setup();
    let transform = points.ecef_from_local();
    let beam = OrientedBeam::new(transform.clone(), Vector2::new(15.0, 15.0));
    let query = PointQuery {
        attributes: vec!["color"],
        location: PointLocation::OrientedBeam(beam),
        global_from_local: None,
    };
    let points_oct = query_and_sort(&oct, &query, args.batch_size);
    let points_s2 = query_and_sort(&s2, &query, args.batch_size);
    assert_points_equal(&points_s2, &points_oct);
}
