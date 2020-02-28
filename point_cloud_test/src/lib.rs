use point_cloud_client::{PointCloudClient, PointCloudClientBuilder};
/// This module has functions to generate synthetic point clouds in a temp dir
/// and provides queries on these synthetic point clouds.
use point_viewer::data_provider::{DataProviderFactory, OnDiskDataProvider};
use point_viewer::octree::{build_octree, Octree};
use point_viewer::read_write::{Encoding, NodeWriter, OpenMode, RawNodeWriter, S2Splitter};
use point_viewer::s2_cells::S2Cells;
use protobuf::Message;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Once;
use tempdir::TempDir;

mod synthetic_data;
pub use synthetic_data::{Batched, SyntheticData};

mod queries;
pub use queries::{get_abb_query, get_cell_union_query, get_frustum_query, get_obb_query};

pub const S2_LEVEL: u64 = 20;

#[derive(Clone, Debug, PartialEq)]
pub struct Arguments {
    // Minimal precision that this point cloud should have.
    // This decides on the number of bits used to encode each node.
    pub resolution: f64,
    // Width of the area to generate points for (used for local x and y), in meters.
    pub width: f64,
    // Height of the area to generate points for, in meters
    pub height: f64,
    // The number of points in the test point cloud.
    pub num_points: usize,
    // The batch size used for building the S2 point cloud.
    pub batch_size: usize,
    // The seed used for generating point clouds
    pub seed: u64,
}

impl Eq for Arguments {}

impl Default for Arguments {
    fn default() -> Arguments {
        Arguments {
            resolution: 0.001,
            width: 200.0,
            height: 20.0,
            num_points: 1_000_000,
            batch_size: 5000,
            seed: 80_293_751_232,
        }
    }
}

pub fn make_octree(args: &Arguments, dir: &Path) {
    let points_oct = SyntheticData::new(args.width, args.height, args.num_points, args.seed);
    let bbox = points_oct.bbox();
    let batches_oct = Batched::new(points_oct, args.batch_size);

    build_octree(dir, args.resolution, bbox, batches_oct, &["color"]);
}

pub fn make_s2_cells(args: &Arguments, dir: &Path) {
    let points_s2 = SyntheticData::new(args.width, args.height, args.num_points, args.seed);
    let mut s2_writer: S2Splitter<RawNodeWriter> =
        S2Splitter::with_split_level(S2_LEVEL, dir, Encoding::Plain, OpenMode::Truncate);
    Batched::new(points_s2, args.batch_size)
        .try_for_each(|batch| s2_writer.write(&batch))
        .expect("Writing failed");
    let meta = s2_writer.get_meta().to_proto();
    let mut meta_writer = BufWriter::new(File::create(dir.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut meta_writer).unwrap();
}

static INIT: Once = Once::new();
static mut OCTREE_DIR: Option<TempDir> = None;
static mut S2_DIR: Option<TempDir> = None;
static mut ARGUMENTS: Option<Arguments> = None;

pub fn get_s2_and_octree_path(args: &Arguments) -> (PathBuf, PathBuf, SyntheticData) {
    let (s2_path_buf, octree_path_buf) = unsafe {
        INIT.call_once(|| {
            let octree_dir = TempDir::new("octree").unwrap();
            make_octree(&args, octree_dir.path());
            OCTREE_DIR = Some(octree_dir);
            let s2_dir = TempDir::new("s2").unwrap();
            make_s2_cells(&args, s2_dir.path());
            S2_DIR = Some(s2_dir);
            ARGUMENTS = Some(args.clone());
        });
        // If somebody called this twice with different arguments, caching doesn't make sense
        assert_eq!(ARGUMENTS, Some(args.clone()));
        let s2_path_buf = S2_DIR.as_ref().unwrap().path().to_owned();
        let octree_path_buf = OCTREE_DIR.as_ref().unwrap().path().to_owned();
        (s2_path_buf, octree_path_buf)
    };
    let data = SyntheticData::new(args.width, args.height, args.num_points, args.seed);
    (s2_path_buf, octree_path_buf, data)
}

pub fn setup_pointcloud(args: &Arguments) -> (S2Cells, Octree, SyntheticData) {
    let (s2_path_buf, oct_path_buf, data) = get_s2_and_octree_path(args);
    let s2_data_provider = OnDiskDataProvider {
        directory: s2_path_buf,
    };
    let s2 = S2Cells::from_data_provider(Box::new(s2_data_provider)).unwrap();
    let oct_data_provider = OnDiskDataProvider {
        directory: oct_path_buf,
    };
    let oct = Octree::from_data_provider(Box::new(oct_data_provider)).unwrap();
    (s2, oct, data)
}

pub fn setup_s2_client(args: &Arguments) -> (PointCloudClient, SyntheticData) {
    let (s2_path_buf, _, data) = get_s2_and_octree_path(args);
    let s2_locations = &[s2_path_buf.to_str().unwrap().to_owned()];
    let client = PointCloudClientBuilder::new(s2_locations, DataProviderFactory::new())
        .build()
        .unwrap();
    (client, data)
}

pub fn setup_octree_client(args: &Arguments) -> (PointCloudClient, SyntheticData) {
    let (_, oct_path_buf, data) = get_s2_and_octree_path(args);
    let octree_locations = &[oct_path_buf.to_str().unwrap().to_owned()];
    let client = PointCloudClientBuilder::new(octree_locations, DataProviderFactory::new())
        .build()
        .unwrap();
    (client, data)
}
