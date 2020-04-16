use anyhow::Result;
use point_viewer::data_provider::{DataProvider, OnDiskDataProvider};
use point_viewer::geometry::Frustum;
use point_viewer::octree::Octree;
use std::fs::File;
use std::io::{self, BufRead, BufReader};
use std::path::PathBuf;
use std::time::Instant;
use structopt::StructOpt;

#[derive(Debug, StructOpt)]
#[structopt(name = "example", about = "An example of StructOpt usage.")]
struct Opt {
    /// Point cloud
    #[structopt(parse(from_os_str))]
    point_cloud_dir: PathBuf,

    /// Recorded queries from that point cloud as JSON
    #[structopt(parse(from_os_str))]
    frustums_filename: PathBuf,

    #[structopt(short = "i", default_value = "1000")]
    iterations: u32,
}

fn main() -> Result<()> {
    let opt = Opt::from_args();
    println!("{:?}", opt);
    let f = File::open(&opt.frustums_filename)?;
    let mut f = BufReader::new(f);
    let frustums: Vec<Frustum<f64>> = serde_json::from_reader(&mut f)?;
    let data_provider = Box::new(OnDiskDataProvider {
        directory: opt.point_cloud_dir,
    }) as Box<dyn DataProvider>;
    let point_cloud = Octree::from_data_provider(data_provider).unwrap();

    for frustum in &frustums {
        let node_ids_old = point_cloud.get_visible_nodes(frustum.as_matrix());
        let node_ids_new = point_cloud.get_visible_nodes_simple(frustum.as_matrix());
        assert_eq!(node_ids_old, node_ids_new);
    }

    let mut now = Instant::now();
    let x = with_get_visible_nodes(&point_cloud, opt.iterations, &frustums);
    println!("get_visible_nodes: {:?} {}", now.elapsed(), x);
    now = Instant::now();
    let x = with_get_visible_nodes_simple(&point_cloud, opt.iterations, &frustums);
    println!("get_visible_nodes_simple: {:?} {}", now.elapsed(), x);
    now = Instant::now();
    let y = with_nodes_in_location_impl(&point_cloud, opt.iterations, &frustums);
    println!("nodes_in_location_impl: {:?} {}", now.elapsed(), y);

    Ok(())
}

#[inline(never)]
fn with_get_visible_nodes(
    point_cloud: &Octree,
    iterations: u32,
    frustums: &Vec<Frustum<f64>>,
) -> usize {
    // Record this to prevent optimization
    let mut total_length = 0;
    for _ in 0..iterations {
        for frustum in frustums {
            let node_ids = point_cloud.get_visible_nodes(frustum.as_matrix());
            total_length += node_ids.len();
        }
        total_length = total_length % 100;
    }
    total_length
}

#[inline(never)]
fn with_get_visible_nodes_simple(
    point_cloud: &Octree,
    iterations: u32,
    frustums: &Vec<Frustum<f64>>,
) -> usize {
    // Record this to prevent optimization
    let mut total_length = 0;
    for _ in 0..iterations {
        for frustum in frustums {
            let node_ids = point_cloud.get_visible_nodes_simple(frustum.as_matrix());
            total_length += node_ids.len();
        }
        total_length = total_length % 100;
    }
    total_length
}

#[inline(never)]
fn with_nodes_in_location_impl(
    point_cloud: &Octree,
    iterations: u32,
    frustums: &Vec<Frustum<f64>>,
) -> usize {
    // Record this to prevent optimization
    let mut total_length = 0;
    for _ in 0..iterations {
        for frustum in frustums {
            let node_ids = point_cloud.nodes_in_location_impl(frustum);
            total_length += node_ids.len();
        }
        total_length = total_length % 100;
    }
    total_length
}
