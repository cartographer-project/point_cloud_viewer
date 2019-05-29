// This removes clippy warnings regarding redundant StructOpt.
#![allow(clippy::redundant_closure)]

use cgmath::Point3;
use collision::Aabb3;
use point_cloud_client::PointCloudClient;
use point_viewer::errors::*;
use point_viewer::octree::{OctreeFactory, PointLocation};
use point_viewer::PointData;
use std::sync::Arc;
use structopt::StructOpt;

fn point3f64_from_str(s: &str) -> std::result::Result<Point3<f64>, &'static str> {
    let coords: std::result::Result<Vec<f64>, &'static str> = s
        .split(|c| c == ' ' || c == ',' || c == ';')
        .map(|s| s.parse::<f64>().map_err(|_| "Could not parse point."))
        .collect();
    let coords = coords?;
    if coords.len() != 3 {
        return Err("Wrong number of coordinates.");
    }
    Ok(Point3::new(coords[0], coords[1], coords[2]))
}

#[derive(StructOpt)]
#[structopt(about = "Simple point_cloud_client test.")]
struct CommandlineArguments {
    /// The locations containing the octree data.
    #[structopt(parse(from_str), raw(required = "true"))]
    locations: Vec<String>,

    /// The minimum value of the bounding box to be queried.
    #[structopt(
        long = "min",
        default_value = "-500.0 -500.0 -500.0",
        parse(try_from_str = "point3f64_from_str")
    )]
    min: Point3<f64>,

    /// The maximum value of the bounding box to be queried.
    #[structopt(
        long = "max",
        default_value = "500.0 500.0 500.0",
        parse(try_from_str = "point3f64_from_str")
    )]
    max: Point3<f64>,

    /// The maximum number of points to return.
    #[structopt(long = "num-points", default_value = "50000000")]
    num_points: usize,
}

fn main() {
    let args = CommandlineArguments::from_args();
    let num_points = args.num_points;
    let point_cloud_client = PointCloudClient::new(&args.locations, OctreeFactory::new())
        .expect("Couldn't create octree client.");
    let point_location = PointLocation {
        culling: Arc::new(Box::new(Aabb3::new(args.min, args.max))),
        global_from_local: None,
    };
    let mut point_count: usize = 0;
    let mut print_count: usize = 1;
    let callback_func = |point_data: PointData| -> Result<()> {
        point_count += point_data.position.len();
        if point_count >= print_count * 1_000_000 {
            print_count += 1;
            println!("Streamed {}M points", point_count / 1_000_000);
        }
        if point_count >= num_points {
            return Err(std::io::Error::new(
                std::io::ErrorKind::Interrupted,
                format!("Maximum number of {} points reached.", num_points),
            )
            .into());
        }
        Ok(())
    };
    match point_cloud_client.for_each_point_data(&point_location, callback_func) {
        Ok(_) => (),
        Err(e) => match e.kind() {
            ErrorKind::Io(ref e) if e.kind() == std::io::ErrorKind::Interrupted => (),
            _ => {
                println!("Encountered error:\n{}", e);
                std::process::exit(1);
            }
        },
    }
}
