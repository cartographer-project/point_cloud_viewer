extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate fnv;
extern crate image;
extern crate point_viewer;
extern crate protobuf;
extern crate quadtree;
extern crate scoped_pool;
extern crate xray;

use cgmath::{Point2, Point3};
use collision::{Aabb, Aabb2, Aabb3};
use octree::OnDiskOctree;
use point_viewer::octree;
use std::error::Error;
use std::path::Path;
use xray::generation::{xray_from_points, ColoringStrategyArgument, ColoringStrategyKind};

fn parse_arguments() -> clap::ArgMatches<'static> {
    // TODO(sirver): pull out a function for common args.
    clap::App::new("build_xray_tile")
        .version("1.0")
        .author("Holger H. Rapp <hrapp@lyft.com>")
        .args(&[
            clap::Arg::with_name("output_filename")
                .help("Output filename to write into.")
                .default_value("output.png")
                .long("output_filename")
                .takes_value(true),
            clap::Arg::with_name("resolution")
                .help("Size of 1px in meters.")
                .long("resolution")
                .default_value("0.05"),
            clap::Arg::with_name("coloring_strategy")
                .long("coloring_strategy")
                .takes_value(true)
                .possible_values(&ColoringStrategyArgument::variants())
                .default_value("xray"),
            clap::Arg::with_name("min_intensity")
                .help(
                    "Minimum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("min_intensity")
                .takes_value(true)
                .required_if("coloring_strategy", "colored_with_intensity"),
            clap::Arg::with_name("max_intensity")
                .help(
                    "Minimum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("max_intensity")
                .takes_value(true)
                .required_if("coloring_strategy", "colored_with_intensity"),
            clap::Arg::with_name("max_stddev")
                .help(
                    "Maximum stddev for colored_with_height_stddev. Every stddev above this \
                     will be clamped to this value and appear saturated in the X-Rays. \
                     Only used for 'colored_with_height_stddev'.",
                )
                .long("max_stddev")
                .takes_value(true)
                .required_if("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::with_name("octree_directory")
                .help("Octree directory to turn into xrays.")
                .index(1)
                .required(true),
            clap::Arg::with_name("min_x")
                .long("min_x")
                .takes_value(true)
                .help("Bounding box minimum x in meters.")
                .required(true),
            clap::Arg::with_name("min_y")
                .long("min_y")
                .takes_value(true)
                .help("Bounding box minimum y in meters.")
                .required(true),
            clap::Arg::with_name("max_x")
                .long("max_x")
                .takes_value(true)
                .help("Bounding box maximum x in meters.")
                .required(true),
            clap::Arg::with_name("max_y")
                .long("max_y")
                .takes_value(true)
                .help("Bounding box maximum y in meters.")
                .required(true),
        ])
        .get_matches()
}

fn run(
    octree_directory: &Path,
    output_filename: &Path,
    resolution: f32,
    coloring_strategy_kind: ColoringStrategyKind,
    bbox2: &Aabb2<f32>,
) -> Result<(), Box<Error>> {
    let octree = &OnDiskOctree::new(octree_directory)?;
    let bbox3 = octree.bounding_box();
    let bbox3 = Aabb3::new(
        Point3::new(
            bbox2.min().x.max(bbox3.min().x),
            bbox2.min().y.max(bbox3.min().y),
            bbox3.min().z,
        ),
        Point3::new(
            bbox2.max().x.min(bbox3.max().x),
            bbox2.max().y.min(bbox3.max().y),
            bbox3.max().z,
        ),
    );
    let image_width = (bbox2.dim().x / resolution).ceil() as u32;
    let image_height = (bbox2.dim().y / resolution).ceil() as u32;
    if !xray_from_points(
        octree,
        &bbox3,
        output_filename,
        image_width,
        image_height,
        coloring_strategy_kind.new_strategy(),
    ) {
        println!("No points in bounding box. No output written.");
    }
    Ok(())
}

pub fn main() {
    let matches = parse_arguments();
    let resolution = value_t!(matches, "resolution", f32).expect("resolution could not be parsed.");
    let coloring_strategy_kind = {
        use ColoringStrategyArgument::*;
        let arg = value_t!(matches, "coloring_strategy", ColoringStrategyArgument)
            .expect("coloring_strategy is invalid");
        match arg {
            xray => ColoringStrategyKind::XRay,
            colored => ColoringStrategyKind::Colored,
            colored_with_intensity => ColoringStrategyKind::ColoredWithIntensity(
                value_t!(matches, "min_intensity", f32).unwrap_or(1.),
                value_t!(matches, "max_intensity", f32).unwrap_or(1.),
            ),
            colored_with_height_stddev => ColoringStrategyKind::ColoredWithHeightStddev(
                value_t!(matches, "max_stddev", f32).unwrap_or(1.),
            ),
        }
    };
    let octree_directory = Path::new(matches.value_of("octree_directory").unwrap());
    let output_filename = Path::new(matches.value_of("output_filename").unwrap());
    let min_x = value_t!(matches, "min_x", f32).expect("min_x could not be parsed.");
    let min_y = value_t!(matches, "min_y", f32).expect("min_y could not be parsed.");
    let max_x = value_t!(matches, "max_x", f32).expect("max_x could not be parsed.");
    let max_y = value_t!(matches, "max_y", f32).expect("max_y could not be parsed.");

    let bbox2 = Aabb2::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y));
    run(
        octree_directory,
        output_filename,
        resolution,
        coloring_strategy_kind,
        &bbox2,
    ).unwrap();
}
