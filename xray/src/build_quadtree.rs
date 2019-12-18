use crate::generation::{
    build_xray_quadtree, ColoringStrategyArgument, ColoringStrategyKind, Tile,
    TileBackgroundColorArgument,
};
use clap::value_t;
use point_cloud_client::PointCloudClient;
use point_viewer::color::{TRANSPARENT, WHITE};
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::math::Isometry3;
use point_viewer::read_write::attempt_increasing_rlimit_to_max;
use scoped_pool::Pool;
use std::path::Path;

pub trait Extension {
    fn pre_init<'a, 'b>(app: clap::App<'a, 'b>) -> clap::App<'a, 'b>;
    fn query_from_global(matches: &clap::ArgMatches) -> Option<Isometry3<f64>>;
}

fn parse_arguments<T: Extension>() -> clap::ArgMatches<'static> {
    let mut app = clap::App::new("build_xray_quadtree")
        .version("1.0")
        .author("Holger H. Rapp <hrapp@lyft.com>")
        .args(&[
            clap::Arg::with_name("output_directory")
                .help("Output directory to write the X-Ray quadtree into.")
                .long("output_directory")
                .required(true)
                .takes_value(true),
            clap::Arg::with_name("resolution")
                .help("Size of 1px in meters on the finest X-Ray level.")
                .long("resolution")
                .default_value("0.01"),
            clap::Arg::with_name("num_threads")
                .help("The number of threads used to shard X-Ray tile building.")
                .takes_value(true)
                .long("num_threads")
                .default_value("10"),
            clap::Arg::with_name("tile_size")
                .help("Size of finest X-Ray level tile in pixels. Must be a power of two.")
                .long("tile_size")
                .default_value("256"),
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
            clap::Arg::with_name("max_stddev")
                .help(
                    "Maximum standard deviation for colored_with_height_stddev. Every stddev above this \
                     will be clamped to this value and appear saturated in the X-Rays. \
                     Only used for 'colored_with_height_stddev'.",
                )
                .long("max_stddev")
                .takes_value(true)
                .required_if("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::with_name("max_intensity")
                .help(
                    "Minimum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("max_intensity")
                .takes_value(true)
                .required_if("coloring_strategy", "colored_with_intensity"),
            clap::Arg::with_name("octree_locations")
                .help("Octree locations to turn into xrays.")
                .index(1)
                .multiple(true)
                .required(true),
            clap::Arg::with_name("tile_background_color")
                .long("tile_background_color")
                .takes_value(true)
                .possible_values(&TileBackgroundColorArgument::variants())
                .default_value("white"),
        ]);
    app = T::pre_init(app);
    app.get_matches()
}

pub fn run<T: Extension>(data_provider_factory: DataProviderFactory) {
    attempt_increasing_rlimit_to_max();

    let args = parse_arguments::<T>();
    let resolution = args
        .value_of("resolution")
        .unwrap()
        .parse::<f64>()
        .expect("resolution could not be parsed.");
    let num_threads = args
        .value_of("num_threads")
        .unwrap()
        .parse::<usize>()
        .expect("num_threads could not be parsed.");
    let tile_size = args
        .value_of("tile_size")
        .unwrap()
        .parse::<u32>()
        .expect("tile_size could not be parsed.");
    if !tile_size.is_power_of_two() {
        panic!("tile_size is not a power of two.");
    }

    let coloring_strategy_kind = {
        use ColoringStrategyArgument::*;
        let arg = value_t!(args, "coloring_strategy", ColoringStrategyArgument)
            .expect("coloring_strategy is invalid");
        match arg {
            xray => ColoringStrategyKind::XRay,
            colored => ColoringStrategyKind::Colored,
            colored_with_intensity => ColoringStrategyKind::ColoredWithIntensity(
                value_t!(args, "min_intensity", f32).unwrap_or(1.),
                value_t!(args, "max_intensity", f32).unwrap_or(1.),
            ),
            colored_with_height_stddev => ColoringStrategyKind::ColoredWithHeightStddev(
                value_t!(args, "max_stddev", f32).unwrap_or(1.),
            ),
        }
    };

    let tile_background_color = {
        let arg = value_t!(args, "tile_background_color", TileBackgroundColorArgument)
            .expect("tile_background_color is invalid");
        match arg {
            TileBackgroundColorArgument::white => WHITE.to_u8(),
            TileBackgroundColorArgument::transparent => TRANSPARENT.to_u8(),
        }
    };

    let output_directory = Path::new(args.value_of("output_directory").unwrap());

    let pool = Pool::new(num_threads);

    let octree_locations = args
        .values_of("octree_locations")
        .unwrap()
        .map(String::from)
        .collect::<Vec<_>>();
    let point_cloud_client = PointCloudClient::new(&octree_locations, data_provider_factory)
        .expect("Could not create point cloud client.");

    let query_from_global = T::query_from_global(&args);
    build_xray_quadtree(
        &pool,
        &point_cloud_client,
        &query_from_global,
        output_directory,
        &Tile {
            size_px: tile_size,
            resolution,
        },
        &coloring_strategy_kind,
        tile_background_color,
    )
    .unwrap();
}
