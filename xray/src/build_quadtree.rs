use crate::generation::{
    build_xray_quadtree, ColoringStrategyArgument, ColoringStrategyKind, ColormapArgument,
    TileBackgroundColorArgument, XrayParameters,
};
use clap::{crate_authors, ArgEnum};
use nalgebra::Isometry3;
use point_cloud_client::PointCloudClientBuilder;
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::math::ClosedInterval;
use point_viewer::read_write::attempt_increasing_rlimit_to_max;
use point_viewer::utils::parse_key_val;
use quadtree::NodeId;
use std::collections::HashMap;
use std::path::PathBuf;

pub trait Extension {
    fn pre_init(app: clap::App) -> clap::App;
    fn query_from_global(matches: &clap::ArgMatches) -> Option<Isometry3<f64>>;
}

fn parse_arguments<T: Extension>() -> clap::ArgMatches {
    let mut app = clap::App::new("build_xray_quadtree")
        .version("1.0")
        .author(crate_authors!())
        .args(&[
            clap::Arg::new("output_directory")
                .about("Output directory to write the X-Ray quadtree into.")
                .long("output-directory")
                .required(true)
                .takes_value(true),
            clap::Arg::new("resolution")
                .about("Size of 1px in meters on the finest X-Ray level.")
                .long("resolution")
                .default_value("0.01"),
            clap::Arg::new("num_threads")
                .about("The number of threads used to shard X-Ray tile building.")
                .takes_value(true)
                .long("num-threads")
                .default_value("10"),
            clap::Arg::new("tile_size")
                .about("Size of finest X-Ray level tile in pixels. Must be a power of two.")
                .long("tile-size")
                .default_value("256"),
            clap::Arg::new("coloring_strategy")
                .long("coloring-strategy")
                .takes_value(true)
                .possible_values(&ColoringStrategyArgument::VARIANTS)
                .default_value("xray"),
            clap::Arg::new("min_intensity")
                .about(
                    "Minimum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("min-intensity")
                .takes_value(true)
                .default_value("0")
                .required_if_eq("coloring_strategy", "colored_with_intensity"),
            clap::Arg::new("max_intensity")
                .about(
                    "Maximum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("max-intensity")
                .takes_value(true)
                .default_value("1")
                .required_if_eq("coloring_strategy", "colored_with_intensity"),
            clap::Arg::new("colormap")
                .about("How values are mapped to colors")
                .long("colormap")
                .takes_value(true)
                .possible_values(&ColormapArgument::VARIANTS)
                .default_value("jet")
                .required_if_eq("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::new("max_stddev")
                .about(
                    "Maximum standard deviation for colored_with_height_stddev. Every stddev above this \
                     will be clamped to this value and appear saturated in the X-Rays. \
                     Only used for 'colored_with_height_stddev'.",
                )
                .long("max-stddev")
                .takes_value(true)
                .default_value("1")
                .required_if_eq("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::new("point_cloud_locations")
                .about("Point cloud locations to turn into xrays.")
                .index(1)
                .multiple(true)
                .required(true),
            clap::Arg::new("tile_background_color")
                .long("tile-background-color")
                .takes_value(true)
                .possible_values(&TileBackgroundColorArgument::VARIANTS)
                .default_value("white"),
            clap::Arg::new("filter_interval")
                .about("Filter intervals for attributes, e.g. --filter-interval intensity=2.0,51.0")
                .long("filter-interval")
                .takes_value(true)
                .multiple(true),
            clap::Arg::new("binning")
                .about(
                    "Binning size for one attribute, e.g. --binning timestamp=30000000000, \
                     which will be applied to 'colored' and 'colored_with_intensity' strategies. \
                     Colors will be first averaged within the same bin and then averaged over all \
                     bins, so e.g. for timestamped bins temporally closer points will get less \
                     weight than points temporally further away.")
                .long("binning")
                .takes_value(true),
            clap::Arg::new("root_node_id")
                .about("The root node id to start building with.")
                .long("root-node-id")
                .takes_value(true)
                .default_value("r"),
        ]);
    app = T::pre_init(app);
    app.get_matches()
}

pub fn run<T: Extension>(data_provider_factory: DataProviderFactory) {
    attempt_increasing_rlimit_to_max();

    let args = parse_arguments::<T>();
    let pixel_size_m = args
        .value_of("resolution")
        .unwrap()
        .parse::<f64>()
        .expect("resolution could not be parsed.");
    let num_threads = args
        .value_of("num_threads")
        .unwrap()
        .parse::<usize>()
        .expect("num_threads could not be parsed.");
    let tile_size_px = args
        .value_of("tile_size")
        .unwrap()
        .parse::<u32>()
        .expect("tile_size could not be parsed.");
    if !tile_size_px.is_power_of_two() {
        panic!("tile_size is not a power of two.");
    }

    let binning = args.value_of("binning").map(|f| parse_key_val(f).unwrap());
    let coloring_strategy_kind = {
        use ColoringStrategyArgument::*;
        let arg = ColoringStrategyArgument::from_str(
            args.value_of("coloring_strategy")
                .expect("coloring_strategy is invalid"),
            false,
        )
        .expect("coloring_strategy couldn't be parsed");
        match arg {
            Xray => ColoringStrategyKind::XRay,
            Colored => ColoringStrategyKind::Colored(binning),
            ColoredWithIntensity => ColoringStrategyKind::ColoredWithIntensity(
                args.value_of_t("min_intensity")
                    .expect("min_intensity is invalid"),
                args.value_of_t("max_intensity")
                    .expect("max_intensity is invalid"),
                binning,
            ),
            ColoredWithHeightStddev => ColoringStrategyKind::ColoredWithHeightStddev(
                args.value_of_t("max_stddev")
                    .expect("max_stddev is invalid"),
                ColormapArgument::from_str(
                    args.value_of("colormap").expect("colormap is invalid"),
                    false,
                )
                .expect("colormap couldn't be parsed"),
            ),
        }
    };

    let tile_background_color = TileBackgroundColorArgument::from_str(
        args.value_of("tile_background_color")
            .expect("tile_background_color is invalid"),
        false,
    )
    .expect("tile_background_color couldn't be parsed")
    .to_color();

    let output_directory = PathBuf::from(args.value_of("output_directory").unwrap());

    rayon::ThreadPoolBuilder::new()
        .num_threads(num_threads)
        .build_global()
        .expect("Could not create thread pool.");

    let point_cloud_locations = args
        .values_of("point_cloud_locations")
        .unwrap()
        .map(String::from)
        .collect::<Vec<_>>();
    let point_cloud_client = PointCloudClientBuilder::new(&point_cloud_locations)
        .data_provider_factory(data_provider_factory)
        // We do threading outside
        .num_threads(1)
        .build()
        .expect("Could not create point cloud client.");

    let filter_intervals = args
        .values_of("filter_interval")
        .unwrap_or_default()
        .map(|f| parse_key_val(f).unwrap())
        .collect::<HashMap<String, ClosedInterval<f64>>>();
    let root_node_id = args
        .value_of("root_node_id")
        .unwrap()
        .parse::<NodeId>()
        .expect("root_node_id could not be parsed.");
    let parameters = XrayParameters {
        output_directory,
        point_cloud_client,
        query_from_global: T::query_from_global(&args),
        filter_intervals,
        tile_background_color,
        tile_size_px,
        pixel_size_m,
        root_node_id,
    };
    build_xray_quadtree(&coloring_strategy_kind, &parameters)
        .expect("Failed to build xray quadtree.");
}
