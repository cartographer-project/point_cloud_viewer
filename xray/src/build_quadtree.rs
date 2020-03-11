use crate::generation::{
    build_xray_quadtree, ColoringStrategyArgument, ColoringStrategyKind, ColormapArgument, Tile,
    TileBackgroundColorArgument, XrayParameters,
};
use clap::value_t;
use point_cloud_client::PointCloudClientBuilder;
use point_viewer::color::{TRANSPARENT, WHITE};
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::math::{ClosedInterval, Isometry3};
use point_viewer::read_write::attempt_increasing_rlimit_to_max;
use point_viewer::utils::parse_key_val;
use std::collections::HashMap;
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
                .long("output-directory")
                .required(true)
                .takes_value(true),
            clap::Arg::with_name("resolution")
                .help("Size of 1px in meters on the finest X-Ray level.")
                .long("resolution")
                .default_value("0.01"),
            clap::Arg::with_name("num_threads")
                .help("The number of threads used to shard X-Ray tile building.")
                .takes_value(true)
                .long("num-threads")
                .default_value("10"),
            clap::Arg::with_name("tile_size")
                .help("Size of finest X-Ray level tile in pixels. Must be a power of two.")
                .long("tile-size")
                .default_value("256"),
            clap::Arg::with_name("coloring_strategy")
                .long("coloring-strategy")
                .takes_value(true)
                .possible_values(&ColoringStrategyArgument::variants())
                .default_value("xray"),
            clap::Arg::with_name("min_intensity")
                .help(
                    "Minimum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("min-intensity")
                .takes_value(true)
                .default_value("0")
                .required_if("coloring_strategy", "colored_with_intensity"),
            clap::Arg::with_name("max_intensity")
                .help(
                    "Maximum intensity of all points for color scaling. \
                     Only used for 'colored_with_intensity'.",
                )
                .long("max-intensity")
                .takes_value(true)
                .default_value("1")
                .required_if("coloring_strategy", "colored_with_intensity"),
            clap::Arg::with_name("colormap")
                .help("How values are mapped to colors")
                .long("colormap")
                .takes_value(true)
                .possible_values(&ColormapArgument::variants())
                .default_value("jet")
                .required_if("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::with_name("max_stddev")
                .help(
                    "Maximum standard deviation for colored_with_height_stddev. Every stddev above this \
                     will be clamped to this value and appear saturated in the X-Rays. \
                     Only used for 'colored_with_height_stddev'.",
                )
                .long("max-stddev")
                .takes_value(true)
                .default_value("1")
                .required_if("coloring_strategy", "colored_with_height_stddev"),
            clap::Arg::with_name("point_cloud_locations")
                .help("Point cloud locations to turn into xrays.")
                .index(1)
                .multiple(true)
                .required(true),
            clap::Arg::with_name("tile_background_color")
                .long("tile-background-color")
                .takes_value(true)
                .possible_values(&TileBackgroundColorArgument::variants())
                .default_value("white"),
            clap::Arg::with_name("filter_interval")
                .help("Filter intervals for attributes, e.g. --filter-interval intensity=2.0,51.0")
                .long("filter-interval")
                .takes_value(true)
                .multiple(true),
            clap::Arg::with_name("binning")
                .help(
                    "Binning size for one attribute, e.g. --binning timestamp=30000000000, \
                     which will be applied to 'colored' and 'colored_with_intensity' strategies. \
                     Colors will be first averaged within the same bin and then averaged over all
                     bins, so e.g. for timestamped bins temporally closer points will get less
                     weight than points temporally further away.")
                .long("binning")
                .takes_value(true),
            clap::Arg::with_name("inpaint_distance_px")
                .help("The inpainting distance in pixels to fill holes (in particular useful \
                       for high resolutions).")
                .long("inpaint-distance-px")
                .takes_value(true)
                .default_value("0"),
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

    let binning = args.value_of("binning").map(|f| parse_key_val(f).unwrap());
    let coloring_strategy_kind = {
        use ColoringStrategyArgument::*;
        let arg = value_t!(args, "coloring_strategy", ColoringStrategyArgument)
            .expect("coloring_strategy is invalid");
        match arg {
            xray => ColoringStrategyKind::XRay,
            colored => ColoringStrategyKind::Colored(binning),
            colored_with_intensity => ColoringStrategyKind::ColoredWithIntensity(
                value_t!(args, "min_intensity", f32).expect("min_intensity is invalid"),
                value_t!(args, "max_intensity", f32).expect("max_intensity is invalid"),
                binning,
            ),
            colored_with_height_stddev => ColoringStrategyKind::ColoredWithHeightStddev(
                value_t!(args, "max_stddev", f32).expect("max_stddev is invalid"),
                value_t!(args, "colormap", ColormapArgument).expect("colormap is invalid"),
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
    let inpaint_distance_px = args
        .value_of("inpaint_distance_px")
        .unwrap()
        .parse::<u8>()
        .expect("inpaint_distance_px could not be parsed.");
    let parameters = XrayParameters {
        point_cloud_client,
        query_from_global: T::query_from_global(&args),
        filter_intervals,
        tile_background_color,
        inpaint_distance_px,
    };
    build_xray_quadtree(
        output_directory,
        &Tile {
            size_px: tile_size,
            resolution,
        },
        &coloring_strategy_kind,
        &parameters,
    )
    .unwrap();
}
