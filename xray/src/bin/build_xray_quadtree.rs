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
use collision::{Aabb, Aabb3};
use fnv::FnvHashSet;
use octree::OnDiskOctree;
use point_viewer::octree;
use protobuf::Message;
use quadtree::{ChildIndex, Node, NodeId, Rect};
use scoped_pool::Pool;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use xray::{proto,
           generation::{xray_from_points, ColoringStrategy, ColoringStrategyArgument,
                        ColoringStrategyKind},
           CURRENT_VERSION};

fn parse_arguments() -> clap::ArgMatches<'static> {
    clap::App::new("build_xray_quadtree")
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
            clap::Arg::with_name("octree_directory")
                .help("Octree directory to turn into xrays.")
                .index(1)
                .required(true),
        ])
        .get_matches()
}

fn find_quadtree_bounding_rect_and_levels(bbox: &Aabb3<f32>, tile_size_m: f32) -> (Rect, u8) {
    let mut levels = 0;
    let mut cur_size = tile_size_m;
    while cur_size < bbox.dim().x || cur_size < bbox.dim().y {
        cur_size *= 2.;
        levels += 1;
    }
    (
        Rect::new(Point2::new(bbox.min().x, bbox.min().y), cur_size),
        levels,
    )
}

pub fn get_image_path(directory: &Path, id: NodeId) -> PathBuf {
    let mut rv = directory.join(&id.to_string());
    rv.set_extension("png");
    rv
}

fn run(
    octree_directory: &Path,
    output_directory: &Path,
    resolution: f32,
    tile_size_px: u32,
    coloring_strategy_kind: ColoringStrategyKind,
) -> Result<(), Box<Error>> {
    let octree = &OnDiskOctree::new(octree_directory)?;

    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);

    let bounding_box = octree.bounding_box();

    let (bounding_rect, deepest_level) =
        find_quadtree_bounding_rect_and_levels(&bounding_box, tile_size_px as f32 * resolution);

    let pool = Pool::new(10);

    // Create the deepest level of the quadtree.
    let (parents_to_create_tx, mut parents_to_create_rx) = mpsc::channel();
    let (all_nodes_tx, all_nodes_rx) = mpsc::channel();
    println!("Building level {}.", deepest_level);

    pool.scoped(|scope| {
        let mut open = vec![Node::root_with_bounding_rect(bounding_rect.clone())];
        while !open.is_empty() {
            let node = open.pop().unwrap();
            if node.level() == deepest_level {
                let parents_to_create_tx_clone = parents_to_create_tx.clone();
                let all_nodes_tx_clone = all_nodes_tx.clone();
                let strategy: Box<ColoringStrategy> = coloring_strategy_kind.new_strategy();
                scope.execute(move || {
                    let bbox = Aabb3::new(
                        Point3::new(
                            node.bounding_rect.min().x,
                            node.bounding_rect.min().y,
                            bounding_box.min().z,
                        ),
                        Point3::new(
                            node.bounding_rect.max().x,
                            node.bounding_rect.max().y,
                            bounding_box.max().z,
                        ),
                    );
                    if xray_from_points(
                        octree,
                        &bbox,
                        &get_image_path(output_directory, node.id),
                        tile_size_px,
                        tile_size_px,
                        strategy,
                    ) {
                        all_nodes_tx_clone.send(node.id).unwrap();
                        node.id
                            .parent_id()
                            .map(|id| parents_to_create_tx_clone.send(id).unwrap());
                    }
                });
            } else {
                for i in 0..4 {
                    open.push(node.get_child(ChildIndex::from_u8(i)));
                }
            }
        }
    });
    drop(parents_to_create_tx);

    for current_level in (0..deepest_level).rev() {
        println!("Building level {}.", current_level);
        let nodes_to_create: FnvHashSet<NodeId> = parents_to_create_rx.into_iter().collect();
        let (parents_to_create_tx, new_rx) = mpsc::channel();
        parents_to_create_rx = new_rx;
        pool.scoped(|scope| {
            for node_id in nodes_to_create {
                all_nodes_tx.send(node_id).unwrap();
                let tx_clone = parents_to_create_tx.clone();
                scope.execute(move || {
                    let mut children = [ None, None, None, None ];

                    // We a right handed coordinate system with the x-axis of world and images
                    // aligning. This means that the y-axis aligns too, but the origin of the image
                    // space must be at the bottom left. Since images have their origin at the top
                    // left, we need actually have to invert y and go from the bottom of the image.
                    for id in 0..4 {
                        let png = get_image_path(
                            output_directory,
                            node_id.get_child_id(ChildIndex::from_u8(id)),
                        );
                        if !png.exists() {
                            continue;
                        }
                        children[id as usize] = Some(image::open(&png).unwrap().to_rgb());
                    }
                    let large_image = xray::generation::build_parent(&children);
                    let image = image::DynamicImage::ImageRgb8(large_image).resize(
                        tile_size_px,
                        tile_size_px,
                        image::FilterType::Lanczos3,
                    );
                    image
                        .as_rgb8()
                        .unwrap()
                        .save(&get_image_path(output_directory, node_id))
                        .unwrap();
                    node_id.parent_id().map(|id| tx_clone.send(id).unwrap());
                });
            }
        });
        drop(parents_to_create_tx);
    }
    drop(all_nodes_tx);

    let meta = {
        let mut meta = proto::Meta::new();
        meta.mut_bounding_rect()
            .mut_min()
            .set_x(bounding_rect.min().x);
        meta.mut_bounding_rect()
            .mut_min()
            .set_y(bounding_rect.min().y);
        meta.mut_bounding_rect()
            .set_edge_length(bounding_rect.edge_length());
        meta.set_deepest_level(deepest_level as u32);
        meta.set_tile_size(tile_size_px);
        meta.set_version(CURRENT_VERSION);

        for node_id in all_nodes_rx {
            let mut proto = proto::NodeId::new();
            proto.set_index(node_id.index());
            proto.set_level(node_id.level() as u32);
            meta.mut_nodes().push(proto);
        }
        meta
    };

    let mut buf_writer = BufWriter::new(File::create(output_directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();

    Ok(())
}

pub fn main() {
    let args = parse_arguments();
    let resolution = args.value_of("resolution")
        .unwrap()
        .parse::<f32>()
        .expect("resolution could not be parsed.");
    let tile_size = args.value_of("tile_size")
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

    let octree_directory = Path::new(args.value_of("octree_directory").unwrap());
    let output_directory = Path::new(args.value_of("output_directory").unwrap());

    run(
        octree_directory,
        output_directory,
        resolution,
        tile_size,
        coloring_strategy_kind,
    ).unwrap();
}
