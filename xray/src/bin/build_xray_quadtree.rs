extern crate cgmath;
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
use fnv::{FnvHashMap, FnvHashSet};
use image::GenericImage;
use octree::OnDiskOctree;
use point_viewer::{octree, InternalIterator};
use protobuf::Message;
use quadtree::{ChildIndex, Node, NodeId, Rect};
use scoped_pool::Pool;
use std::collections::hash_map::Entry;
use std::error::Error;
use std::fs;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::mpsc;
use xray::{proto, CURRENT_VERSION};

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
            clap::Arg::with_name("octree_directory")
                .help("Octree directory to turn into xrays.")
                .index(1)
                .required(true),
        ])
        .get_matches()
}

fn xray_from_points(
    octree: &octree::OnDiskOctree,
    bbox: &Aabb3<f32>,
    png_file: &Path,
    image_size: u32,
) -> bool {
    const NUM_Z_BUCKETS: f32 = 1024.;

    let mut aggregation: FnvHashMap<(u32, u32), FnvHashSet<u32>> = FnvHashMap::default();
    let mut seen_any_points = false;
    octree.points_in_box(&bbox).for_each(|p| {
        seen_any_points = true;
        let x = (((p.position.x - bbox.min().x) / bbox.dim().x) * image_size as f32) as u32;
        let y = (((p.position.y - bbox.min().y) / bbox.dim().y) * image_size as f32) as u32;
        let z = (((p.position.z - bbox.min().z) / bbox.dim().z) * NUM_Z_BUCKETS) as u32;
        match aggregation.entry((x, y)) {
            Entry::Occupied(mut e) => {
                e.get_mut().insert(z);
            }
            Entry::Vacant(v) => {
                let mut s = FnvHashSet::default();
                s.insert(z);
                v.insert(s);
            }
        }
    });

    if !seen_any_points {
        return false;
    }

    let max_saturation = NUM_Z_BUCKETS.ln();
    let mut image = image::RgbImage::new(image_size, image_size);
    for x in 0..image_size {
        for y in 0..image_size {
            if !aggregation.contains_key(&(x, y)) {
                image.put_pixel(
                    x,
                    y,
                    image::Rgb {
                        data: [255, 255, 255],
                    },
                );
                continue;
            }
            let saturation = (aggregation[&(x, y)].len() as f32).ln() / max_saturation;
            let value = ((1. - saturation) * 255.) as u8;
            image.put_pixel(
                x,
                y,
                image::Rgb {
                    data: [value, value, value],
                },
            );
        }
    }
    image.save(png_file).unwrap();
    true
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
                    let mut large_image = image::RgbImage::from_pixel(
                        tile_size_px * 2,
                        tile_size_px * 2,
                        image::Rgb {
                            data: [255, 255, 255],
                        },
                    );

                    for &(id, xoffs, yoffs) in &[
                        (0, 0, 0),
                        (1, 0, tile_size_px),
                        (2, tile_size_px, 0),
                        (3, tile_size_px, tile_size_px),
                    ] {
                        let png = get_image_path(
                            output_directory,
                            node_id.get_child_id(ChildIndex::from_u8(id)),
                        );
                        if !png.exists() {
                            continue;
                        }
                        let img = image::open(&png).unwrap().to_rgb();
                        large_image.copy_from(&img, xoffs, yoffs);
                    }
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

    let octree_directory = Path::new(args.value_of("octree_directory").unwrap());
    let output_directory = Path::new(args.value_of("output_directory").unwrap());

    run(octree_directory, output_directory, resolution, tile_size).unwrap();
}
