// Code related to X-Ray generation.

use crate::proto;
use crate::CURRENT_VERSION;
use cgmath::{Point2, Point3};
use clap::{_clap_count_exprs, arg_enum};
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use image::{self, GenericImage};
use point_viewer::math::clamp;
use point_viewer::{
    color::{Color, TRANSPARENT},
    octree, InternalIterator, Point,
};
use protobuf::Message;
use quadtree::{ChildIndex, Node, NodeId, Rect};
use scoped_pool::Pool;
use stats::OnlineStats;
use std::collections::hash_map::Entry;
use std::error::Error;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::mpsc;

// The number of Z-buckets we subdivide our bounding cube into along the z-direction. This affects
// the saturation of a point in x-rays: the more buckets contain a point, the darker the pixel
// becomes.
const NUM_Z_BUCKETS: f32 = 1024.;

// Implementation of matlab's jet colormap from here:
// https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale
struct Jet;

impl Jet {
    fn red(&self, gray: f32) -> f32 {
        self.base(gray - 0.5)
    }

    fn green(&self, gray: f32) -> f32 {
        self.base(gray)
    }

    fn blue(&self, gray: f32) -> f32 {
        self.base(gray + 0.5)
    }

    fn base(&self, val: f32) -> f32 {
        if val <= -0.75 {
            0.
        } else if val <= -0.25 {
            self.interpolate(val, 0.0, -0.75, 1.0, -0.25)
        } else if val <= 0.25 {
            1.0
        } else if val <= 0.75 {
            self.interpolate(val, 1.0, 0.25, 0.0, 0.75)
        } else {
            0.0
        }
    }

    fn interpolate(&self, val: f32, y0: f32, x0: f32, y1: f32, x1: f32) -> f32 {
        (val - x0) * (y1 - y0) / (x1 - x0) + y0
    }

    pub fn for_value(&self, val: f32) -> Color<u8> {
        assert!(0. <= val);
        assert!(val <= 1.);
        Color {
            red: self.red(val),
            green: self.green(val),
            blue: self.blue(val),
            alpha: 1.,
        }
        .to_u8()
    }
}

arg_enum! {
    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum ColoringStrategyArgument {
        xray,
        colored,
        colored_with_intensity,
        colored_with_height_stddev,
    }
}

#[derive(Debug)]
pub enum ColoringStrategyKind {
    XRay,
    Colored,

    // Min and max intensities.
    ColoredWithIntensity(f32, f32),

    // Colored in heat-map colors by stddev. Takes the max stddev to clamp on.
    ColoredWithHeightStddev(f32),
}
impl ColoringStrategyKind {
    pub fn new_strategy(&self) -> Box<ColoringStrategy> {
        match *self {
            ColoringStrategyKind::XRay => Box::new(XRayColoringStrategy::new()),
            ColoringStrategyKind::Colored => Box::new(PointColorColoringStrategy::default()),
            ColoringStrategyKind::ColoredWithIntensity(min_intensity, max_intensity) => {
                Box::new(IntensityColoringStrategy::new(min_intensity, max_intensity))
            }
            ColoringStrategyKind::ColoredWithHeightStddev(max_stddev) => {
                Box::new(HeightStddevColoringStrategy::new(max_stddev))
            }
        }
    }
}

pub trait ColoringStrategy: Send {
    // Processes a point that has been discretized into the pixel (x, y) and the z column according
    // to NUM_Z_BUCKETS.
    fn process_discretized_point(&mut self, p: &Point, x: u32, y: u32, z: u32);

    // After all points are processed, this is used to query the color that should be assigned to
    // the pixel (x, y) in the final tile image.
    fn get_pixel_color(&self, x: u32, y: u32) -> Color<u8>;
}

struct XRayColoringStrategy {
    z_buckets: FnvHashMap<(u32, u32), FnvHashSet<u32>>,
    max_saturation: f32,
}

impl XRayColoringStrategy {
    fn new() -> Self {
        XRayColoringStrategy {
            z_buckets: FnvHashMap::default(),
            // TODO(sirver): Once 'const fn' lands, this constant can be calculated at compile time.
            max_saturation: NUM_Z_BUCKETS.ln(),
        }
    }
}

impl ColoringStrategy for XRayColoringStrategy {
    fn process_discretized_point(&mut self, _: &Point, x: u32, y: u32, z: u32) {
        match self.z_buckets.entry((x, y)) {
            Entry::Occupied(mut e) => {
                e.get_mut().insert(z);
            }
            Entry::Vacant(v) => {
                let mut s = FnvHashSet::default();
                s.insert(z);
                v.insert(s);
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Color<u8> {
        if !self.z_buckets.contains_key(&(x, y)) {
            return TRANSPARENT.to_u8();
        }
        let saturation = (self.z_buckets[&(x, y)].len() as f32).ln() / self.max_saturation;
        let value = ((1. - saturation) * 255.) as u8;
        Color {
            red: value,
            green: value,
            blue: value,
            alpha: value,
        }
    }
}

struct IntensityPerColumnData {
    sum: f32,
    count: usize,
}

struct IntensityColoringStrategy {
    min: f32,
    max: f32,
    per_column_data: FnvHashMap<(u32, u32), IntensityPerColumnData>,
}

impl IntensityColoringStrategy {
    fn new(min: f32, max: f32) -> Self {
        IntensityColoringStrategy {
            min,
            max,
            per_column_data: FnvHashMap::default(),
        }
    }
}

impl ColoringStrategy for IntensityColoringStrategy {
    fn process_discretized_point(&mut self, p: &Point, x: u32, y: u32, _: u32) {
        let intensity = p
            .intensity
            .expect("Coloring by intensity was requested, but point without intensity found.");
        if intensity < 0. {
            return;
        }
        match self.per_column_data.entry((x, y)) {
            Entry::Occupied(mut e) => {
                let per_column_data = e.get_mut();
                per_column_data.sum += intensity;
                per_column_data.count += 1;
            }
            Entry::Vacant(v) => {
                v.insert(IntensityPerColumnData {
                    sum: intensity,
                    count: 1,
                });
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return TRANSPARENT.to_u8();
        }
        let c = &self.per_column_data[&(x, y)];
        let mean = (c.sum / c.count as f32).max(self.min).min(self.max);
        let brighten = (mean - self.min).ln() / (self.max - self.min).ln();
        Color {
            red: brighten,
            green: brighten,
            blue: brighten,
            alpha: 1.,
        }
        .to_u8()
    }
}

struct PerColumnData {
    // The sum of all seen color values.
    color_sum: Color<f32>,

    // The number of all points that landed in this column.
    count: usize,
}

#[derive(Default)]
struct PointColorColoringStrategy {
    per_column_data: FnvHashMap<(u32, u32), PerColumnData>,
}

impl ColoringStrategy for PointColorColoringStrategy {
    fn process_discretized_point(&mut self, p: &Point, x: u32, y: u32, _: u32) {
        match self.per_column_data.entry((x, y)) {
            Entry::Occupied(mut e) => {
                let per_column_data = e.get_mut();
                let clr = p.color.to_f32();
                per_column_data.color_sum.red += clr.red;
                per_column_data.color_sum.green += clr.green;
                per_column_data.color_sum.blue += clr.blue;
                per_column_data.color_sum.alpha += clr.alpha;
                per_column_data.count += 1;
            }
            Entry::Vacant(v) => {
                v.insert(PerColumnData {
                    color_sum: p.color.to_f32(),
                    count: 1,
                });
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return TRANSPARENT.to_u8();
        }
        let c = &self.per_column_data[&(x, y)];
        Color {
            red: c.color_sum.red / c.count as f32,
            green: c.color_sum.green / c.count as f32,
            blue: c.color_sum.blue / c.count as f32,
            alpha: c.color_sum.alpha / c.count as f32,
        }
        .to_u8()
    }
}

struct HeightStddevColoringStrategy {
    per_column_data: FnvHashMap<(u32, u32), OnlineStats>,
    max_stddev: f32,
}

impl HeightStddevColoringStrategy {
    fn new(max_stddev: f32) -> Self {
        HeightStddevColoringStrategy {
            max_stddev,
            per_column_data: FnvHashMap::default(),
        }
    }
}

/// Build a parent image created of the 4 children tiles. All tiles are optionally, in which case
/// they are left white in the resulting image. The input images must be square with length N,
/// the returned image is square with length 2*N.
pub fn build_parent(children: &[Option<image::RgbaImage>]) -> image::RgbaImage {
    assert_eq!(children.len(), 4);
    let mut child_size_px = None;
    for c in children.iter() {
        if c.is_none() {
            continue;
        }
        let c = c.as_ref().unwrap();
        assert_eq!(
            c.width(),
            c.height(),
            "Expected width to be equal to height."
        );
        match child_size_px {
            None => child_size_px = Some(c.width()),
            Some(w) => {
                assert_eq!(w, c.width(), "Not all images have the same size.");
            }
        }
    }
    let child_size_px = child_size_px.expect("No children passed to 'build_parent'.");
    let mut large_image = image::RgbaImage::from_pixel(
        child_size_px * 2,
        child_size_px * 2,
        image::Rgba {
            data: [
                TRANSPARENT.to_u8().red,
                TRANSPARENT.to_u8().green,
                TRANSPARENT.to_u8().blue,
                TRANSPARENT.to_u8().alpha,
            ],
        },
    );

    // We want the x-direction to be up in the octree. Since (0, 0) is the top left
    // position in the image, we actually have to invert y and go from the bottom
    // of the image.
    for &(id, xoffs, yoffs) in &[
        (1, 0, 0),
        (0, 0, child_size_px),
        (3, child_size_px, 0),
        (2, child_size_px, child_size_px),
    ] {
        if let Some(ref img) = children[id] {
            large_image.copy_from(img, xoffs, yoffs);
        }
    }
    large_image
}

impl ColoringStrategy for HeightStddevColoringStrategy {
    fn process_discretized_point(&mut self, p: &Point, x: u32, y: u32, _: u32) {
        self.per_column_data
            .entry((x, y))
            .or_insert_with(|| OnlineStats::new())
            .add(p.position.z);
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return TRANSPARENT.to_u8();
        }
        let c = &self.per_column_data[&(x, y)];
        let saturation = clamp(c.stddev() as f32, 0., self.max_stddev) / self.max_stddev;
        Jet {}.for_value(saturation)
    }
}

pub fn xray_from_points(
    octree: &octree::OnDiskOctree,
    bbox: &Aabb3<f32>,
    png_file: &Path,
    image_width: u32,
    image_height: u32,
    mut coloring_strategy: Box<ColoringStrategy>,
) -> bool {
    let mut seen_any_points = false;
    octree.points_in_box(bbox).for_each(|p| {
        seen_any_points = true;
        // We want a right handed coordinate system with the x-axis of world and images aligning.
        // This means that the y-axis aligns too, but the origin of the image space must be at the
        // bottom left. Since images have their origin at the top left, we need actually have to
        // invert y and go from the bottom of the image.
        let x = (((p.position.x - bbox.min().x) / bbox.dim().x) * image_width as f32) as u32;
        let y =
            ((1. - ((p.position.y - bbox.min().y) / bbox.dim().y)) * image_height as f32) as u32;
        let z = (((p.position.z - bbox.min().z) / bbox.dim().z) * NUM_Z_BUCKETS) as u32;
        coloring_strategy.process_discretized_point(p, x, y, z);
    });

    if !seen_any_points {
        return false;
    }

    let mut image = image::RgbaImage::new(image_width, image_height);
    for x in 0..image_width {
        for y in 0..image_height {
            let color = coloring_strategy.get_pixel_color(x, y);
            image.put_pixel(
                x,
                y,
                image::Rgba {
                    data: [color.red, color.green, color.blue, color.alpha],
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

pub fn build_xray_quadtree(
    pool: &Pool,
    octree: &octree::OnDiskOctree,
    output_directory: &Path,
    resolution: f32,
    tile_size_px: u32,
    coloring_strategy_kind: ColoringStrategyKind,
) -> Result<(), Box<Error>> {
    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);

    let bounding_box = octree.bounding_box();
    let (bounding_rect, deepest_level) =
        find_quadtree_bounding_rect_and_levels(&bounding_box, tile_size_px as f32 * resolution);

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
                    let mut children = [None, None, None, None];

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
                        children[id as usize] = Some(image::open(&png).unwrap().to_rgba());
                    }
                    let large_image = build_parent(&children);
                    let image = image::DynamicImage::ImageRgba8(large_image).resize(
                        tile_size_px,
                        tile_size_px,
                        image::FilterType::Lanczos3,
                    );
                    image
                        .as_rgba8()
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
