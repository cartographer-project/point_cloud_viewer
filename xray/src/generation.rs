// Code related to X-Ray generation.

use crate::colormap::{Colormap, Jet, Monochrome, PURPLISH};
use crate::proto;
use crate::CURRENT_VERSION;
use cgmath::{Decomposed, EuclideanSpace, Point2, Point3, Quaternion, Vector2, Vector3};
use clap::arg_enum;
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use image::{self, GenericImage};
use num::clamp;
use pbr::ProgressBar;
use point_cloud_client::PointCloudClient;
use point_viewer::attributes::AttributeData;
use point_viewer::iterator::{PointLocation, PointQuery};
use point_viewer::math::{ClosedInterval, Isometry3, Obb};
use point_viewer::{color::Color, PointsBatch};
use protobuf::Message;
use quadtree::{ChildIndex, Node, NodeId, Rect};
use scoped_pool::Pool;
use stats::OnlineStats;
use std::collections::{hash_map::Entry, HashMap, HashSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::{mpsc, Arc, Mutex};
use std::time::Duration;

// The number of Z-buckets we subdivide our bounding cube into along the z-direction. This affects
// the saturation of a point in x-rays: the more buckets contain a point, the darker the pixel
// becomes.
const NUM_Z_BUCKETS: f64 = 1024.;

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

arg_enum! {
    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum TileBackgroundColorArgument {
        white,
        transparent,
    }
}

arg_enum! {
    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum ColormapArgument {
        jet,
        purplish,
    }
}

#[derive(Debug)]
pub enum ColoringStrategyKind {
    XRay,
    Colored,

    // Min and max intensities.
    ColoredWithIntensity(f32, f32),

    // Colored in heat-map colors by stddev. Takes the max stddev to clamp on.
    ColoredWithHeightStddev(f32, ColormapArgument),
}

impl ColoringStrategyKind {
    pub fn new_strategy(&self) -> Box<dyn ColoringStrategy> {
        match *self {
            ColoringStrategyKind::XRay => Box::new(XRayColoringStrategy::new()),
            ColoringStrategyKind::Colored => Box::new(PointColorColoringStrategy::default()),
            ColoringStrategyKind::ColoredWithIntensity(min_intensity, max_intensity) => {
                Box::new(IntensityColoringStrategy::new(min_intensity, max_intensity))
            }
            ColoringStrategyKind::ColoredWithHeightStddev(max_stddev, ColormapArgument::jet) => {
                Box::new(HeightStddevColoringStrategy::<Jet>::new(max_stddev, Jet {}))
            }
            ColoringStrategyKind::ColoredWithHeightStddev(
                max_stddev,
                ColormapArgument::purplish,
            ) => Box::new(HeightStddevColoringStrategy::new(
                max_stddev,
                Monochrome(PURPLISH),
            )),
        }
    }
}

pub trait ColoringStrategy: Send {
    // Processes points that have been discretized into the pixels (x, y) and the z columns according
    // to NUM_Z_BUCKETS.
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    );

    fn process_point_data(
        &mut self,
        points_batch: &PointsBatch,
        bbox: &Aabb3<f64>,
        image_size: Vector2<u32>,
    ) {
        let mut discretized_locations = Vec::with_capacity(points_batch.position.len());
        for pos in &points_batch.position {
            // We want a right handed coordinate system with the x-axis of world and images aligning.
            // This means that the y-axis aligns too, but the origin of the image space must be at the
            // bottom left. Since images have their origin at the top left, we need actually have to
            // invert y and go from the bottom of the image.
            let x = (((pos.x - bbox.min().x) / bbox.dim().x) * f64::from(image_size.x)) as u32;
            let y =
                ((1. - ((pos.y - bbox.min().y) / bbox.dim().y)) * f64::from(image_size.y)) as u32;
            let z = (((pos.z - bbox.min().z) / bbox.dim().z) * NUM_Z_BUCKETS) as u32;
            discretized_locations.push(Point3::new(x, y, z));
        }
        self.process_discretized_point_data(points_batch, discretized_locations)
    }

    // After all points are processed, this is used to query the color that should be assigned to
    // the pixel (x, y) in the final tile image.
    fn get_pixel_color(&self, x: u32, y: u32, background_color: Color<u8>) -> Color<u8>;

    fn attributes(&self) -> Vec<&'static str>;
}

struct XRayColoringStrategy {
    z_buckets: FnvHashMap<(u32, u32), FnvHashSet<u32>>,
    max_saturation: f64,
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
    fn process_discretized_point_data(
        &mut self,
        _: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        for d_loc in discretized_locations {
            match self.z_buckets.entry((d_loc.x, d_loc.y)) {
                Entry::Occupied(mut e) => {
                    e.get_mut().insert(d_loc.z);
                }
                Entry::Vacant(v) => {
                    let mut set = FnvHashSet::default();
                    set.insert(d_loc.z);
                    v.insert(set);
                }
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32, background_color: Color<u8>) -> Color<u8> {
        if !self.z_buckets.contains_key(&(x, y)) {
            return background_color;
        }
        let saturation = (self.z_buckets[&(x, y)].len() as f64).ln() / self.max_saturation;
        let value = ((1. - saturation) * 255.) as u8;
        Color {
            red: value,
            green: value,
            blue: value,
            alpha: 255,
        }
    }

    fn attributes(&self) -> Vec<&'static str> {
        vec![]
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
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        let intensity_attribute = points_batch
            .attributes
            .get("intensity")
            .expect("Coloring by intensity was requested, but point data without intensity found.");
        if let AttributeData::F32(intensity_vec) = intensity_attribute {
            for i in 0..intensity_vec.len() {
                let intensity = intensity_vec[i];
                if intensity < 0. {
                    return;
                }
                match self
                    .per_column_data
                    .entry((discretized_locations[i].x, discretized_locations[i].y))
                {
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
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32, background_color: Color<u8>) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return background_color;
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

    fn attributes(&self) -> Vec<&'static str> {
        vec!["intensity"]
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
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        let color_attribute = points_batch
            .attributes
            .get("color")
            .expect("Coloring was requested, but point data without color found.");
        if let AttributeData::U8Vec3(color_vec) = color_attribute {
            for i in 0..color_vec.len() {
                let clr = Color::<u8> {
                    red: color_vec[i][0],
                    green: color_vec[i][1],
                    blue: color_vec[i][2],
                    alpha: 255,
                }
                .to_f32();
                match self
                    .per_column_data
                    .entry((discretized_locations[i].x, discretized_locations[i].y))
                {
                    Entry::Occupied(mut e) => {
                        let per_column_data = e.get_mut();
                        per_column_data.color_sum.red += clr.red;
                        per_column_data.color_sum.green += clr.green;
                        per_column_data.color_sum.blue += clr.blue;
                        per_column_data.color_sum.alpha += clr.alpha;
                        per_column_data.count += 1;
                    }
                    Entry::Vacant(v) => {
                        v.insert(PerColumnData {
                            color_sum: clr,
                            count: 1,
                        });
                    }
                }
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32, background_color: Color<u8>) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return background_color;
        }
        let c = &self.per_column_data[&(x, y)];
        Color {
            red: c.color_sum.red / c.count as f32,
            green: c.color_sum.green / c.count as f32,
            blue: c.color_sum.blue / c.count as f32,
            alpha: 1.,
        }
        .to_u8()
    }

    fn attributes(&self) -> Vec<&'static str> {
        vec!["color"]
    }
}

struct HeightStddevColoringStrategy<C: Colormap> {
    per_column_data: FnvHashMap<(u32, u32), OnlineStats>,
    max_stddev: f32,
    colormap: C,
}

impl<C: Colormap> HeightStddevColoringStrategy<C> {
    fn new(max_stddev: f32, colormap: C) -> Self {
        HeightStddevColoringStrategy {
            max_stddev,
            per_column_data: FnvHashMap::default(),
            colormap,
        }
    }
}

impl<C: Colormap> ColoringStrategy for HeightStddevColoringStrategy<C> {
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        for (i, d_loc) in discretized_locations
            .iter()
            .enumerate()
            .take(discretized_locations.len())
        {
            self.per_column_data
                .entry((d_loc.x, d_loc.y))
                .or_insert_with(OnlineStats::new)
                .add(points_batch.position[i].z);
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32, background_color: Color<u8>) -> Color<u8> {
        if !self.per_column_data.contains_key(&(x, y)) {
            return background_color;
        }
        let c = &self.per_column_data[&(x, y)];
        let saturation = clamp(c.stddev() as f32, 0., self.max_stddev) / self.max_stddev;
        self.colormap.for_value(saturation)
    }

    fn attributes(&self) -> Vec<&'static str> {
        vec![]
    }
}

/// Build a parent image created of the 4 children tiles. All tiles are optionally, in which case
/// they are left white in the resulting image. The input images must be square with length N,
/// the returned image is square with length 2*N.
pub fn build_parent(
    children: &[Option<image::RgbaImage>],
    tile_background_color: Color<u8>,
) -> image::RgbaImage {
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
        image::Rgba([
            tile_background_color.red,
            tile_background_color.green,
            tile_background_color.blue,
            tile_background_color.alpha,
        ]),
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

pub struct Tile {
    pub size_px: u32,
    pub resolution: f64,
}

pub struct XrayParameters {
    pub point_cloud_client: PointCloudClient,
    pub query_from_global: Option<Isometry3<f64>>,
    pub filter_intervals: HashMap<String, ClosedInterval<f64>>,
    pub tile_background_color: Color<u8>,
}

pub fn xray_from_points(
    bbox: &Aabb3<f64>,
    png_file: &Path,
    image_size: Vector2<u32>,
    mut coloring_strategy: Box<dyn ColoringStrategy>,
    parameters: &XrayParameters,
) -> bool {
    let mut seen_any_points = false;
    let location = match &parameters.query_from_global {
        Some(query_from_global) => {
            let global_from_query = query_from_global.inverse();
            PointLocation::Obb(Obb::from(bbox).transformed(&global_from_query))
        }
        None => PointLocation::Aabb(*bbox),
    };
    let mut attributes: HashSet<_> = coloring_strategy.attributes().into_iter().collect();
    attributes.extend(parameters.filter_intervals.keys().map(|k| &k[..]));
    let point_query = PointQuery {
        attributes: attributes.into_iter().collect(),
        location,
        filter_intervals: parameters
            .filter_intervals
            .iter()
            .map(|(k, v)| (&k[..], *v))
            .collect(),
    };
    let _ = parameters
        .point_cloud_client
        .for_each_point_data(&point_query, |mut points_batch| {
            seen_any_points = true;
            if let Some(query_from_global) = &parameters.query_from_global {
                for p in &mut points_batch.position {
                    *p = (query_from_global * &Point3::from_vec(*p)).to_vec();
                }
            }
            coloring_strategy.process_point_data(&points_batch, bbox, image_size);
            Ok(())
        });

    if !seen_any_points {
        return false;
    }

    let mut image = image::RgbaImage::new(image_size.x, image_size.y);
    for x in 0..image.width() {
        for y in 0..image.height() {
            let color = coloring_strategy.get_pixel_color(x, y, parameters.tile_background_color);
            image.put_pixel(
                x,
                y,
                image::Rgba([color.red, color.green, color.blue, color.alpha]),
            );
        }
    }
    image.save(png_file).unwrap();
    true
}

fn find_quadtree_bounding_rect_and_levels(bbox: &Aabb3<f64>, tile_size_m: f64) -> (Rect, u8) {
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
    output_directory: &Path,
    tile: &Tile,
    coloring_strategy_kind: &ColoringStrategyKind,
    parameters: &XrayParameters,
) -> Result<(), Box<dyn Error>> {
    // Ignore errors, maybe directory is already there.
    let _ = fs::create_dir(output_directory);

    let bounding_box = match &parameters.query_from_global {
        Some(query_from_global) => {
            let decomposed: Decomposed<Vector3<f64>, Quaternion<f64>> =
                query_from_global.clone().into();
            parameters
                .point_cloud_client
                .bounding_box()
                .transform(&decomposed)
        }
        None => *parameters.point_cloud_client.bounding_box(),
    };
    let (bounding_rect, deepest_level) = find_quadtree_bounding_rect_and_levels(
        &bounding_box,
        f64::from(tile.size_px) * tile.resolution,
    );

    // Create the deepest level of the quadtree.
    let (parents_to_create_tx, mut parents_to_create_rx) = mpsc::channel();
    let (all_nodes_tx, all_nodes_rx) = mpsc::channel();
    let num_leaf_nodes = 4usize.pow(deepest_level.into());
    let num_all_nodes = (4 * num_leaf_nodes - 1) / 3;

    let mut progress_bar = ProgressBar::new(num_all_nodes as u64);
    progress_bar.set_max_refresh_rate(Some(Duration::from_secs(2)));
    progress_bar.message(&format!("Building level {}: ", deepest_level));
    let progress_bar = Arc::new(Mutex::new(progress_bar));

    let mut leaf_nodes = Vec::with_capacity(num_leaf_nodes);
    let mut nodes_to_traverse = Vec::with_capacity(num_all_nodes);
    nodes_to_traverse.push(Node::root_with_bounding_rect(bounding_rect.clone()));
    while let Some(node) = nodes_to_traverse.pop() {
        if node.level() == deepest_level {
            leaf_nodes.push(node);
        } else {
            for i in 0..4 {
                nodes_to_traverse.push(node.get_child(&ChildIndex::from_u8(i)));
            }
        }
    }
    pool.scoped(|scope| {
        while let Some(node) = leaf_nodes.pop() {
            let parents_to_create_tx_clone = parents_to_create_tx.clone();
            let all_nodes_tx_clone = all_nodes_tx.clone();
            let strategy: Box<dyn ColoringStrategy> = coloring_strategy_kind.new_strategy();
            let progress_bar = Arc::clone(&progress_bar);
            scope.execute(move || {
                let rect_min = node.bounding_rect.min();
                let rect_max = node.bounding_rect.max();
                let min = Point3::new(rect_min.x, rect_min.y, bounding_box.min().z);
                let max = Point3::new(rect_max.x, rect_max.y, bounding_box.max().z);
                let bbox = Aabb3::new(min, max);
                if xray_from_points(
                    &bbox,
                    &get_image_path(output_directory, node.id),
                    Vector2::new(tile.size_px, tile.size_px),
                    strategy,
                    parameters,
                ) {
                    all_nodes_tx_clone.send(node.id).unwrap();
                    if let Some(id) = node.id.parent_id() {
                        parents_to_create_tx_clone.send(id).unwrap()
                    }
                }
                progress_bar.lock().unwrap().inc();
            });
        }
    });
    drop(parents_to_create_tx);

    for current_level in (0..deepest_level).rev() {
        progress_bar
            .lock()
            .unwrap()
            .message(&format!("Building level {}: ", current_level));
        let nodes_to_create: FnvHashSet<NodeId> = parents_to_create_rx.into_iter().collect();
        let (parents_to_create_tx, new_rx) = mpsc::channel();
        parents_to_create_rx = new_rx;
        pool.scoped(|scope| {
            for node_id in nodes_to_create {
                all_nodes_tx.send(node_id).unwrap();
                let tx_clone = parents_to_create_tx.clone();
                let progress_bar = Arc::clone(&progress_bar);
                scope.execute(move || {
                    let mut children = [None, None, None, None];

                    // We a right handed coordinate system with the x-axis of world and images
                    // aligning. This means that the y-axis aligns too, but the origin of the image
                    // space must be at the bottom left. Since images have their origin at the top
                    // left, we need actually have to invert y and go from the bottom of the image.
                    for id in 0..4 {
                        let png = get_image_path(
                            output_directory,
                            node_id.get_child_id(&ChildIndex::from_u8(id)),
                        );
                        if !png.exists() {
                            continue;
                        }
                        children[id as usize] = Some(image::open(&png).unwrap().to_rgba());
                    }
                    let large_image = build_parent(&children, parameters.tile_background_color);
                    let image = image::DynamicImage::ImageRgba8(large_image).resize(
                        tile.size_px,
                        tile.size_px,
                        image::FilterType::Lanczos3,
                    );
                    image
                        .as_rgba8()
                        .unwrap()
                        .save(&get_image_path(output_directory, node_id))
                        .unwrap();
                    if let Some(id) = node_id.parent_id() {
                        tx_clone.send(id).unwrap()
                    }
                    progress_bar.lock().unwrap().inc();
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
        meta.set_deepest_level(u32::from(deepest_level));
        meta.set_tile_size(tile.size_px);
        meta.set_version(CURRENT_VERSION);

        for node_id in all_nodes_rx {
            let mut proto = proto::NodeId::new();
            proto.set_index(node_id.index());
            proto.set_level(u32::from(node_id.level()));
            meta.mut_nodes().push(proto);
        }
        meta
    };

    let mut buf_writer = BufWriter::new(File::create(output_directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();

    progress_bar.lock().unwrap().finish();

    Ok(())
}
