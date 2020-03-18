// Code related to X-Ray generation.

use crate::colormap::{Colormap, Jet, Monochrome, PURPLISH};
use crate::inpaint::perform_inpainting;
use crate::utils::get_image_path;
use crate::{proto, CURRENT_VERSION};
use cgmath::{Decomposed, EuclideanSpace, Point2, Point3, Quaternion, Vector2, Vector3};
use clap::arg_enum;
use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use image::{self, GenericImage, Rgba, RgbaImage};
use imageproc::map::map_colors;
use num::clamp;
use point_cloud_client::PointCloudClient;
use point_viewer::attributes::AttributeData;
use point_viewer::color::{Color, WHITE, TRANSPARENT};
use point_viewer::geometry::Obb;
use point_viewer::iterator::{PointLocation, PointQuery};
use point_viewer::math::{ClosedInterval, Isometry3};
use point_viewer::utils::create_syncable_progress_bar;
use point_viewer::{match_1d_attr_data, PointsBatch};
use protobuf::Message;
use quadtree::{ChildIndex, Node, NodeId, Rect};
use rayon::iter::{IntoParallelRefIterator, ParallelIterator};
use stats::OnlineStats;
use std::collections::{HashMap, HashSet};
use std::error::Error;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;
use std::sync::Arc;

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

impl TileBackgroundColorArgument {
    pub fn to_color(&self) -> Color<u8> {
            match self {
                TileBackgroundColorArgument::white => WHITE.to_u8(),
                TileBackgroundColorArgument::transparent => TRANSPARENT.to_u8(),
            }
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

// Maps from attribute name to the bin size
type Binning = Option<(String, f64)>;

#[derive(Debug)]
pub enum ColoringStrategyKind {
    XRay,
    Colored(Binning),

    // Min and max intensities.
    ColoredWithIntensity(f32, f32, Binning),

    // Colored in heat-map colors by stddev. Takes the max stddev to clamp on.
    ColoredWithHeightStddev(f32, ColormapArgument),
}

impl ColoringStrategyKind {
    pub fn new_strategy(&self) -> Box<dyn ColoringStrategy> {
        use ColoringStrategyKind::*;
        match self {
            XRay => Box::new(XRayColoringStrategy::new()),
            Colored(binning) => Box::new(PointColorColoringStrategy::new(binning.clone())),
            ColoredWithIntensity(min_intensity, max_intensity, binning) => Box::new(
                IntensityColoringStrategy::new(*min_intensity, *max_intensity, binning.clone()),
            ),
            ColoredWithHeightStddev(max_stddev, ColormapArgument::jet) => {
                Box::new(HeightStddevColoringStrategy::new(*max_stddev, Jet {}))
            }
            ColoredWithHeightStddev(max_stddev, ColormapArgument::purplish) => Box::new(
                HeightStddevColoringStrategy::new(*max_stddev, Monochrome(PURPLISH)),
            ),
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
    fn get_pixel_color(&self, x: u32, y: u32) -> Option<Color<u8>>;

    fn attributes(&self) -> HashSet<String> {
        HashSet::default()
    }
}

trait BinnedColoringStrategy {
    fn binning(&self) -> &Binning;
    fn bins(&self, points_batch: &PointsBatch) -> Vec<i64> {
        match self.binning() {
            Some((attrib_name, size)) => {
                let attr_data = points_batch
                    .attributes
                    .get(attrib_name)
                    .expect("Binning attribute needs to be available in points batch.");
                macro_rules! rhs {
                    ($dtype:ident, $data:ident, $size:expr) => {
                        $data.iter().map(|e| (*e as f64 / *$size) as i64).collect()
                    };
                }
                match_1d_attr_data!(attr_data, rhs, size)
            }
            None => vec![0; points_batch.position.len()],
        }
    }
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
            let z_buckets = self.z_buckets.entry((d_loc.x, d_loc.y)).or_default();
            z_buckets.insert(d_loc.z);
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Option<Color<u8>> {
        self.z_buckets.get(&(x, y)).map(|z| {
            let saturation = (z.len() as f64).ln() / self.max_saturation;
            let value = ((1. - saturation) * 255.) as u8;
            Color {
                red: value,
                green: value,
                blue: value,
                alpha: 255,
            }
        })
    }
}

#[derive(Default)]
struct PerColumnData<T> {
    // The sum of all seen values.
    sum: T,
    // The number of all points that landed in this column.
    count: usize,
}

type IntensityPerColumnData = FnvHashMap<(u32, u32), FnvHashMap<i64, PerColumnData<f32>>>;

struct IntensityColoringStrategy {
    min: f32,
    max: f32,
    binning: Binning,
    per_column_data: IntensityPerColumnData,
}

impl IntensityColoringStrategy {
    fn new(min: f32, max: f32, binning: Binning) -> Self {
        IntensityColoringStrategy {
            min,
            max,
            binning,
            per_column_data: FnvHashMap::default(),
        }
    }
}

impl BinnedColoringStrategy for IntensityColoringStrategy {
    fn binning(&self) -> &Binning {
        &self.binning
    }
}

impl ColoringStrategy for IntensityColoringStrategy {
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        let bins = self.bins(points_batch);
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
                let per_column_data = self
                    .per_column_data
                    .entry((discretized_locations[i].x, discretized_locations[i].y))
                    .or_default();
                let bin_data = per_column_data.entry(bins[i]).or_default();
                bin_data.sum += intensity;
                bin_data.count += 1;
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Option<Color<u8>> {
        self.per_column_data.get(&(x, y)).map(|c| {
            let mean = (c
                .values()
                .map(|bin_data| bin_data.sum / bin_data.count as f32)
                .sum::<f32>()
                / c.len() as f32)
                .max(self.min)
                .min(self.max);
            let brighten = (mean - self.min).ln() / (self.max - self.min).ln();
            Color {
                red: brighten,
                green: brighten,
                blue: brighten,
                alpha: 1.,
            }
            .to_u8()
        })
    }

    fn attributes(&self) -> HashSet<String> {
        let mut attributes = HashSet::default();
        attributes.insert("intensity".into());
        if let Some((attr_name, _)) = &self.binning {
            attributes.insert(attr_name.clone());
        }
        attributes
    }
}

type PointColorPerColumnData = FnvHashMap<(u32, u32), FnvHashMap<i64, PerColumnData<Color<f32>>>>;

struct PointColorColoringStrategy {
    binning: Binning,
    per_column_data: PointColorPerColumnData,
}

impl PointColorColoringStrategy {
    fn new(binning: Binning) -> Self {
        Self {
            binning,
            per_column_data: FnvHashMap::default(),
        }
    }
}

impl BinnedColoringStrategy for PointColorColoringStrategy {
    fn binning(&self) -> &Binning {
        &self.binning
    }
}

impl ColoringStrategy for PointColorColoringStrategy {
    fn process_discretized_point_data(
        &mut self,
        points_batch: &PointsBatch,
        discretized_locations: Vec<Point3<u32>>,
    ) {
        let bins = self.bins(points_batch);
        let color_attribute = points_batch
            .attributes
            .get("color")
            .expect("Coloring was requested, but point data without color found.");
        if let AttributeData::U8Vec3(color_vec) = color_attribute {
            for i in 0..color_vec.len() {
                let color = Color::<u8> {
                    red: color_vec[i][0],
                    green: color_vec[i][1],
                    blue: color_vec[i][2],
                    alpha: 255,
                }
                .to_f32();
                let per_column_data = self
                    .per_column_data
                    .entry((discretized_locations[i].x, discretized_locations[i].y))
                    .or_default();
                let bin_data = per_column_data.entry(bins[i]).or_default();
                bin_data.sum += color;
                bin_data.count += 1;
            }
        }
    }

    fn get_pixel_color(&self, x: u32, y: u32) -> Option<Color<u8>> {
        self.per_column_data.get(&(x, y)).map(|c| {
            (c.values()
                .map(|bin_data| bin_data.sum / bin_data.count as f32)
                .sum::<Color<f32>>()
                / c.len() as f32)
                .to_u8()
        })
    }

    fn attributes(&self) -> HashSet<String> {
        let mut attributes = HashSet::default();
        attributes.insert("color".into());
        if let Some((attr_name, _)) = &self.binning {
            attributes.insert(attr_name.clone());
        }
        attributes
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

    fn get_pixel_color(&self, x: u32, y: u32) -> Option<Color<u8>> {
        self.per_column_data.get(&(x, y)).map(|c| {
            let saturation = clamp(c.stddev() as f32, 0., self.max_stddev) / self.max_stddev;
            self.colormap.for_value(saturation)
        })
    }
}

/// Build a parent image created of the 4 children tiles. All tiles are optionally, in which case
/// they are left white in the resulting image. The input images must be square with length N,
/// the returned image is square with length 2*N.
pub fn build_parent(children: &[Option<RgbaImage>], tile_background_color: Color<u8>) -> RgbaImage {
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
    let mut large_image = RgbaImage::from_pixel(
        child_size_px * 2,
        child_size_px * 2,
        Rgba::from(tile_background_color),
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
            large_image.copy_from(img, xoffs, yoffs).unwrap();
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
    pub inpaint_distance_px: u8,
    pub root_node_id: NodeId,
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
    let mut attributes = coloring_strategy.attributes();
    attributes.extend(parameters.filter_intervals.keys().cloned());
    let point_query = PointQuery {
        attributes: attributes.iter().map(|a| a.as_ref()).collect(),
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

    let mut image = RgbaImage::new(image_size.x, image_size.y);
    let background_color = Rgba::from(TRANSPARENT.to_u8());
    for (x, y, i) in image.enumerate_pixels_mut() {
        let pixel_color = coloring_strategy.get_pixel_color(x, y);
        *i = pixel_color.map(Rgba::from).unwrap_or(background_color);
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

pub fn build_xray_quadtree(
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
    let (created_leaf_node_ids_tx, created_leaf_node_ids_rx) = crossbeam::channel::unbounded();

    let root_node_id = parameters.root_node_id;
    let root_level = root_node_id.level();
    assert!(
        root_level <= deepest_level,
        "Specified root node id is outside quadtree."
    );
    let root_node = Node::from_node_id_and_root_bounding_rect(root_node_id, bounding_rect.clone());
    let mut leaf_nodes = Vec::with_capacity(4usize.pow((deepest_level - root_level).into()));
    let mut nodes_to_traverse = Vec::with_capacity((4 * leaf_nodes.capacity() - 1) / 3);
    nodes_to_traverse.push(root_node);
    while let Some(node) = nodes_to_traverse.pop() {
        if node.level() == deepest_level {
            leaf_nodes.push(node);
        } else {
            for i in 0..4 {
                nodes_to_traverse.push(node.get_child(&ChildIndex::from_u8(i)));
            }
        }
    }
    let progress_bar = create_syncable_progress_bar(
        leaf_nodes.len(),
        &format!("Building level {}", deepest_level),
    );
    rayon::scope(|scope| {
        while let Some(node) = leaf_nodes.pop() {
            let created_leaf_node_ids_tx_clone = created_leaf_node_ids_tx.clone();
            let strategy: Box<dyn ColoringStrategy> = coloring_strategy_kind.new_strategy();
            let progress_bar = Arc::clone(&progress_bar);
            scope.spawn(move |_| {
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
                    created_leaf_node_ids_tx_clone.send(node.id).unwrap();
                }
                progress_bar.lock().unwrap().inc();
            });
        }
    });
    progress_bar.lock().unwrap().finish_println("");
    drop(created_leaf_node_ids_tx);
    let created_leaf_node_ids: FnvHashSet<NodeId> = created_leaf_node_ids_rx.into_iter().collect();

    perform_inpainting(
        output_directory,
        parameters.inpaint_distance_px,
        &created_leaf_node_ids,
    )
    .expect("Inpainting failed.");

    let progress_bar =
        create_syncable_progress_bar(created_leaf_node_ids.len(), "Assigning background color");
    let background_color = Rgba::from(parameters.tile_background_color);
    created_leaf_node_ids.par_iter().for_each(|node_id| {
        let image_path = get_image_path(output_directory, *node_id);
        let mut image = image::open(&image_path).unwrap().to_rgba();
        // Depending on the implementation of the inpainting function above we may get pixels
        // that are not fully opaque or fully transparent. This is why we choose a threshold
        // in the middle to consider pixels as background or foreground and could be reevaluated
        // in the future.
        image = map_colors(&image, |p| if p[3] < 128 { background_color } else { p });
        image.save(&image_path).unwrap();
        progress_bar.lock().unwrap().inc();
    });
    progress_bar.lock().unwrap().finish_println("");

    let mut current_level_nodes = created_leaf_node_ids;
    let mut all_nodes = current_level_nodes.clone();

    for current_level in (root_level..deepest_level).rev() {
        current_level_nodes = current_level_nodes
            .iter()
            .filter_map(|node| node.parent_id())
            .collect();
        build_level(
            output_directory,
            tile.size_px,
            current_level,
            &current_level_nodes,
            parameters.tile_background_color,
        );
        all_nodes.extend(&current_level_nodes);
    }

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

        for node_id in all_nodes {
            let mut proto = proto::NodeId::new();
                proto.set_index(node_id.index());
                proto.set_level(u32::from(node_id.level()));
            meta.mut_nodes().push(proto);
        }
        meta
    };

    let meta_pb_name = format!("{}.pb", root_node_id).replace("r", "meta");
    let mut buf_writer = BufWriter::new(File::create(output_directory.join(meta_pb_name)).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();

    progress_bar.lock().unwrap().finish();

    Ok(())
}

pub fn build_level(
    output_directory: &Path,
    tile_size_px: u32,
    current_level: u8,
    nodes: &FnvHashSet<NodeId>,
    tile_background_color: Color<u8>,
) {
    let progress_bar =
        create_syncable_progress_bar(nodes.len(), &format!("Building level {}", current_level));
    nodes.par_iter().for_each(|node| {
        build_node(output_directory, *node, tile_size_px, tile_background_color);
        progress_bar.lock().unwrap().inc();
    });
    progress_bar.lock().unwrap().finish_println("");
}

fn build_node(
    output_directory: &Path,
    node_id: NodeId,
    tile_size_px: u32,
    tile_background_color: Color<u8>,
) {
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
        if png.exists() {
            children[id as usize] = Some(image::open(&png).unwrap().to_rgba());
        }
    }
    if children.iter().any(|child| child.is_some()) {
        let large_image = build_parent(&children, tile_background_color);
        let image = image::DynamicImage::ImageRgba8(large_image).resize(
            tile_size_px,
            tile_size_px,
            image::imageops::FilterType::Lanczos3,
        );
        image
            .as_rgba8()
            .unwrap()
            .save(&get_image_path(output_directory, node_id))
            .unwrap();
    }
}
