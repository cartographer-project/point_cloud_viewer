// Code related to X-Ray generation.

use collision::{Aabb, Aabb3};
use fnv::{FnvHashMap, FnvHashSet};
use image;
use point_viewer::{octree, InternalIterator, Point, color::{Color, WHITE}};
use std::collections::hash_map::Entry;
use std::path::Path;

// The number of Z-buckets we subdivide our bounding cube into along the z-direction. This affects
// the saturation of a point in x-rays: the more buckets contain a point, the darker the pixel
// becomes.
const NUM_Z_BUCKETS: f32 = 1024.;

arg_enum! {
    #[derive(Debug)]
    #[allow(non_camel_case_types)]
    pub enum ColoringStrategyArgument {
        xray,
        colored,
        colored_with_intensity,
    }
}

#[derive(Debug)]
pub enum ColoringStrategyKind {
    XRay,
    Colored,

    // Min and max intensities.
    ColoredWithIntensity(f32, f32),
}
impl ColoringStrategyKind {
    pub fn new_strategy(&self) -> Box<ColoringStrategy> {
        match *self {
            ColoringStrategyKind::XRay => Box::new(XRayColoringStrategy::new()),
            ColoringStrategyKind::Colored => Box::new(PointColorColoringStrategy::default()),
            ColoringStrategyKind::ColoredWithIntensity(min_intensity, max_intensity) => {
                Box::new(IntensityColoringStrategy::new(min_intensity, max_intensity))
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
            return WHITE.to_u8();
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
        let intensity = p.intensity
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
            return WHITE.to_u8();
        }
        let c = &self.per_column_data[&(x, y)];
        let mean = (c.sum / c.count as f32).max(self.min).min(self.max);
        let brighten = (mean - self.min).ln() / (self.max - self.min).ln();
        Color {
            red: brighten,
            green: brighten,
            blue: brighten,
            alpha: 1.,
        }.to_u8()
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
            return WHITE.to_u8();
        }
        let c = &self.per_column_data[&(x, y)];
        Color {
            red: c.color_sum.red / c.count as f32,
            green: c.color_sum.green / c.count as f32,
            blue: c.color_sum.blue / c.count as f32,
            alpha: c.color_sum.alpha / c.count as f32,
        }.to_u8()
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
    octree.points_in_box(&bbox).for_each(|p| {
        seen_any_points = true;
        let x = (((p.position.x - bbox.min().x) / bbox.dim().x) * image_width as f32) as u32;
        let y = (((p.position.y - bbox.min().y) / bbox.dim().y) * image_height as f32) as u32;
        let z = (((p.position.z - bbox.min().z) / bbox.dim().z) * NUM_Z_BUCKETS) as u32;
        coloring_strategy.process_discretized_point(p, x, y, z);
    });

    if !seen_any_points {
        return false;
    }

    let mut image = image::RgbImage::new(image_width, image_height);
    for x in 0..image_width {
        for y in 0..image_height {
            let color = coloring_strategy.get_pixel_color(x, y);
            image.put_pixel(
                x,
                y,
                image::Rgb {
                    data: [color.red, color.green, color.blue],
                },
            );
        }
    }
    image.save(png_file).unwrap();
    true
}
