use cgmath::{EuclideanSpace, Matrix4, Point3, Transform, Vector3};
use collision::{Aabb, Aabb3};
use point_viewer::color::Color;
use point_viewer::math::local_frame_from_lat_lng;
use point_viewer::{AttributeData, NumberOfPoints, Point, PointsBatch};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::collections::BTreeMap;

#[derive(Clone)]
pub struct RandomPointsOnEarth {
    rng: StdRng,
    half_width: f64,
    half_height: f64,
    ecef_from_local: Matrix4<f64>,
    size: usize,
    count: usize,
}

pub const SEED: u64 = 80_293_751_232;

impl RandomPointsOnEarth {
    pub fn new(width: f64, height: f64, size: usize, seed: u64) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let lat = rng.gen_range(-90.0, 90.0);
        let lon = rng.gen_range(-180.0, 180.0);
        let ecef_from_local = local_frame_from_lat_lng(lat, lon);
        RandomPointsOnEarth {
            rng,
            half_width: width * 0.5,
            half_height: height * 0.5,
            ecef_from_local,
            size,
            count: 0,
        }
    }

    pub fn next_pos(&mut self) -> Point3<f64> {
        let x = self.rng.gen_range(-self.half_width, self.half_width);
        let y = self.rng.gen_range(-self.half_width, self.half_width);
        let z = self.rng.gen_range(-self.half_height, self.half_height);
        let pt_local = Point3::new(x, y, z);
        self.ecef_from_local.transform_point(pt_local)
    }

    pub fn bbox(&self) -> Aabb3<f64> {
        let local_min = Point3::new(-self.half_width, -self.half_width, -self.half_height);
        let local_max = Point3::new(self.half_width, self.half_width, self.half_height);
        Aabb3::new(local_min, local_max).transform(&self.ecef_from_local)
    }

    pub fn ecef_from_local(&self) -> &Matrix4<f64> {
        &self.ecef_from_local
    }
}

impl Iterator for RandomPointsOnEarth {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        if self.count == self.size {
            return None;
        }
        let pos = self.next_pos();
        let point = Point {
            position: pos.to_vec(),
            // Encode index in color, which is preserved in octrees.
            color: Color::<u8> {
                red: (self.count >> 16) as u8,
                green: (self.count >> 8) as u8,
                blue: self.count as u8,
                alpha: 0,
            },
            intensity: None,
        };
        self.count += 1;
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.size, Some(self.size))
    }
}

pub struct Batched<T>
where
    T: Iterator<Item = Point>,
{
    inner: T,
    batch: PointsBatch,
    batch_size: usize,
}

impl<T> Batched<T>
where
    T: Iterator<Item = Point>,
{
    fn new_batch(batch_size: usize) -> PointsBatch {
        let mut attrs = BTreeMap::new();
        attrs.insert(
            "color".to_owned(),
            AttributeData::U8Vec3(Vec::with_capacity(batch_size)),
        );
        PointsBatch {
            position: Vec::with_capacity(batch_size),
            attributes: attrs,
        }
    }

    pub fn new(inner: T, batch_size: usize) -> Self {
        let batch = Self::new_batch(batch_size);
        Batched {
            inner,
            batch,
            batch_size,
        }
    }
}

impl<T> NumberOfPoints for Batched<T>
where
    T: Iterator<Item = Point>,
{
    fn num_points(&self) -> usize {
        self.inner.size_hint().1.unwrap()
    }
}

impl<T> Iterator for Batched<T>
where
    T: Iterator<Item = Point>,
{
    type Item = PointsBatch;

    fn next(&mut self) -> Option<PointsBatch> {
        for _ in 0..self.batch_size {
            if let Some(pt) = self.inner.next() {
                self.batch.position.push(pt.position);
                let color = Vector3::new(pt.color.red, pt.color.green, pt.color.blue);
                self.batch
                    .get_attribute_vec_mut("color")
                    .unwrap()
                    .push(color);
            }
        }
        if !self.batch.position.is_empty() {
            let output_batch = std::mem::replace(&mut self.batch, Self::new_batch(self.batch_size));
            Some(output_batch)
        } else {
            None
        }
    }
}
