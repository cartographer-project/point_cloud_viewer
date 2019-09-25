use cgmath::{EuclideanSpace, Matrix4, Point3, Transform, Vector3};
use collision::{Aabb, Aabb3};
use point_viewer::color::RED;
use point_viewer::math::local_frame_from_lat_lng;
use point_viewer::{AttributeData, Point, PointsBatch};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use std::collections::BTreeMap;

#[derive(Clone)]
pub struct RandomPointsOnEarth {
    rng: StdRng,
    half_width: f64,
    half_height: f64,
    ecef_from_local: Matrix4<f64>,
}

pub const SEED: u64 = 80_293_751_234;

impl RandomPointsOnEarth {
    pub fn new(width: f64, height: f64, seed: u64) -> Self {
        let mut rng = StdRng::seed_from_u64(seed);
        let lat = rng.gen_range(-90.0, 90.0);
        let lon = rng.gen_range(-180.0, 180.0);
        let ecef_from_local = local_frame_from_lat_lng(lat, lon);
        RandomPointsOnEarth {
            rng,
            half_width: width * 0.5,
            half_height: height * 0.5,
            ecef_from_local,
        }
    }

    pub fn next_pos(&mut self) -> Point3<f64> {
        let x = self.rng.gen_range(-self.half_width, self.half_width);
        let y = self.rng.gen_range(-self.half_width, self.half_width);
        let z = self.rng.gen_range(-self.half_height, self.half_height);
        let pt_local = Point3::new(x, y, z);
        self.ecef_from_local.transform_point(pt_local)
    }

    pub fn bbox(&mut self) -> Aabb3<f64> {
        let local_min = Point3::new(-self.half_width, -self.half_width, -self.half_height);
        let local_max = Point3::new(self.half_width, self.half_width, self.half_height);
        Aabb3::new(local_min, local_max).transform(&self.ecef_from_local)
    }

    pub fn ecef_from_local(&self) -> Matrix4<f64> {
        self.ecef_from_local.clone()
    }
}

impl Iterator for RandomPointsOnEarth {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        let pos = self.next_pos();
        Some(Point {
            position: pos.to_vec(),
            color: RED.to_u8(),
            intensity: None,
        })
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
