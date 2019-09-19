use cgmath::{
    Decomposed, Deg, EuclideanSpace, Matrix3, Matrix4, Point3, Quaternion, Rotation, Transform,
    Vector3,
};
use collision::Aabb3;
use nav_types::{ECEF, WGS84};
use point_viewer::color::RED;
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

impl RandomPointsOnEarth {
    pub fn new(width: f64, height: f64) -> Self {
        let mut rng = StdRng::seed_from_u64(80_293_751_234);
        let lat = rng.gen_range(-90.0, 90.0);
        let lon = rng.gen_range(-180.0, 180.0);
        let ecef_from_local = Self::local_frame_from_lat_lng(lat, lon);
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
        // take center of global bbox as initial value for min and max corner
        let mut ecef_min = Point3::from_vec(self.ecef_from_local.w.truncate());
        let mut ecef_max = Point3::from_vec(self.ecef_from_local.w.truncate());
        Aabb3::new(local_min, local_max)
            .to_corners()
            .iter()
            .map(|pt| self.ecef_from_local.transform_point(*pt))
            .for_each(|corner| {
                ecef_min.x = ecef_min.x.min(corner.x);
                ecef_min.y = ecef_min.y.min(corner.y);
                ecef_min.z = ecef_min.z.min(corner.z);
                ecef_max.x = ecef_max.x.max(corner.x);
                ecef_max.y = ecef_max.y.max(corner.y);
                ecef_max.z = ecef_max.z.max(corner.z);
            });
        Aabb3::new(ecef_min, ecef_max)
    }

    // Returns transform needed to go from ECEF to local frame with the specified origin where
    // the axes are ENU (east, north, up <in the direction normal to the oblate spheroid
    // used as Earth's ellipsoid, which does not generally pass through the center of the Earth>)
    // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_ECEF_to_ENU
    // transcribed from `localFrameFromEcefWgs84()` in avsoftware/src/common/math/geo_math.cc
    fn local_frame_from_lat_lng(lat: f64, lon: f64) -> Matrix4<f64> {
        const PI_HALF: Deg<f64> = Deg(90.0);
        let lat_lng_alt = WGS84::new(lat, lon, 0.0);
        let origin = ECEF::from(lat_lng_alt);
        let origin_vector = Vector3::new(origin.x(), origin.y(), origin.z());
        println!("origin_vector {:?}", origin_vector);
        let rotation_matrix = Matrix3::from_angle_z(-PI_HALF)
            * Matrix3::from_angle_y(Deg(lat_lng_alt.latitude_degrees()) - PI_HALF)
            * Matrix3::from_angle_z(Deg(-lat_lng_alt.longitude_degrees()));
        let rotation = Quaternion::from(rotation_matrix);

        let frame = Decomposed {
            scale: 1.0,
            rot: rotation,
            disp: rotation.rotate_vector(-origin_vector),
        };
        Matrix4::from(frame)
    }
}

impl Iterator for RandomPointsOnEarth {
    type Item = Point;

    fn next(&mut self) -> Option<Point> {
        let pos = self.next_pos();
        // println!("pos {:?}, bbox {:?}", pos, self.bbox());
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
