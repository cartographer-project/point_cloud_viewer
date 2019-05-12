use crate::errors::*;
use crate::math::OrientedBeam;
use crate::octree::{self, Octree};
use crate::{LayerData, Point, PointData};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
use fnv::FnvHashMap;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

///possible kind of iterators that can be evaluated in batch of points in BatchIterator
#[allow(clippy::large_enum_variant)]
pub enum PointLocation {
    Any(),
    Aabb(Aabb3<f64>),
    Frustum(Matrix4<f64>),
    OrientedBeam(OrientedBeam),
}

/// current implementation of the stream of points used in BatchIterator
struct PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    func: &'a mut F,
}

impl<'a, F> PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    fn new(num_points_per_batch: usize, func: &'a mut F) -> Self {
        PointStream {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            func,
        }
    }

    /// push point in batch
    fn push_point(&mut self, point: Point) {
        self.position.push(point.position);
        self.color.push(Vector4::new(
            point.color.red,
            point.color.green,
            point.color.blue,
            point.color.alpha,
        ));
        if let Some(point_intensity) = point.intensity {
            self.intensity.push(point_intensity);
        };
    }

    /// execute function on batch of points
    fn callback(&mut self) -> Result<()> {
        if self.position.is_empty() {
            return Ok(());
        }

        let mut layers = FnvHashMap::default();
        layers.insert(
            "color".to_string(),
            LayerData::U8Vec4(self.color.split_off(0)),
        );
        if !self.intensity.is_empty() {
            layers.insert(
                "intensity".to_string(),
                LayerData::F32(self.intensity.split_off(0)),
            );
        }
        let point_data = PointData {
            position: self.position.split_off(0),
            layers,
        };
        (self.func)(point_data)
    }

    fn push_point_and_callback(&mut self, point: Point) -> Result<()> {
        self.push_point(point);
        if self.position.len() == self.position.capacity() {
            return self.callback();
        }
        Ok(())
    }
}

/// Iterator on point batches
pub struct BatchIterator<'a> {
    _octree: &'a Octree,
    iterator: Box<Iterator<Item = Point> + 'a>,
    batch_size: usize,
}

impl<'a> BatchIterator<'a> {
    pub fn new(
        octree: &'a octree::Octree,
        location: &'a octree::batch_iterator::PointLocation,
        batch_size: usize,
    ) -> Self {
        let iterator: Box<Iterator<Item = Point>> = match location {
            PointLocation::Any() => Box::new(octree.all_points()),
            PointLocation::Aabb(aabb) => Box::new(octree.points_in_box(aabb)),
            PointLocation::Frustum(frustum) => Box::new(octree.points_in_frustum(frustum)),
            PointLocation::OrientedBeam(beam) => Box::new(octree.points_in_oriented_beam(beam)),
        };
        BatchIterator {
            _octree: octree,
            iterator,
            batch_size,
        }
    }

    /// compute a funtion while iterating on a batch of points
    pub fn try_for_each_batch<F>(&mut self, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        let mut point_stream = PointStream::new(self.batch_size, &mut func);
        'octree_loop: loop {
            let mut n = 0;
            while n < self.batch_size {
                match self.iterator.next() {
                    Some(point) => {
                        n += 1;
                        point_stream.push_point(point);
                    }
                    None => {
                        break 'octree_loop;
                    }
                }
            }
            // call at every batch, return if error
            match point_stream.callback() {
                Ok(()) => continue,
                Err(e) => return Err(e),
            }
        }
        // call on the last batch
        point_stream.callback()
    }

    pub fn try_for_each_batch2<F>(&mut self, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        let mut point_stream = PointStream::new(self.batch_size, &mut func);
        self.iterator
            .try_for_each(|point: Point| point_stream.push_point_and_callback(point))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::color::Color;
    use crate::generation::build_octree;
    use cgmath::Point3;
    use tempdir::TempDir;

    fn build_test_octree(batch_size: usize) -> Box<Octree> {
        let default_point = Point {
            position: Vector3::new(-2_699_182.0, -4_294_938.0, 3_853_373.0), //ECEF parking lot porter dr
            color: Color {
                red: 255,
                green: 0,
                blue: 0,
                alpha: 255,
            },
            intensity: None,
        };

        let mut points = vec![default_point; 4 * batch_size + 1];
        points[3 * batch_size].position = Vector3::new(-2_702_846.0, -4_291_151.0, 3_855_012.0); // ECEF STANFORD

        let p = Point3::new(6_400_000.0, 6_400_000.0, 6_400_000.0);
        let bounding_box = Aabb3::new(-1.0 * p, p);

        let pool = scoped_pool::Pool::new(10);
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(&pool, &tmp_dir, 1.0, bounding_box, points.into_iter());
        crate::octree::on_disk::octree_from_directory(tmp_dir.into_path()).unwrap()
    }

    #[test]
    //#[ignore]
    fn test_batch_iterator() {
        let batch_size = NUM_POINTS_PER_BATCH / 10;
        // define function
        let mut point_count: usize = 0;
        let mut print_count: usize = 1;
        let num_points = 25 * batch_size / 10;
        println!("batch_size= {} ,  num_points= {}", batch_size, num_points);
        let callback_func = |point_data: PointData| -> Result<()> {
            point_count += point_data.position.len();
            if point_count >= print_count * 2 * batch_size {
                print_count += 1;
                println!("Streamed {} points", point_count);
            }
            if point_count >= num_points {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Interrupted,
                    format!("Maximum number of {} points reached.", num_points),
                )
                .into());
            }
            Ok(())
        };

        // octree and iterator
        let octree = build_test_octree(batch_size);
        let location = PointLocation::Any();
        let mut batch_iterator = BatchIterator::new(&octree, &location, batch_size);

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert_eq!(3 * batch_size, point_count);
        assert_eq!(2, print_count);
    }

    #[test]
    fn test_batch_iterator_time() {
        let batch_size = NUM_POINTS_PER_BATCH / 10;
        // define function

        let callback_func = |point_data: PointData| -> Result<()> {
            println!("Test_batch_function");
            Ok(())
        };

        // octree and iterator
        let octree = build_test_octree(batch_size);
        let location = PointLocation::Any();
        let mut batch_iterator = BatchIterator::new(&octree, &location, batch_size);
        let time_test_loop_begin = std::time::Instant::now();
        batch_iterator.try_for_each_batch(callback_func);
        let time_test_loop_end = std::time::Instant::now();
        println!(
            "Current {:?}",
            time_test_loop_end.duration_since(time_test_loop_begin)
        );

        let mut batch_iterator2 = BatchIterator::new(&octree, &location, batch_size);
        let time_test_try_begin = std::time::Instant::now();
        batch_iterator2.try_for_each_batch2(callback_func);
        let time_test_try_end = std::time::Instant::now();
        println!(
            "Try for each {:?}",
            time_test_try_end.duration_since(time_test_try_begin)
        );
    }
}
