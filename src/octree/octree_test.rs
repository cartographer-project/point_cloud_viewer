#[cfg(test)]
mod tests {
    use crate::color::Color;
    use crate::errors::Result;
    use crate::generation::build_octree;
    use crate::math::AllPoints;
    use crate::octree::{self, BatchIterator, PointLocation, NUM_POINTS_PER_BATCH};
    use crate::{Point, PointData};
    use cgmath::{Point3, Vector3};
    use collision::Aabb3;
    use std::sync::Arc;
    use tempdir::TempDir;

    fn build_test_octree(batch_size: usize) -> Box<octree::Octree> {
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

        let mut points = vec![default_point; 4 * batch_size];
        points[3 * batch_size].position = Vector3::new(-2_702_846.0, -4_291_151.0, 3_855_012.0); // ECEF STANFORD

        let p = Point3::new(6_400_000.0, 6_400_000.0, 6_400_000.0);
        let bounding_box = Aabb3::new(-1.0 * p, p);

        let pool = scoped_pool::Pool::new(10);
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(&pool, &tmp_dir, 1.0, bounding_box, points.into_iter());
        crate::octree::on_disk::octree_from_directory(tmp_dir.into_path()).unwrap()
    }

    #[test]
    fn test_batch_iterator() {
        let batch_size = NUM_POINTS_PER_BATCH / 10;
        // define function
        let mut point_count: i64 = 0;
        let mut print_count: i64 = 1;
        let num_points: i64 = 25 * batch_size as i64 / 10;
        let mut last_batch_size: i64 = 0;
        println!("batch_size= {} ,  num_points= {}", batch_size, num_points);
        let callback_func = |point_data: PointData| -> Result<()> {
            last_batch_size = point_data.position.len() as i64;
            point_count += last_batch_size;
            if point_count >= print_count * 2 * batch_size as i64 {
                print_count += 1;
                println!("Streamed {} points", point_count);
            }
            if point_count >= num_points {
                println!(
                    "Maximum number of {} points reached with last batch of {} points.",
                    num_points, last_batch_size,
                );
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Interrupted,
                    format!(
                        "Maximum number of {} points reached with last batch of {} points.",
                        num_points, last_batch_size,
                    ),
                )
                .into());
            }
            Ok(())
        };

        // octree and iterator
        let octree = build_test_octree(batch_size);
        let location = PointLocation {
            culling: Arc::new(Box::new(AllPoints {})),
            global_from_local: None,
        };
        let mut batch_iterator = BatchIterator::new(&octree, &location, batch_size);

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert!(point_count - num_points >= 0);
        assert!(point_count - last_batch_size - num_points <= 0);
        assert_eq!(2, print_count);
    }

}
