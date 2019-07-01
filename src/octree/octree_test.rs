#[cfg(test)]
mod tests {
    use crate::color::Color;
    use crate::errors::Result;
    use crate::generation::build_octree;
    use crate::octree::{self, BatchIterator, Octree, PointLocation, PointQuery};
    use crate::{Point, PointData};
    use cgmath::{EuclideanSpace, Point3, Vector3};
    use collision::{Aabb, Aabb3};
    use tempdir::TempDir;

    fn build_big_test_octree() -> Box<octree::Octree> {
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

        let mut points = vec![default_point; 200_000];
        points[3].position = Vector3::new(-2_702_846.0, -4_291_151.0, 3_855_012.0); // ECEF STANFORD

        let p = Point3::new(6_400_000.0, 6_400_000.0, 6_400_000.0);
        let bounding_box = Aabb3::new(-1.0 * p, p);

        let pool = scoped_pool::Pool::new(10);
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(&pool, &tmp_dir, 1.0, bounding_box, points.into_iter());
        crate::octree::on_disk::octree_from_directory(tmp_dir.into_path()).unwrap()
    }

    fn build_test_octree() -> Box<octree::Octree> {
        let default_point = Point {
            position: Vector3::new(0.0, 0.0, 0.0),
            color: Color {
                red: 255,
                green: 0,
                blue: 0,
                alpha: 255,
            },
            intensity: None,
        };

        let mut points = vec![default_point; 100_001];
        points[100_000].position = Vector3::new(-200., -40., 30.);

        let bounding_box = Aabb3::zero().grow(Point3::from_vec(points[100_000].position));

        let pool = scoped_pool::Pool::new(10);
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(&pool, &tmp_dir, 1.0, bounding_box, points.into_iter());
        crate::octree::on_disk::octree_from_directory(tmp_dir.into_path()).unwrap()
    }

    #[test]
    fn test_batch_iterator() {
        let batch_size = 5000;
        // define function
        let mut callback_count: usize = 0;
        let max_num_points = 13_000; // 2*batch size + 3000
        let mut delivered_points = 0;
        println!(
            "batch_size= {} ,  num_points= {}",
            batch_size, max_num_points
        );
        let callback_func = |point_data: PointData| -> Result<()> {
            callback_count += 1;
            delivered_points += point_data.position.len();
            println!(
                "Callback_count {:}, delivered points {:}",
                callback_count, delivered_points
            );
            if delivered_points >= max_num_points {
                println!("Callback: Max Points reached!");
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Interrupted,
                    format!("Maximum number of {} points reached.", max_num_points),
                )
                .into());
            }
            Ok(())
        };

        // octree and iterator
        let octree_vec: [Octree; 1] = [*build_test_octree()];
        let location = PointQuery {
            location: PointLocation::AllPoints(),
            global_from_local: None,
        };
        let mut batch_iterator = BatchIterator::new(
            &octree_vec,
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert_eq!(3, callback_count);
        assert_eq!(3 * batch_size, delivered_points);
        assert!(delivered_points as i32 - max_num_points as i32 >= 0);
    }

    #[test]
    //#[ignore]
    fn test_batch_iterator_big_octree() {
        let batch_size = 5000;
        // define function
        let mut callback_count: u64 = 0;
        let max_num_points = 13_000; // 2*batch size + 3000
        let mut delivered_points: u64 = 0;
        println!(
            "batch_size= {} ,  num_points= {}",
            batch_size, max_num_points
        );
        let callback_func = |point_data: PointData| -> Result<()> {
            callback_count += 1;
            delivered_points += point_data.position.len() as u64;
            println!(
                "Callback_count {:}, delivered points {:}",
                callback_count, delivered_points
            );
            if delivered_points >= max_num_points {
                return Err(std::io::Error::new(
                    std::io::ErrorKind::Interrupted,
                    format!("Maximum number of {} points reached.", max_num_points),
                )
                .into());
            }
            Ok(())
        };

        // octree and iterator
        let octree_vec: [Octree; 1] = [*build_big_test_octree()];
        let location = PointQuery {
            location: PointLocation::AllPoints(),
            global_from_local: None,
        };
        let mut batch_iterator = BatchIterator::new(
            &octree_vec,
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert!(callback_count >= 3);
        assert!(callback_count * batch_size as u64 >= delivered_points);
        assert!(delivered_points as i32 - max_num_points as i32 >= 0);
    }

}
