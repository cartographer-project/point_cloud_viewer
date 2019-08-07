#[cfg(test)]
mod tests {
    use crate::color::Color;
    use crate::errors::Result;
    use crate::octree::{self, build_octree, BatchIterator, PointLocation, PointQuery};
    use crate::{Point, PointsBatch};
    use cgmath::{EuclideanSpace, Point3, Vector3};
    use collision::{Aabb, Aabb3};
    use tempdir::TempDir;
    const NUM_POINTS: usize = 100_001;
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

        let mut points = vec![default_point; 2 * NUM_POINTS];
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

        let mut points = vec![default_point; NUM_POINTS];
        points[NUM_POINTS - 1].position = Vector3::new(-200., -40., 30.);

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
        let callback_func = |points_batch: PointsBatch| -> Result<()> {
            callback_count += 1;
            delivered_points += points_batch.position.len();
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
        let octree = build_test_octree();
        let location = PointQuery {
            location: PointLocation::AllPoints(),
            global_from_local: None,
        };
        let mut batch_iterator = BatchIterator::new(
            std::slice::from_ref(&octree),
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        //  requiring and returning less points than the total
        assert!(3 <= callback_count);
        assert!(3 * batch_size <= delivered_points);
        assert!(callback_count >= 3);
        assert!((3 + 1) * batch_size >= delivered_points);
        assert!(delivered_points as i32 - max_num_points as i32 >= 0);
    }

    #[test]
    fn test_batch_iterator_more_points() {
        let batch_size = NUM_POINTS / 2;
        // define function

        // requiring more points than in the octree
        let mut callback_count = 0;
        let mut delivered_points = 0;
        let max_num_points = NUM_POINTS + 30_000;
        println!(
            "batch_size= {} ,  num_points= {}",
            batch_size, max_num_points
        );
        let callback_func = |points_batch: PointsBatch| -> Result<()> {
            callback_count += 1;
            delivered_points += points_batch.position.len();
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
        let octree = build_test_octree();
        let location = PointQuery {
            location: PointLocation::AllPoints(),
            global_from_local: None,
        };

        let mut batch_iterator =
            BatchIterator::new(std::slice::from_ref(&octree), &location, batch_size, 2, 2);

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect("Test OK");
        assert!(delivered_points == NUM_POINTS); // only delivers all the points in the octree
    }

    #[test]
    //#[ignore]
    fn test_batch_iterator_big_octree() {
        let batch_size = 5000;
        // define function
        let mut callback_count: u64 = 0;
        let max_num_points = 13_001; // 2*batch size + 3000
        let mut delivered_points: u64 = 0;
        println!(
            "batch_size= {} ,  num_points= {}",
            batch_size, max_num_points
        );
        let callback_func = |points_batch: PointsBatch| -> Result<()> {
            callback_count += 1;
            delivered_points += points_batch.position.len() as u64;
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
        let octree = build_big_test_octree();
        let location = PointQuery {
            location: PointLocation::AllPoints(),
            global_from_local: None,
        };
        let mut batch_iterator = BatchIterator::new(
            std::slice::from_ref(&octree),
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = batch_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert!(callback_count >= 3);
        assert!((3 + 1) * batch_size as u64 >= delivered_points);
        assert!(delivered_points as i32 - max_num_points as i32 >= 0);
        assert!(delivered_points as i32 - max_num_points as i32 <= batch_size as i32);
    }

}
