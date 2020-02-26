#[cfg(test)]
mod tests {
    use crate::data_provider::OnDiskDataProvider;
    use crate::errors::Result;
    use crate::iterator::{ParallelIterator, PointQuery};
    use crate::octree::{build_octree, Octree};
    use crate::{AttributeData, NumberOfPoints, PointsBatch};
    use cgmath::{EuclideanSpace, Point3, Vector3};
    use collision::{Aabb, Aabb3};
    use tempdir::TempDir;

    const NUM_POINTS: usize = 100_001;

    impl NumberOfPoints for std::vec::IntoIter<PointsBatch> {
        fn num_points(&self) -> usize {
            self.clone().map(|b| b.position.len()).sum()
        }
    }

    fn build_big_test_octree() -> Octree {
        let mut batch = PointsBatch {
            //ECEF parking lot porter dr
            position: vec![Vector3::new(-2_699_182.0, -4_294_938.0, 3_853_373.0); 2 * NUM_POINTS],
            attributes: vec![(
                "color".to_string(),
                AttributeData::U8Vec3(vec![Vector3::new(255, 0, 0); 2 * NUM_POINTS]),
            )]
            .into_iter()
            .collect(),
        };

        batch.position[3] = Vector3::new(-2_702_846.0, -4_291_151.0, 3_855_012.0); // ECEF STANFORD

        let p = Point3::new(6_400_000.0, 6_400_000.0, 6_400_000.0);
        let bounding_box = Aabb3::new(-1.0 * p, p);

        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(10)
            .build()
            .expect("Could not create thread pool.");
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(
            &pool,
            &tmp_dir,
            1.0,
            bounding_box,
            vec![batch].into_iter(),
            &["color"],
        );
        Octree::from_data_provider(Box::new(OnDiskDataProvider {
            directory: tmp_dir.into_path(),
        }))
        .unwrap()
    }

    fn build_test_octree() -> Octree {
        let mut batch = PointsBatch {
            position: vec![Vector3::new(0.0, 0.0, 0.0); NUM_POINTS],
            attributes: vec![(
                "color".to_string(),
                AttributeData::U8Vec3(vec![Vector3::new(255, 0, 0); NUM_POINTS]),
            )]
            .into_iter()
            .collect(),
        };

        batch.position[NUM_POINTS - 1] = Vector3::new(-200., -40., 30.);

        let bounding_box = Aabb3::zero().grow(Point3::from_vec(batch.position[NUM_POINTS - 1]));

        let pool = rayon::ThreadPoolBuilder::new()
            .num_threads(10)
            .build()
            .expect("Could not create thread pool.");
        let tmp_dir = TempDir::new("octree").unwrap();

        build_octree(
            &pool,
            &tmp_dir,
            1.0,
            bounding_box,
            vec![batch].into_iter(),
            &["color"],
        );
        Octree::from_data_provider(Box::new(OnDiskDataProvider {
            directory: tmp_dir.into_path(),
        }))
        .unwrap()
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
            attributes: vec!["color"],
            ..Default::default()
        };
        let octree_slice: &[Octree] = std::slice::from_ref(&octree);
        let mut parallel_iterator = ParallelIterator::new(
            octree_slice,
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = parallel_iterator
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
            attributes: vec!["color"],
            ..Default::default()
        };

        let octree_slice: &[Octree] = std::slice::from_ref(&octree);
        let mut parallel_iterator =
            ParallelIterator::new(octree_slice, &location, batch_size, 2, 2);

        let _err_stop = parallel_iterator
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
            attributes: vec!["color"],
            ..Default::default()
        };
        let octree_slice: &[Octree] = std::slice::from_ref(&octree);
        let mut parallel_iterator = ParallelIterator::new(
            octree_slice,
            &location,
            batch_size,
            std::cmp::max(1, num_cpus::get() - 1),
            4,
        );

        let _err_stop = parallel_iterator
            .try_for_each_batch(callback_func)
            .expect_err("Test error");

        assert!(callback_count >= 3);
        assert!((3 + 1) * batch_size as u64 >= delivered_points);
        assert!(delivered_points as i32 - max_num_points as i32 >= 0);
        assert!(delivered_points as i32 - max_num_points as i32 <= batch_size as i32);
    }
}
