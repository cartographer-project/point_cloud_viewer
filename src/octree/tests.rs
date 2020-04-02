use crate::data_provider::OnDiskDataProvider;
use crate::errors::Result;
use crate::geometry::Aabb;
use crate::iterator::{ParallelIterator, PointQuery};
use crate::octree::{build_octree, Octree};
use crate::{AttributeData, NumberOfPoints, PointsBatch};
use nalgebra::{Point3, Vector3};
use tempdir::TempDir;

const NUM_POINTS: usize = 100_001;

impl NumberOfPoints for std::vec::IntoIter<PointsBatch> {
    fn num_points(&self) -> usize {
        self.clone().map(|b| b.position.len()).sum()
    }
}

fn build_test_octree() -> Octree {
    let mut batch = PointsBatch {
        position: vec![Point3::new(0.0, 0.0, 0.0); NUM_POINTS],
        attributes: vec![(
            "color".to_string(),
            AttributeData::U8Vec3(vec![Vector3::new(255, 0, 0); NUM_POINTS]),
        )]
        .into_iter()
        .collect(),
    };

    batch.position[NUM_POINTS - 1] = Point3::new(-200., -40., 30.);

    let bounding_box = Aabb::new(batch.position[0], batch.position[NUM_POINTS - 1]);

    let tmp_dir = TempDir::new("octree").unwrap();

    build_octree(
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

struct Consumer {
    max_num_points: usize,
    num_received_points: usize,
    num_received_callbacks: usize,
}

impl Consumer {
    fn new(max_num_points: usize) -> Self {
        Self {
            max_num_points,
            num_received_points: 0,
            num_received_callbacks: 0,
        }
    }

    // Consumes points until it reaches max_num_points, then errors out
    fn consume(&mut self, points_batch: PointsBatch) -> Result<()> {
        self.num_received_callbacks += 1;
        self.num_received_points += points_batch.position.len();
        eprintln!(
            "Callback_count {}, delivered points {}",
            self.num_received_callbacks, self.num_received_points
        );
        if self.num_received_points >= self.max_num_points {
            eprintln!("Callback: Max Points reached!");
            return Err(std::io::Error::new(
                std::io::ErrorKind::Interrupted,
                format!("Maximum number of {} points reached.", self.max_num_points),
            )
            .into());
        }
        Ok(())
    }
}

#[test]
fn test_batch_iterator() {
    let batch_size = 5000;
    let max_num_points = 13_000;
    let mut c = Consumer::new(max_num_points);
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

    parallel_iterator
        .try_for_each_batch(|points_batch| c.consume(points_batch))
        .expect_err("Iterator did not error even though callback errored.");

    assert!(c.num_received_points >= c.max_num_points);
    // The number of points doesn't fit in two batches
    assert_eq!(c.num_received_callbacks, 3);
    // The callback received full batches
    assert_eq!(c.num_received_points, 3 * batch_size);
}

#[test]
fn test_batch_iterator_more_points() {
    let batch_size = NUM_POINTS / 2;
    // This test asks for more points than exist in the octree.
    // Expected result: The number of points returned is equal to the number of points in the octree.
    let max_num_points = NUM_POINTS + 30_000;
    let mut c = Consumer::new(max_num_points);

    // octree and iterator
    let octree = build_test_octree();
    let location = PointQuery {
        attributes: vec!["color"],
        ..Default::default()
    };

    let octree_slice: &[Octree] = std::slice::from_ref(&octree);
    let mut parallel_iterator = ParallelIterator::new(octree_slice, &location, batch_size, 2, 2);

    parallel_iterator
        .try_for_each_batch(|points_batch| c.consume(points_batch))
        .expect("Iterator errored even though callback should not have errored.");
    assert_eq!(c.num_received_points, NUM_POINTS);
}
