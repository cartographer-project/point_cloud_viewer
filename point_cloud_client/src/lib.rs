use collision::{Aabb, Aabb3, Union};
use point_viewer::data_provider::DataProviderFactory;
use point_viewer::errors::*;
use point_viewer::iterator::{ParallelIterator, PointQuery};
use point_viewer::octree::Octree;
use point_viewer::{PointsBatch, NUM_POINTS_PER_BATCH};

pub struct PointCloudClient {
    octrees: Vec<Octree>,
    aabb: Aabb3<f64>,
    pub num_points_per_batch: usize,
    pub num_threads: usize,
    pub buffer_size: usize,
}

impl PointCloudClient {
    pub fn new(locations: &[String], data_provider_factory: DataProviderFactory) -> Result<Self> {
        let octrees = locations
            .iter()
            .map(|location| {
                data_provider_factory
                    .generate_data_provider(location)
                    .and_then(|provider| Octree::from_data_provider(provider))
            })
            .collect::<Result<Vec<Octree>>>()?;
        let mut aabb = Aabb3::zero();
        if let Some((first_octree, remaining_octrees)) = octrees.split_first() {
            aabb = *first_octree.bounding_box();
            for octree in remaining_octrees {
                aabb = aabb.union(octree.bounding_box());
            }
        }
        Ok(PointCloudClient {
            octrees,
            aabb,
            num_points_per_batch: NUM_POINTS_PER_BATCH,
            num_threads: num_cpus::get() - 1,
            buffer_size: 4,
        })
    }

    pub fn bounding_box(&self) -> &Aabb3<f64> {
        &self.aabb
    }

    pub fn for_each_point_data<F>(&self, point_location: &PointQuery, mut func: F) -> Result<()>
    where
        F: FnMut(PointsBatch) -> Result<()>,
    {
        let mut parallel_iterator = ParallelIterator::new(
            &self.octrees,
            point_location,
            self.num_points_per_batch,
            self.num_threads,
            self.buffer_size,
        );
        parallel_iterator.try_for_each_batch(&mut func)?;
        Ok(())
    }
}
