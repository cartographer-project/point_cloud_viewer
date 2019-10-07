use collision::{Aabb, Aabb3, Union};
use point_viewer::data_provider::{DataProvider, DataProviderFactory};
use point_viewer::errors::*;
use point_viewer::iterator::{ParallelIterator, PointQuery};
use point_viewer::octree::Octree;
use point_viewer::s2_cells::S2Cells;
use point_viewer::{PointsBatch, NUM_POINTS_PER_BATCH};

enum PointClouds {
    Octrees(Vec<Octree>),
    S2Cells(Vec<S2Cells>),
}

pub struct PointCloudClient {
    point_clouds: PointClouds,
    aabb: Aabb3<f64>,
    pub num_points_per_batch: usize,
    pub num_threads: usize,
    pub buffer_size: usize,
}

impl PointCloudClient {
    pub fn new(locations: &[String], data_provider_factory: DataProviderFactory) -> Result<Self> {
        if locations.is_empty() {
            return Err("No locations specified for point cloud client.".into());
        }
        let data_providers = locations
            .iter()
            .map(|location| data_provider_factory.generate_data_provider(location))
            .collect::<Result<Vec<Box<dyn DataProvider>>>>()?;
        let point_clouds = if data_providers[0].meta_proto()?.has_octree() {
            PointClouds::Octrees(
                data_providers
                    .into_iter()
                    .map(|provider| Octree::from_data_provider(provider))
                    .collect::<Result<Vec<Octree>>>()?,
            )
        } else {
            PointClouds::S2Cells(
                data_providers
                    .into_iter()
                    .map(|provider| S2Cells::from_data_provider(provider))
                    .collect::<Result<Vec<S2Cells>>>()?,
            )
        };
        let mut aabb = Aabb3::zero();
        match &point_clouds {
            PointClouds::Octrees(octrees) => {
                if let Some((first_octree, remaining_octrees)) = octrees.split_first() {
                    aabb = *first_octree.bounding_box();
                    for octree in remaining_octrees {
                        aabb = aabb.union(octree.bounding_box());
                    }
                }
            }
            PointClouds::S2Cells(_s2_cells) => {
                // TODO(feuerste): Add bounding box to s2 meta.
                unimplemented!();
            }
        }

        Ok(PointCloudClient {
            point_clouds,
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
        match &self.point_clouds {
            PointClouds::Octrees(octrees) => {
                let mut parallel_iterator = ParallelIterator::new(
                    octrees,
                    point_location,
                    self.num_points_per_batch,
                    self.num_threads,
                    self.buffer_size,
                );
                parallel_iterator.try_for_each_batch(&mut func)?;
            }
            PointClouds::S2Cells(s2_cells) => {
                let mut parallel_iterator = ParallelIterator::new(
                    s2_cells,
                    point_location,
                    self.num_points_per_batch,
                    self.num_threads,
                    self.buffer_size,
                );
                parallel_iterator.try_for_each_batch(&mut func)?;
            }
        }
        Ok(())
    }
}
