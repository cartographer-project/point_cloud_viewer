use point_viewer::data_provider::{DataProvider, DataProviderFactory};
use point_viewer::errors::*;
use point_viewer::iterator::{ParallelIterator, PointCloud, PointQuery};
use point_viewer::math::AABB;
use point_viewer::octree::Octree;
use point_viewer::s2_cells::S2Cells;
use point_viewer::{PointsBatch, NUM_POINTS_PER_BATCH};

enum PointClouds {
    Octrees(Vec<Octree>),
    S2Cells(Vec<S2Cells>),
}

pub struct PointCloudClient {
    point_clouds: PointClouds,
    aabb: AABB<f64>,
    num_points_per_batch: usize,
    num_threads: usize,
    buffer_size: usize,
}

impl PointCloudClient {
    pub fn bounding_box(&self) -> &AABB<f64> {
        &self.aabb
    }

    fn for_each<C, F>(&self, point_cloud: &[C], point_query: &PointQuery, mut func: F) -> Result<()>
    where
        C: PointCloud,
        F: FnMut(PointsBatch) -> Result<()>,
    {
        let mut parallel_iterator = ParallelIterator::new(
            point_cloud,
            point_query,
            self.num_points_per_batch,
            self.num_threads,
            self.buffer_size,
        );
        parallel_iterator.try_for_each_batch(&mut func)
    }

    pub fn for_each_point_data<F>(&self, point_query: &PointQuery, func: F) -> Result<()>
    where
        F: FnMut(PointsBatch) -> Result<()>,
    {
        match &self.point_clouds {
            PointClouds::Octrees(octrees) => self.for_each(octrees, point_query, func),
            PointClouds::S2Cells(s2_cells) => self.for_each(s2_cells, point_query, func),
        }
    }
}

pub struct PointCloudClientBuilder<'a> {
    locations: &'a [String],
    data_provider_factory: DataProviderFactory,
    num_points_per_batch: usize,
    num_threads: usize,
    buffer_size: usize,
}

impl<'a> PointCloudClientBuilder<'a> {
    pub fn new(locations: &'a [String]) -> Self {
        Self {
            locations,
            data_provider_factory: DataProviderFactory::new(),
            num_points_per_batch: NUM_POINTS_PER_BATCH,
            num_threads: std::cmp::max(1, num_cpus::get() - 1),
            buffer_size: 4,
        }
    }

    pub fn data_provider_factory(mut self, data_provider_factory: DataProviderFactory) -> Self {
        self.data_provider_factory = data_provider_factory;
        self
    }

    pub fn num_points_per_batch(mut self, num_points_per_batch: usize) -> Self {
        self.num_points_per_batch = num_points_per_batch;
        self
    }

    pub fn num_threads(mut self, num_threads: usize) -> Self {
        self.num_threads = num_threads;
        self
    }

    pub fn buffer_size(mut self, buffer_size: usize) -> Self {
        self.buffer_size = buffer_size;
        self
    }

    pub fn build(self) -> Result<PointCloudClient> {
        if self.locations.is_empty() {
            return Err("No locations specified for point cloud client.".into());
        }
        let data_providers = self
            .locations
            .iter()
            .map(|location| self.data_provider_factory.generate_data_provider(location))
            .collect::<Result<Vec<Box<dyn DataProvider>>>>()?;
        let mut aabb: Option<AABB<f64>> = None;
        let unite = |bbox: &AABB<f64>, with: &mut Option<AABB<f64>>| {
            let b = with.get_or_insert(bbox.clone());
            b.grow(*bbox.min());
            b.grow(*bbox.max());
        };
        let first_meta = data_providers[0].meta_proto()?;
        let point_clouds = if first_meta.version <= 11 || first_meta.has_octree() {
            PointClouds::Octrees(
                data_providers
                    .into_iter()
                    .map(|provider| {
                        Octree::from_data_provider(provider).map(|octree| {
                            unite(octree.bounding_box(), &mut aabb);
                            octree
                        })
                    })
                    .collect::<Result<Vec<Octree>>>()?,
            )
        } else {
            PointClouds::S2Cells(
                data_providers
                    .into_iter()
                    .map(|provider| {
                        S2Cells::from_data_provider(provider).map(|s2_cells| {
                            unite(s2_cells.bounding_box(), &mut aabb);
                            s2_cells
                        })
                    })
                    .collect::<Result<Vec<S2Cells>>>()?,
            )
        };

        Ok(PointCloudClient {
            point_clouds,
            aabb: aabb.unwrap_or_else(AABB::zero),
            num_points_per_batch: self.num_points_per_batch,
            num_threads: self.num_threads,
            buffer_size: self.buffer_size,
        })
    }
}
