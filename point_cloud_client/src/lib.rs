use point_viewer::errors::*;
use point_viewer::octree::{
    BatchIterator, Octree, OctreeFactory, PointLocation, NUM_POINTS_PER_BATCH,
};
use point_viewer::PointData;

pub struct PointCloudClient {
    octrees: Vec<Octree>,
    pub num_points_per_batch: usize,
}

impl PointCloudClient {
    pub fn new(locations: &[String], octree_factory: OctreeFactory) -> Result<Self> {
        let octrees: Result<Vec<Octree>> = locations
            .iter()
            .map(|location| {
                octree_factory
                    .generate_octree(location)
                    .map(|octree| *octree)
            })
            .collect();
        octrees.map(|octrees| PointCloudClient {
            octrees,
            num_points_per_batch: NUM_POINTS_PER_BATCH,
        })
    }

    pub fn for_each_point_data<F>(&self, point_location: &PointLocation, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        // todo (catevita) to ensure thread safety the mutable function should be
        // either invoked from within the thread
        for octree in &self.octrees {
            let mut batch_iterator =
                BatchIterator::new(octree, point_location, self.num_points_per_batch);
            batch_iterator.try_for_each_batch(&mut func)?;
        }
        Ok(())
    }
}
