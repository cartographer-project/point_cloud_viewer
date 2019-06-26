use collision::{Aabb, Aabb3, Union};
use point_viewer::errors::*;
use point_viewer::octree::{
    BatchIterator, Octree, OctreeFactory, PointQuery, NUM_POINTS_PER_BATCH,
};
use point_viewer::PointData;

pub struct PointCloudClient {
    octrees: Vec<Octree>,
    aabb: Aabb3<f64>,
    pub num_points_per_batch: usize,
}

impl PointCloudClient {
    pub fn new(locations: &[String], octree_factory: OctreeFactory) -> Result<Self> {
        let octrees = locations
            .iter()
            .map(|location| {
                octree_factory
                    .generate_octree(location)
                    .map(|octree| *octree)
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
        })
    }

    pub fn bounding_box(&self) -> &Aabb3<f64> {
        &self.aabb
    }

    pub fn for_each_point_data<F>(&self, point_location: &PointQuery, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        let mut batch_iterator =
            BatchIterator::new(&self.octrees, point_location, self.num_points_per_batch);
        batch_iterator.try_for_each_batch(&mut func)?;
        Ok(())
    }
}
