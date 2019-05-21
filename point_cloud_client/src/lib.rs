use collision::{Aabb, Aabb3};
use point_viewer::errors::*;
use point_viewer::octree::{
    BatchIterator, Octree, OctreeFactory, PointLocation, NUM_POINTS_PER_BATCH,
};
use point_viewer::PointData;

pub struct PointCloudClient {
    octrees: Vec<Octree>,
    aabb: Aabb3<f64>,
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
        let mut aabb = Aabb3::zero();
        if let Ok(octrees) = &octrees {
            if let Some(first_octree) = octrees.first() {
                aabb = *first_octree.bounding_box();
                for octree in octrees.iter().skip(1) {
                    let octree_bbox = octree.bounding_box();
                    aabb = aabb.grow(octree_bbox.min()).grow(octree_bbox.max());
                }
            }
        }
        octrees.map(|octrees| PointCloudClient {
            octrees,
            aabb,
            num_points_per_batch: NUM_POINTS_PER_BATCH,
        })
    }

    pub fn bounding_box(&self) -> &Aabb3<f64> {
        &self.aabb
    }

    pub fn for_each_point_data<F>(&self, point_location: &PointLocation, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        for octree in &self.octrees {
            let mut batch_iterator =
                BatchIterator::new(octree, point_location, self.num_points_per_batch);
            batch_iterator.try_for_each_batch(&mut func)?;
        }
        Ok(())
    }
}
