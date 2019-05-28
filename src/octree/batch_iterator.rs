use crate::errors::*;
use crate::math::{Isometry3, PointCulling};
use crate::octree::{self, FilteredPointsIterator, Octree};
use crate::{LayerData, Point, PointData};
use cgmath::{Vector3, Vector4};
use fnv::FnvHashMap;
use std::sync::Arc;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

pub struct PointLocation {
    pub culling: Arc<Box<PointCulling<f64>>>,
    // If set, culling and the returned points are interpreted to be in local coordinates.
    pub global_from_local: Option<Isometry3<f64>>,
}

/// current implementation of the stream of points used in BatchIterator
struct PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    local_from_global: Option<Isometry3<f64>>,
    func: &'a mut F,
}

impl<'a, F> PointStream<'a, F>
where
    F: FnMut(PointData) -> Result<()>,
{
    fn new(
        num_points_per_batch: usize,
        local_from_global: Option<Isometry3<f64>>,
        func: &'a mut F,
    ) -> Self {
        PointStream {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            local_from_global,
            func,
        }
    }

    /// push point in batch
    fn push_point(&mut self, point: Point) {
        let position = match &self.local_from_global {
            Some(local_from_global) => local_from_global * &point.position,
            None => point.position,
        };
        self.position.push(position);
        self.color.push(Vector4::new(
            point.color.red,
            point.color.green,
            point.color.blue,
            point.color.alpha,
        ));
        if let Some(point_intensity) = point.intensity {
            self.intensity.push(point_intensity);
        };
    }

    /// execute function on batch of points
    fn callback(&mut self) -> Result<()> {
        if self.position.is_empty() {
            return Ok(());
        }

        let mut layers = FnvHashMap::default();
        layers.insert(
            "color".to_string(),
            LayerData::U8Vec4(self.color.split_off(0)),
        );
        if !self.intensity.is_empty() {
            layers.insert(
                "intensity".to_string(),
                LayerData::F32(self.intensity.split_off(0)),
            );
        }
        let point_data = PointData {
            position: self.position.split_off(0),
            layers,
        };
        (self.func)(point_data)
    }

    fn push_node_and_callback(&mut self, mut node_iterator: FilteredPointsIterator) -> Result<()> {
        let mut current_num_points = 0;
        let max_points = self.position.capacity();
        loop {
            match node_iterator.next() {
                Some(point) => {
                    self.push_point(point);
                    current_num_points += 1;
                }
                None => return self.callback(),
            }
            if current_num_points >= max_points {
                current_num_points = 0;
                if let Err(err) = self.callback() {
                    return Err(err);
                }
            }
        }
    }
}

/// Iterator on point batches
pub struct BatchIterator<'a> {
    octree: &'a Octree,
    culling: Arc<Box<PointCulling<f64>>>,
    local_from_global: Option<Isometry3<f64>>,
    batch_size: usize,
}

impl<'a> BatchIterator<'a> {
    pub fn new(octree: &'a octree::Octree, location: &'a PointLocation, batch_size: usize) -> Self {
        let culling: Arc<Box<PointCulling<f64>>> = match &location.global_from_local {
            Some(global_from_local) => Arc::from(location.culling.transform(&global_from_local)),
            None => location.culling.clone(),
        };
        let local_from_global = location.global_from_local.clone().map(|t| t.inverse());
        BatchIterator {
            octree,
            culling,
            local_from_global,
            batch_size,
        }
    }

    /// compute a function while iterating on a batch of points
    pub fn try_for_each_batch<F>(&mut self, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        //todo (catevita) mutable function parallelization
        let mut point_stream =
            PointStream::new(self.batch_size, self.local_from_global.clone(), &mut func);
        // nodes iterator: retrieve nodes
        let iterator = self.octree.nodes_in_location(Arc::clone(&self.culling));
        // operate on nodes
        for id in iterator {
            if let Err(err) = point_stream
                .push_node_and_callback(self.octree.points_in_node(Arc::clone(&self.culling), id))
            {
                return Err(err);
            }
        }
        // todo return point data through mpsc channel
        // todo apply mut function to received data

        Ok(())
    }
}
