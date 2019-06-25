use crate::errors::*;
use crate::math::PointCulling;
use crate::math::{AllPoints, Isometry3, Obb, OrientedBeam};
use crate::octree::{self, FilteredPointsIterator, Octree};
use crate::{LayerData, Point, PointData};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
use crossbeam_deque::{Steal, Worker, Injector};
use fnv::FnvHashMap;

/// size for batch
pub const NUM_POINTS_PER_BATCH: usize = 500_000;

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone)]
pub enum PointLocation {
    AllPoints(),
    Aabb(Aabb3<f64>),
    Frustum(Matrix4<f64>),
    Obb(Obb<f64>),
    OrientedBeam(OrientedBeam<f64>),
}

#[derive(Clone, Debug)]
pub struct PointQuery {
    pub location: PointLocation,
    // If set, culling and the returned points are interpreted to be in local coordinates.
    pub global_from_local: Option<Isometry3<f64>>,
}

impl PointQuery {
    pub fn get_point_culling(&self) -> Box<PointCulling<f64>> {
        let culling: Box<PointCulling<f64>> = match &self.location {
            PointLocation::AllPoints() => return Box::new(AllPoints {}),
            PointLocation::Aabb(aabb) => Box::new(*aabb),
            PointLocation::Frustum(matrix) => Box::new(octree::Frustum::new(*matrix)),
            PointLocation::Obb(obb) => Box::new(obb.clone()),
            PointLocation::OrientedBeam(beam) => Box::new(beam.clone()),
        };
        match &self.global_from_local {
            Some(global_from_local) => culling.transform(&global_from_local),
            None => culling,
        }
    }
}
/// current implementation of the stream of points used in BatchIterator
struct PointStream<'a, F>
where
    F: Fn(PointData) -> Result<()>,
{
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    local_from_global: Option<Isometry3<f64>>,
    func: &'a F,
}

impl<'a, F> PointStream<'a, F>
where
    F: Fn(PointData) -> Result<()>,
{
    fn new(
        num_points_per_batch: usize,
        local_from_global: Option<Isometry3<f64>>,
        func: &'a F,
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
        //println!("internal callback sending last batch...");
        (self.func)(point_data)
    }

    fn push_point_and_callback<F2>(
        &mut self,
        point_iterator: FilteredPointsIterator<F2>,
    ) -> Result<()>
    where
        F2: Fn(&Point) -> bool,
    {
        for point in point_iterator {
            self.push_point(point);
            if self.position.len() == self.position.capacity() {
                self.callback()?;
            }
        }
        self.callback()?;
        Ok(())
    }
}

/// Iterator on point batches
pub struct BatchIterator<'a> {
    octrees: Vec<&'a octree::Octree>,
    point_location: &'a PointQuery,
    batch_size: usize,
}

impl<'a> BatchIterator<'a> {
    pub fn new(
        octrees: Vec<&'a octree::Octree>,
        point_location: &'a PointQuery,
        batch_size: usize,
    ) -> Self {
        BatchIterator {
            octree,
            point_location,
            batch_size,
        }
    }

    /// compute a function while iterating on a batch of points
    pub fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        // number of threads
        let num_threads = 10;
        // get thread safe fifo
        let jobs = Worker::<(NodeId, &Octree)>::new_fifo();
       self.octrees.iter().flat_map(|&octree| {
                octree
                    .nodes_in_location(self.point_location)
                    .zip(std::iter::repeat(octree))
            }).for_each(|tuple| jobs.push(tuple));


        let stealer = jobs.stealer();
        
        let local_from_global = self
            .point_location
            .global_from_local
            .clone()
            .map(|t| t.inverse());
        // operate on nodes: one thread for each node
        crossbeam::scope(|s| {
            let (tx, rx) = crossbeam::channel::bounded::<PointData>(2);
            for _ in 0.. num_threads {
                let tx = tx.clone();
                let local_from_global = local_from_global.clone();
                let point_location = self.point_location.clone();
                let batch_size = self.batch_size;

                s.spawn(move |_| {
                    let send_func = |batch: PointData| match tx.send(batch) {
                        Ok(_) => Ok(()),
                        Err(e) => Err(ErrorKind::Channel(format!(
                            "sending operation failed, nothing more to do {:?}",
                            e,
                        ))
                        .into()),
                    };
                    while let Some(node_id, octree) = stealer.pop() { 
                        let () = task;
                    let point_iterator = octree.points_in_node(&point_location, node_id);
                    let mut point_stream =
                        PointStream::new(batch_size, local_from_global, &send_func);
                    let _ = point_stream.push_point_and_callback(point_iterator);
                });
            }

            rx.iter().try_for_each(func)
        })
        .expect("child thread panic")
    }
}
