use crate::errors::*;
use crate::math::PointCulling;
use crate::math::{AllPoints, Isometry3, Obb, OrientedBeam};
use crate::octree::{self, FilteredPointsIterator, Octree};
use crate::{LayerData, Point, PointData};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
use crossbeam::deque::{Injector, Worker};
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
    local_from_global: &'a Option<Isometry3<f64>>,
    func: &'a F,
}

impl<'a, F> PointStream<'a, F>
where
    F: Fn(PointData) -> Result<()>,
{
    fn new(
        num_points_per_batch: usize,
        local_from_global: &'a Option<Isometry3<f64>>,
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

    fn push_points_and_callback<Filter>(
        &mut self,
        point_iterator: FilteredPointsIterator<Filter>,
    ) -> Result<()>
    where
        Filter: Fn(&Point) -> bool,
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
    octrees: &'a [octree::Octree],
    point_location: &'a PointQuery,
    batch_size: usize,
    num_threads: usize,
}

impl<'a> BatchIterator<'a> {
    pub fn new(
        octrees: &'a [octree::Octree],
        point_location: &'a PointQuery,
        batch_size: usize,
        num_threads: usize,
    ) -> Self {
        BatchIterator {
            octrees,
            point_location,
            batch_size,
            num_threads,
        }
    }

    /// compute a function while iterating on a batch of points
    pub fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        // get thread safe fifo
        let jobs = Injector::<(&Octree, octree::NodeId)>::new();
        self.octrees
            .iter()
            .flat_map(|octree| {
                std::iter::repeat(octree).zip(
                    octree.nodes_in_location(self.point_location))
            })
            .for_each(|(node_id, octree)| {
                jobs.push((node_id, octree));
            });

        let local_from_global = self
            .point_location
            .global_from_local
            .as_ref()
            .map(|t| t.inverse());

        // operate on nodes with limited number of threads
        crossbeam::scope(|s| {
            let (tx, rx) = crossbeam::channel::bounded::<PointData>(2);
            for _ in 0..self.num_threads {
                let tx = tx.clone();
                let local_from_global = &local_from_global;
                let point_location = &self.point_location;
                let batch_size = self.batch_size;
                let worker = Worker::new_fifo();
                let jobs = &jobs;

                s.spawn(move |_| {
                    let send_func = |batch: PointData| match tx.send(batch) {
                        Ok(_) => Ok(()),
                        Err(e) => Err(ErrorKind::Channel(format!(
                            "sending operation failed, nothing more to do {:?}",
                            e,
                        ))
                        .into()),
                    };

                    // one pointstream per thread vs one per node allows to send more full point batches
                    let mut point_stream =
                        PointStream::new(batch_size, &local_from_global, &send_func);

                    while let Some((octree, node_id)) = worker.pop().or_else(|| {
                        std::iter::repeat_with(|| jobs.steal_batch_and_pop(&worker))
                            .find(|task| !task.is_retry())
                            .and_then(|task| task.success())
                    }) {
                            let point_iterator = octree.points_in_node(&point_location, node_id);
                            // executing on the available next task if the function still requires it
                            match point_stream.push_points_and_callback(point_iterator) {
                                Ok(_) => continue,
                                Err(ref e) => match e.kind() {
                                    ErrorKind::Channel(ref _s) => break, // done with the function computation
                                    _ => panic!("BatchIterator: Thread error {}", e), //some other error
                                },
                            }
                        }
                    
                });
            }

            rx.iter().try_for_each(func)
        })
        .expect("BatchIterator: Panic in try_for_each_batch child thread")
    }
}
