use crate::errors::*;
use crate::geometry::{Frustum, Obb};
use crate::math::{AllPoints, ClosedInterval, PointCulling};
use crate::read_write::{Encoding, NodeIterator};
use crate::{match_1d_attr_data, AttributeData, PointsBatch};
use cgmath::Point3;
use collision::Aabb3;
use crossbeam::deque::{Injector, Steal, Worker};
use num_traits::ToPrimitive;
use s2::cellunion::CellUnion;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, HashMap};

#[allow(clippy::large_enum_variant)]
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PointLocation {
    AllPoints,
    Aabb(Aabb3<f64>),
    Frustum(Frustum<f64>),
    Obb(Obb<f64>),
    S2Cells(CellUnion),
}

impl Default for PointLocation {
    fn default() -> Self {
        PointLocation::AllPoints
    }
}

impl PointLocation {
    pub fn get_point_culling(&self) -> Box<dyn PointCulling<f64>> {
        match &self {
            PointLocation::AllPoints => Box::new(AllPoints {}),
            PointLocation::Aabb(aabb) => Box::new(*aabb),
            PointLocation::Frustum(frustum) => Box::new(frustum.clone()),
            PointLocation::Obb(obb) => Box::new(obb.clone()),
            PointLocation::S2Cells(cell_union) => Box::new(cell_union.clone()),
        }
    }
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PointQuery<'a> {
    #[serde(borrow)]
    pub attributes: Vec<&'a str>,
    pub location: PointLocation,
    #[serde(borrow)]
    pub filter_intervals: HashMap<&'a str, ClosedInterval<f64>>,
}

/// Iterator over the points of a point cloud node within the specified PointCulling
/// Essentially a specialized version of the Filter iterator adapter
pub struct FilteredIterator<'a> {
    pub culling: Box<dyn PointCulling<f64>>,
    pub filter_intervals: &'a HashMap<&'a str, ClosedInterval<f64>>,
    pub node_iterator: NodeIterator,
}

fn update_keep<T>(keep: &mut [bool], data: &[T], interval: &ClosedInterval<f64>)
where
    T: ToPrimitive,
{
    for (k, v) in keep.iter_mut().zip(data) {
        if let Some(v) = v.to_f64() {
            *k &= interval.contains(v);
        }
    }
}

impl<'a> Iterator for FilteredIterator<'a> {
    type Item = PointsBatch;

    fn next(&mut self) -> Option<PointsBatch> {
        let culling = &self.culling;
        self.node_iterator.next().map(|mut batch| {
            let mut keep: Vec<bool> = batch
                .position
                .iter()
                .map(|pos| {
                    culling.contains(&<Point3<f64> as cgmath::EuclideanSpace>::from_vec(*pos))
                })
                .collect();
            macro_rules! rhs {
                ($dtype:ident, $data:ident, $interval:expr) => {
                    update_keep(&mut keep, $data, $interval)
                };
            }
            for (attrib, interval) in self.filter_intervals {
                let attr_data = batch
                    .attributes
                    .get(*attrib)
                    .expect("Filter attribute needs to be specified as query attribute.");
                match_1d_attr_data!(attr_data, rhs, interval)
            }
            batch.retain(&keep);
            batch
        })
    }
}

/// current implementation of the stream of points used in ParallelIterator
struct PointStream<'a, F>
where
    F: Fn(PointsBatch) -> Result<()>,
{
    buf: PointsBatch,
    batch_size: usize,
    func: &'a F,
}

impl<'a, F> PointStream<'a, F>
where
    F: Fn(PointsBatch) -> Result<()>,
{
    fn new(batch_size: usize, func: &'a F) -> Self {
        PointStream {
            buf: PointsBatch {
                position: Vec::new(),
                attributes: BTreeMap::new(),
            },
            batch_size,
            func,
        }
    }

    /// execute function on batch of points
    fn callback(&mut self) -> Result<()> {
        if self.buf.position.is_empty() {
            return Ok(());
        }

        let at = std::cmp::min(self.buf.position.len(), self.batch_size);
        let mut res = self.buf.split_off(at);
        std::mem::swap(&mut res, &mut self.buf);
        (self.func)(res)
    }

    fn push_points_and_callback<I>(&mut self, node_iterator: I) -> Result<()>
    where
        I: Iterator<Item = PointsBatch>,
    {
        for mut batch in node_iterator {
            self.buf.append(&mut batch)?;
            while self.buf.position.len() >= self.batch_size {
                self.callback()?;
            }
        }
        Ok(())
    }

    /// Why only a single node? Because the nodes are distributed to several `PointStream`s working
    /// in parallel by the `ParallelIterator`.
    fn stream_points_for_query_in_node<P: PointCloud>(
        &mut self,
        point_cloud: &P,
        query: &'a PointQuery,
        node_id: P::Id,
        batch_size: usize,
    ) -> Result<()> {
        let culling = query.location.get_point_culling();
        let filter_intervals = &query.filter_intervals;
        let node_iterator = point_cloud.points_in_node(&query.attributes, node_id, batch_size)?;
        let filtered_iterator = FilteredIterator {
            culling,
            filter_intervals,
            node_iterator,
        };
        self.push_points_and_callback(filtered_iterator)
    }
}

// TODO(nnmm): Move this somewhere else
pub trait PointCloud: Sync {
    type Id: ToString + Send + Copy;
    fn nodes_in_location(&self, location: &PointLocation) -> Vec<Self::Id>;
    fn encoding_for_node(&self, id: Self::Id) -> Encoding;
    /// Return all points in the selected node.
    fn points_in_node(
        &self,
        attributes: &[&str],
        node_id: Self::Id,
        batch_size: usize,
    ) -> Result<NodeIterator>;
    fn bounding_box(&self) -> &Aabb3<f64>;
    /// Return the points matching the query in the selected node.
    fn stream_points_for_query_in_node<F>(
        &self,
        query: &PointQuery,
        node_id: Self::Id,
        batch_size: usize,
        callback: F,
    ) -> Result<()>
    where
        F: FnMut(PointsBatch) -> Result<()>,
    {
        let culling = query.location.get_point_culling();
        let filter_intervals = &query.filter_intervals;
        let node_iterator = self.points_in_node(&query.attributes, node_id, batch_size)?;
        let mut filtered_iterator = FilteredIterator {
            culling,
            filter_intervals,
            node_iterator,
        };
        filtered_iterator.try_for_each(callback)
    }
}

/// Iterator on point batches
pub struct ParallelIterator<'a, C> {
    point_clouds: &'a [C],
    point_query: &'a PointQuery<'a>,
    batch_size: usize,
    num_threads: usize,
    buffer_size: usize,
}

impl<'a, C> ParallelIterator<'a, C>
where
    C: PointCloud,
{
    pub fn new(
        point_clouds: &'a [C],
        point_query: &'a PointQuery<'a>,
        batch_size: usize,
        num_threads: usize,
        buffer_size: usize,
    ) -> Self {
        ParallelIterator {
            point_clouds,
            point_query,
            batch_size,
            num_threads,
            buffer_size,
        }
    }

    /// compute a function while iterating on a batch of points
    pub fn try_for_each_batch<F>(&mut self, func: F) -> Result<()>
    where
        F: FnMut(PointsBatch) -> Result<()>,
    {
        // get thread safe fifo
        let jobs = Injector::<(&C, C::Id)>::new();
        let mut number_of_jobs = 0;
        self.point_clouds
            .iter()
            .flat_map(|octree| {
                std::iter::repeat(octree).zip(octree.nodes_in_location(&self.point_query.location))
            })
            .for_each(|(node_id, octree)| {
                jobs.push((node_id, octree));
                number_of_jobs += 1;
            });

        // operate on nodes with limited number of threads
        crossbeam::scope(|s| {
            let (tx, rx) = crossbeam::channel::bounded::<PointsBatch>(self.buffer_size);
            for curr_thread in 0..self.num_threads {
                let tx = tx.clone();
                let point_query = &self.point_query;
                let batch_size = self.batch_size;
                let worker = Worker::new_fifo();
                let jobs = &jobs;

                s.spawn(move |_| {
                    let send_func = |batch: PointsBatch| match tx.send(batch) {
                        Ok(_) => Ok(()),
                        Err(e) => Err(ErrorKind::Channel(format!(
                            "Thread {}: sending operation failed, nothing more to do {:?}",
                            curr_thread, e,
                        ))
                        .into()),
                    };

                    // one pointstream per thread vs one per node allows to send more full point batches
                    let mut point_stream = PointStream::new(batch_size, &send_func);

                    while let Some((octree, node_id)) = worker.pop().or_else(|| {
                        std::iter::repeat_with(|| jobs.steal_batch_and_pop(&worker))
                            .find(|task| !task.is_retry())
                            .and_then(Steal::success)
                    }) {
                        // executing on the available next task if the function still requires it
                        match point_stream.stream_points_for_query_in_node(
                            octree,
                            &point_query,
                            node_id,
                            batch_size,
                        ) {
                            Ok(_) => continue,
                            Err(ref e) => {
                                match e.kind() {
                                    ErrorKind::Channel(ref _s) => break, // done with the function computation
                                    _ => panic!("ParallelIterator: Thread error {}", e), //some other error
                                }
                            }
                        }
                    }
                    // last batch of points: calling callback
                    if let Err(ref e) = point_stream.callback() {
                        match e.kind() {
                            ErrorKind::Channel(ref _s) => (), // done with the function computation
                            _ => panic!("ParallelIterator: Thread error {}", e), //some other error
                        }
                    }
                });
            }
            // ensure to close the channel after the threads exit
            drop(tx);

            // receiver collects all the messages
            rx.iter().try_for_each(func)
        })
        .expect("ParallelIterator: Panic in try_for_each_batch child thread")
    }
}
