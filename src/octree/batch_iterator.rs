use crate::errors::Result;
use crate::math::PointCulling;
use crate::math::{AllPoints, Isometry3, Obb, OrientedBeam};
use crate::octree::{self, FilteredPointsIterator};
use crate::{LayerData, Point, PointData};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
use fnv::FnvHashMap;
use std::collections::VecDeque;

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

/// current implementation of the points stored in BatchIterator
pub struct BatchIterator<F> {
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    local_from_global: Option<Isometry3<f64>>,
    point_iterator: FilteredPointsIterator<F>,
}

impl<F> BatchIterator<F> {
    fn new(
        num_points_per_batch: usize,
        local_from_global: Option<Isometry3<f64>>,
        point_iterator: FilteredPointsIterator<F>,
    ) -> Self {
        BatchIterator {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            local_from_global,
            point_iterator,
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

    /// copy into pointdata
    fn extract_pointdata(&mut self) -> Option<PointData> {
        if self.position.is_empty() {
            return None;
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
        Some(PointData {
            position: self.position.split_off(0),
            layers,
        })
    }
}

impl<F> Iterator for BatchIterator<F>
where
    F: Fn(&Point) -> bool,
{
    type Item = PointData;
    fn next(&mut self) -> Option<Self::Item> {
        let max_points = self.position.capacity();
        for _num_point in 0..max_points {
            match self.point_iterator.next() {
                Some(point) => {
                    self.push_point(point);
                }
                None => return self.extract_pointdata(),
            }
        }
        self.extract_pointdata()
    }
}

/// compute a function while iterating on a batch of points
pub fn try_for_each_point_batch<'a, G>(
    octree: &'a octree::Octree,
    point_query: &'a PointQuery,
    batch_size: usize,
    mut func: G,
) -> Result<()>
where
    G: FnMut(PointData) -> Result<()>,
{
    let mut point_batch_vec = get_point_data_iterator(octree, point_query, batch_size);
    // operate on nodes
    while let Some(mut point_batch) = point_batch_vec.pop_front() {
        while let Some(pointdata) = point_batch.next() {
            if let Err(err) = func(pointdata) {
                return Err(err);
            }
        }
    }

    Ok(())
}

pub fn get_point_data_iterator<'a>(
    octree: &'a octree::Octree,
    point_query: &'a PointQuery,
    batch_size: usize,
) -> VecDeque<Box<BatchIterator<impl Fn(&Point) -> bool>>> {
    let local_from_global = point_query.global_from_local.clone().map(|t| t.inverse());
    // nodes iterator: retrieve nodes
    let point_batch_vec: VecDeque<Box<BatchIterator<_>>> = octree
        .nodes_in_location(point_query)
        .map(|id| {
            Box::new(BatchIterator::new(
                batch_size,
                local_from_global.clone(),
                octree.points_in_node(point_query, id),
            ))
        })
        .collect();
    point_batch_vec
}
