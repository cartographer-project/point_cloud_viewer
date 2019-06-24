use crate::errors::*;
use crate::math::PointCulling;
use crate::math::{AllPoints, Isometry3, Obb, OrientedBeam};
use crate::octree::{self, Octree};
use crate::{LayerData, Point, PointData};
use cgmath::{Matrix4, Vector3, Vector4};
use collision::Aabb3;
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
struct PointStream {
    position: Vec<Vector3<f64>>,
    color: Vec<Vector4<u8>>,
    intensity: Vec<f32>,
    local_from_global: Option<Isometry3<f64>>,
}

impl PointStream {
    fn new(num_points_per_batch: usize, local_from_global: Option<Isometry3<f64>>) -> Self {
        PointStream {
            position: Vec::with_capacity(num_points_per_batch),
            color: Vec::with_capacity(num_points_per_batch),
            intensity: Vec::with_capacity(num_points_per_batch),
            local_from_global,
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
    fn callback(&mut self) -> Option<PointData> {
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

    fn push_point_and_callback(&mut self, point: Point) -> Option<PointData> {
        self.push_point(point);
        if self.position.len() == self.position.capacity() {
            self.callback()
        } else {
            None // TODO
        }
    }
}

/// Iterator on point batches
pub struct BatchIterator<'a> {
    octree: &'a Octree,
    point_location: &'a PointQuery,
    batch_size: usize,
}

impl<'a> BatchIterator<'a> {
    pub fn new(
        octree: &'a octree::Octree,
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
    pub fn try_for_each_batch<F>(&mut self, mut func: F) -> Result<()>
    where
        F: FnMut(PointData) -> Result<()>,
    {
        //TODO(catevita): mutable function parallelization
        let local_from_global = self
            .point_location
            .global_from_local
            .clone()
            .map(|t| t.inverse());
        let mut point_stream = PointStream::new(self.batch_size, local_from_global);
        // nodes iterator: retrieve nodes
        let node_id_iterator = self.octree.nodes_in_location(self.point_location);
        // operate on nodes
        for node_id in node_id_iterator {
            let point_iterator = self.octree.points_in_node(self.point_location, node_id);
            for point in point_iterator {
                if let Some(pointdata) = point_stream.push_point_and_callback(point) {
                    //here it could be retuned none, even if the buffer still works on the node. how can be this improved?
                    func(pointdata)?;
                }
            } //last iteration
            if let Some(pointdata) = point_stream.callback() {
                func(pointdata)?;
            }
        }
        // TODO(catevita): return point data through mpsc channel
        // TODO(catevita): apply mut function to received data

        Ok(())
    }
}
