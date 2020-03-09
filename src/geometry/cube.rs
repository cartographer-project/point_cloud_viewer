//! An axis-aligned cube.

use crate::math::sat::{self, ConvexPolyhedron, Intersector, Relation};
use crate::math::PointCulling;
use arrayvec::ArrayVec;
use cgmath::{BaseFloat, Point3, Vector3};
use collision::{Aabb, Contains};
use num_traits::Bounded;

pub use collision::Aabb3;

impl<S> PointCulling<S> for Aabb3<S>
where
    S: 'static + BaseFloat + Sync + Send + Bounded,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        Contains::contains(self, p)
    }

    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        sat::sat(
            ArrayVec::from([Vector3::unit_x(), Vector3::unit_y(), Vector3::unit_z()]),
            &self.to_corners(),
            &aabb.to_corners(),
        ) != Relation::Out
    }
}

impl<S> ConvexPolyhedron<S> for Aabb3<S>
where
    S: BaseFloat,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        self.to_corners()
    }

    fn intersector(&self) -> Intersector<S> {
        let mut unit_axes = ArrayVec::new();
        unit_axes.push(Vector3::unit_x());
        unit_axes.push(Vector3::unit_y());
        unit_axes.push(Vector3::unit_z());
        let corners = self.compute_corners();
        Intersector {
            corners,
            edges: unit_axes.clone(),
            face_normals: unit_axes,
        }
    }
}

#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3<f64>,
    edge_length: f64,
}

impl Cube {
    pub fn bounding(aabb: &Aabb3<f64>) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: aabb.min,
            edge_length,
        }
    }

    pub fn to_aabb3(&self) -> Aabb3<f64> {
        Aabb3::new(self.min(), self.max())
    }

    pub fn new(min: Point3<f64>, edge_length: f64) -> Self {
        Cube { min, edge_length }
    }

    pub fn edge_length(&self) -> f64 {
        self.edge_length
    }

    pub fn min(&self) -> Point3<f64> {
        self.min
    }

    pub fn max(&self) -> Point3<f64> {
        Point3::new(
            self.min.x + self.edge_length,
            self.min.y + self.edge_length,
            self.min.z + self.edge_length,
        )
    }

    /// The center of the box.
    pub fn center(&self) -> Vector3<f64> {
        let min = self.min();
        let max = self.max();
        Vector3::new(
            (min.x + max.x) / 2.,
            (min.y + max.y) / 2.,
            (min.z + max.z) / 2.,
        )
    }
}
