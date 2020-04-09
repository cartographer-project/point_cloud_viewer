//! Axis-aligned box and cube.

use crate::math::sat::{self, ConvexPolyhedron, Intersector, Relation};
use crate::math::PointCulling;
use crate::proto;
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Point3, RealField, Vector3};
use serde::{Deserialize, Serialize};
use std::iter::FromIterator;

/// An axis-aligned bounding box.
#[derive(Debug, PartialEq, Clone, Serialize, Deserialize)]
pub struct Aabb<S: RealField> {
    mins: Point3<S>,
    maxs: Point3<S>,
}

impl<S: RealField> Aabb<S> {
    pub fn new(mins: Point3<S>, maxs: Point3<S>) -> Self {
        Aabb {
            mins: nalgebra::inf(&mins, &maxs),
            maxs: nalgebra::sup(&mins, &maxs),
        }
    }

    pub fn zero() -> Self {
        Self {
            mins: Point3::origin(),
            maxs: Point3::origin(),
        }
    }

    pub fn min(&self) -> &Point3<S> {
        &self.mins
    }

    pub fn max(&self) -> &Point3<S> {
        &self.maxs
    }

    pub fn grow(&mut self, p: Point3<S>) {
        self.mins = nalgebra::inf(&self.mins, &p);
        self.maxs = nalgebra::sup(&self.maxs, &p);
    }

    pub fn contains(&self, p: &Point3<S>) -> bool {
        nalgebra::partial_le(&self.mins, p) && nalgebra::partial_lt(p, &self.maxs)
    }

    pub fn center(&self) -> Point3<S> {
        nalgebra::center(&self.mins, &self.maxs)
    }

    pub fn diag(&self) -> Vector3<S> {
        self.maxs - self.mins
    }

    pub fn transform(&self, transform: &Isometry3<S>) -> Aabb<S> {
        let corners = self.compute_corners();
        let transformed_first = transform.transform_point(&corners[0]);
        let base = Self::new(transformed_first, transformed_first);
        corners[1..].iter().fold(base, |mut u, &corner| {
            u.grow(transform.transform_point(&corner));
            u
        })
    }
}

impl From<&proto::AxisAlignedCuboid> for Aabb<f64> {
    fn from(aac: &proto::AxisAlignedCuboid) -> Self {
        let aac_min = aac.min.clone().unwrap_or_else(|| {
            let deprecated_min = aac.deprecated_min.clone().unwrap(); // Version 9
            proto::Vector3d::from(deprecated_min)
        });
        let aac_max = aac.max.clone().unwrap_or_else(|| {
            let deprecated_max = aac.deprecated_max.clone().unwrap(); // Version 9
            proto::Vector3d::from(deprecated_max)
        });
        Aabb::new(
            std::convert::From::from(&aac_min),
            std::convert::From::from(&aac_max),
        )
    }
}

impl From<&Aabb<f64>> for proto::AxisAlignedCuboid {
    fn from(bbox: &Aabb<f64>) -> Self {
        let mut aac = proto::AxisAlignedCuboid::new();
        aac.set_min(proto::Vector3d::from(bbox.min()));
        aac.set_max(proto::Vector3d::from(bbox.max()));
        aac
    }
}

impl<S> PointCulling<S> for Aabb<S>
where
    S: RealField + num_traits::Bounded,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains(p)
    }

    fn intersects_aabb(&self, aabb: &Aabb<S>) -> bool {
        sat::sat(
            ArrayVec::from([Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()]),
            &self.compute_corners(),
            &aabb.compute_corners(),
        ) != Relation::Out
    }
}

impl<S> ConvexPolyhedron<S> for Aabb<S>
where
    S: RealField,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        [
            Point3::new(self.mins.x, self.mins.y, self.mins.z),
            Point3::new(self.maxs.x, self.mins.y, self.mins.z),
            Point3::new(self.mins.x, self.maxs.y, self.mins.z),
            Point3::new(self.maxs.x, self.maxs.y, self.mins.z),
            Point3::new(self.mins.x, self.mins.y, self.maxs.z),
            Point3::new(self.maxs.x, self.mins.y, self.maxs.z),
            Point3::new(self.mins.x, self.maxs.y, self.maxs.z),
            Point3::new(self.maxs.x, self.maxs.y, self.maxs.z),
        ]
    }

    fn intersector(&self) -> Intersector<S> {
        let mut edges = ArrayVec::new();
        edges.push(Vector3::x_axis());
        edges.push(Vector3::y_axis());
        edges.push(Vector3::z_axis());
        let face_normals = ArrayVec::from_iter(edges.clone());
        Intersector {
            corners: self.compute_corners(),
            edges,
            face_normals,
        }
    }
}

/// A simple cube, representing an octree node.
#[derive(Debug, Clone)]
pub struct Cube {
    min: Point3<f64>,
    edge_length: f64,
}

impl Cube {
    pub fn bounding(aabb: &Aabb<f64>) -> Self {
        let edge_length = (aabb.max().x - aabb.min().x)
            .max(aabb.max().y - aabb.min().y)
            .max(aabb.max().z - aabb.min().z);
        Cube {
            min: *aabb.min(),
            edge_length,
        }
    }

    pub fn to_aabb(&self) -> Aabb<f64> {
        Aabb::new(self.min(), self.max())
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
