//! An asymmetric frustum with an arbitrary 3D pose.

use crate::math::sat::{ConvexPolyhedron, Intersector};
use crate::math::{Isometry3, PointCulling};
use arrayvec::ArrayVec;
use cgmath::{
    BaseFloat, Decomposed, InnerSpace, Matrix4, Perspective, Point3, Quaternion, Transform, Vector3,
};
use collision::{Aabb3, Relation};
use serde::{Deserialize, Serialize};

/// A frustum is defined in eye coordinates, where x points right, y points up,
/// and z points against the viewing direction. This is not how e.g. OpenCV
/// defines a camera coordinate system. To get from OpenCV camera coordinates
/// to eye coordinates, you need to rotate 180 deg around the x axis before
/// creating the perspective projection, see also the frustum unit test below.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frustum<S: BaseFloat> {
    /// The "query" coordinates are basically our world coordinates.
    query_from_clip: Matrix4<S>,
    /// The inverse of the above matrix, suitable for intersection tests.
    frustum: collision::Frustum<S>,
}

impl<S: BaseFloat> Frustum<S> {
    pub fn new(query_from_eye: Isometry3<S>, clip_from_eye: Perspective<S>) -> Self {
        let eye_from_query: Decomposed<Vector3<S>, Quaternion<S>> = query_from_eye.inverse().into();
        let clip_from_query =
            Matrix4::<S>::from(clip_from_eye) * Matrix4::<S>::from(eye_from_query);
        let query_from_clip = clip_from_query.inverse_transform().unwrap();
        let frustum = collision::Frustum::from_matrix4(clip_from_query).unwrap();
        Frustum {
            query_from_clip,
            frustum,
        }
    }
}

impl<S> PointCulling<S> for Frustum<S>
where
    S: 'static + BaseFloat + Sync + Send,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        match self.frustum.contains(point) {
            Relation::Cross => true,
            Relation::In => true,
            Relation::Out => false,
        }
    }
    fn intersects_aabb3(&self, aabb: &Aabb3<S>) -> bool {
        match self.frustum.contains(aabb) {
            Relation::Cross => true,
            Relation::In => true,
            Relation::Out => false,
        }
    }
}

impl<S> ConvexPolyhedron<S> for Frustum<S>
where
    S: BaseFloat,
{
    fn compute_corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| self.query_from_clip.transform_point(Point3::new(x, y, z));
        [
            corner_from(-S::one(), -S::one(), -S::one()),
            corner_from(-S::one(), -S::one(), S::one()),
            corner_from(-S::one(), S::one(), -S::one()),
            corner_from(-S::one(), S::one(), S::one()),
            corner_from(S::one(), -S::one(), -S::one()),
            corner_from(S::one(), -S::one(), S::one()),
            corner_from(S::one(), S::one(), -S::one()),
            corner_from(S::one(), S::one(), S::one()),
        ]
    }

    fn intersector(&self) -> Intersector<S> {
        let corners = self.compute_corners();

        let edges = ArrayVec::from([
            (corners[4] - corners[0]).normalize(), // x
            (corners[2] - corners[0]).normalize(), // y
            (corners[1] - corners[0]).normalize(), // z lower left
            (corners[3] - corners[2]).normalize(), // z upper left
            (corners[5] - corners[4]).normalize(), // z lower right
            (corners[7] - corners[6]).normalize(), // z upper right
        ]);

        let mut face_normals = ArrayVec::new();
        face_normals.push((edges[0].cross(edges[1])).normalize()); // Front and back sides
        face_normals.push((edges[0].cross(edges[2])).normalize()); // Lower side
        face_normals.push((edges[0].cross(edges[3])).normalize()); // Upper side
        face_normals.push((edges[1].cross(edges[2])).normalize()); // Left side
        face_normals.push((edges[1].cross(edges[4])).normalize()); // Right side

        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}
