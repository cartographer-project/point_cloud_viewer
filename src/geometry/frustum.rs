//! An asymmetric frustum with an arbitrary 3D pose.

use crate::math::base::{HasAabbIntersector, PointCulling};
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Intersector};
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Matrix4, Perspective3, Point3, Unit, Vector3};
use serde::{Deserialize, Serialize};

/// A perspective projection matrix analogous to cgmath::Perspective.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Perspective{
    matrix: Matrix4<f64>,
}

impl Perspective {
    /// Left, right, bottom, and top are in radians.
    pub fn new(left: f64, right: f64, bottom: f64, top: f64, near: f64, far: f64) -> Self {
        assert!(
            left < right,
            "`left` must be smaller than `right`, found: left: {:?} right: {:?}",
            left,
            right
        );
        assert!(
            bottom < top,
            "`bottom` must be smaller than `top`, found: bottom: {:?} top: {:?}",
            bottom,
            top
        );
        assert!(
            near > 0.0 && near < far,
            "`near` must be greater than 0 and must be smaller than `far`, found: near: {:?} far: {:?}",
            near,
            far
        );

        let r0c0 = (2.0 * near) / (right - left);
        let r0c2 = (right + left) / (right - left);

        let r1c1 = (2.0 * near) / (top - bottom);
        let r1c2 = (top + bottom) / (top - bottom);

        let r2c2 = -(far + near) / (far - near);
        let r2c3 = -(2.0 * far * near) / (far - near);

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            r0c0, 0.0,  r0c2, 0.0,
            0.0,  r1c1, r1c2, 0.0,
            0.0,  0.0,  r2c2, r2c3,
            0.0,  0.0,  -1.0, 0.0,
        );
        Self { matrix }
    }

    pub fn as_matrix(&self) -> &Matrix4<f64> {
        &self.matrix
    }

    pub fn inverse(&self) -> Matrix4<f64> {
        let r0c0 = self.matrix[(0, 0)].recip();
        let r0c3 = self.matrix[(0, 2)] / self.matrix[(0, 0)];

        let r1c1 = self.matrix[(1, 1)].recip();
        let r1c3 = self.matrix[(1, 2)] / self.matrix[(1, 1)];

        let r3c2 = self.matrix[(2, 3)].recip();
        let r3c3 = self.matrix[(2, 2)] / self.matrix[(2, 3)];

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            r0c0,      0.0, 0.0, r0c3,
            0.0, r1c1,      0.0, r1c3,
            0.0, 0.0, 0.0, -1.0,
            0.0, 0.0, r3c2,      r3c3,
        );
        matrix
    }
}

impl From<Perspective3<f64>> for Perspective {
    fn from(per3: Perspective3<f64>) -> Self {
        Self {
            matrix: per3.to_homogeneous(),
        }
    }
}

/// A frustum is defined in eye coordinates, where x points right, y points up,
/// and z points against the viewing direction. This is not how e.g. OpenCV
/// defines a camera coordinate system. To get from OpenCV camera coordinates
/// to eye coordinates, you need to rotate 180 deg around the x axis before
/// creating the perspective projection, see also the frustum unit test below.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frustum {
    query_from_clip: Matrix4<f64>,
    clip_from_query: Matrix4<f64>,
}

impl Frustum {
    pub fn new(query_from_eye: Isometry3<f64>, clip_from_eye: Perspective) -> Self {
        let clip_from_query = clip_from_eye.as_matrix() * query_from_eye.inverse().to_homogeneous();
        let query_from_clip = query_from_eye.to_homogeneous() * clip_from_eye.inverse();
        Frustum {
            query_from_clip,
            clip_from_query,
        }
    }

    /// Fails if the matrix is not invertible.
    pub fn from_matrix4(clip_from_query: Matrix4<f64>) -> Option<Self> {
        let query_from_clip = clip_from_query.try_inverse()?;
        Some(Self {
            query_from_clip,
            clip_from_query,
        })
    }
}

impl PointCulling<f64> for Frustum {
    fn contains(&self, point: &Point3<f64>) -> bool {
        let p_clip = self.clip_from_query.transform_point(point);
        p_clip.coords.min() > -1.0
            && p_clip.coords.max() < 1.0
    }
}

has_aabb_intersector_for_convex_polyhedron!(Frustum);

impl ConvexPolyhedron<f64> for Frustum {
    #[rustfmt::skip]
    fn compute_corners(&self) -> [Point3<f64>; 8] {
        let corner_from = |x, y, z| self.query_from_clip.transform_point(&Point3::new(x, y, z));
        [
            corner_from(-1.0, -1.0, -1.0),
            corner_from(-1.0, -1.0,  1.0),
            corner_from(-1.0,  1.0, -1.0),
            corner_from(-1.0,  1.0,  1.0),
            corner_from( 1.0, -1.0, -1.0),
            corner_from( 1.0, -1.0,  1.0),
            corner_from( 1.0,  1.0, -1.0),
            corner_from( 1.0,  1.0,  1.0),
        ]
    }

    fn intersector(&self) -> Intersector<f64> {
        let corners = self.compute_corners();

        let mut edges: ArrayVec<[Unit<Vector3<f64>>; 12]> = ArrayVec::new();
        edges.push(Unit::new_normalize(corners[4] - corners[0])); // x
        edges.push(Unit::new_normalize(corners[2] - corners[0])); // y
        edges.push(Unit::new_normalize(corners[1] - corners[0])); // z lower left
        edges.push(Unit::new_normalize(corners[3] - corners[2])); // z upper left
        edges.push(Unit::new_normalize(corners[5] - corners[4])); // z lower right
        edges.push(Unit::new_normalize(corners[7] - corners[6])); // z upper right

        let mut face_normals = ArrayVec::new();
        face_normals.push(Unit::new_normalize(edges[0].cross(&edges[1]))); // Front and back sides
        face_normals.push(Unit::new_normalize(edges[0].cross(&edges[2]))); // Lower side
        face_normals.push(Unit::new_normalize(edges[0].cross(&edges[3]))); // Upper side
        face_normals.push(Unit::new_normalize(edges[1].cross(&edges[2]))); // Left side
        face_normals.push(Unit::new_normalize(edges[1].cross(&edges[4]))); // Right side

        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// This compares the From instance with another way of getting a more
    /// general `Perspective` from a symmetric Perspective defined through
    /// aspect, fovy, near and far.
    #[test]
    fn compare_perspective() {
        impl Perspective {
            pub fn new_fov(aspect: S, fovy: S, near: S, far: S) -> Self {
                assert!(
                    fovy > 0.0 && fovy < S::pi(),
                    "`fovy` must be a number between 0 and π, found: {:?}",
                    fovy
                );
                assert!(
                    aspect > 0.0,
                    "`aspect` must be a positive number, found: {:?}",
                    aspect
                );
                let angle = fovy * nalgebra::convert(0.5);
                let ymax = near * angle.tan();
                let xmax = ymax * aspect;

                Self::new(-xmax, xmax, -ymax, ymax, near, far)
            }
        }

        let persp_a: Perspective<f64> = Perspective::new_fov(1.2, 0.66, 1.0, 100.0);
        let persp_b: Perspective<f64> = nalgebra::Perspective3::new(1.2, 0.66, 1.0, 100.0).into();
        for (el_a, el_b) in persp_a.as_matrix().iter().zip(persp_b.as_matrix().iter()) {
            assert_eq!(el_a, el_b);
        }
    }
}
