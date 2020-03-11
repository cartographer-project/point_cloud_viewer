//! An asymmetric frustum with an arbitrary 3D pose.

use crate::geometry::Aabb;
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Intersector, Relation};
use crate::math::PointCulling;
use arrayvec::ArrayVec;
use nalgebra::{Isometry3, Matrix4, Point3, RealField, Unit, Vector3};
use num_traits::Bounded;
use serde::{Deserialize, Serialize};

/// A perspective projection matrix analogous to cgmath::Perspective.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Perspective<S: RealField> {
    matrix: Matrix4<S>,
}

impl<S: RealField> Perspective<S> {
    /// Left, right, bottom, and top are in radians.
    pub fn new(left: S, right: S, bottom: S, top: S, near: S, far: S) -> Self {
        assert!(
            left < right,
            "`left` cannot be greater than `right`, found: left: {:?} right: {:?}",
            left,
            right
        );
        assert!(
            bottom < top,
            "`bottom` cannot be greater than `top`, found: bottom: {:?} top: {:?}",
            bottom,
            top
        );
        assert!(
            near > S::zero() && near < far,
            "`near` must be greater than 0 and cannot be greater than `far`, found: near: {:?} far: {:?}",
            near,
            far
        );

        let two: S = nalgebra::convert(2.0);

        let r0c0 = (two * near) / (right - left);
        let r0c2 = (right + left) / (right - left);

        let r1c1 = (two * near) / (top - bottom);
        let r1c2 = (top + bottom) / (top - bottom);

        let r2c2 = -(far + near) / (far - near);
        let r2c3 = -(two * far * near) / (far - near);

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            r0c0,      S::zero(), r0c2,      S::zero(),
            S::zero(), r1c1,      r1c2,      S::zero(),
            S::zero(), S::zero(), r2c2,      r2c3,
            S::zero(), S::zero(), -S::one(), S::zero(),
        );
        Self { matrix }
    }

    // This emulates cgmath::PerspectiveFov, which is more restricted
    // and corresponds to nalgebra::Perspective.
    pub fn new_fov(fovy: S, aspect: S, near: S, far: S) -> Self {
        assert!(
            fovy > S::zero() && fovy < S::pi(),
            "`fovy` must be a number between 0 and π, found: {:?}",
            fovy
        );
        assert!(
            aspect > S::zero(),
            "`aspect` must be a positive number, found: {:?}",
            aspect
        );
        let angle = nalgebra::convert::<f64, S>(0.5) * fovy;
        let ymax = near * angle.tan();
        let xmax = ymax * aspect;

        Self::new(-xmax, xmax, -ymax, ymax, near, far)
    }

    pub fn as_matrix(&self) -> &Matrix4<S> {
        &self.matrix
    }

    pub fn inverse(&self) -> Matrix4<S> {
        let r0c0 = self.matrix[(0, 0)].recip();
        let r0c3 = self.matrix[(0, 2)] / self.matrix[(0, 0)];

        let r1c1 = self.matrix[(1, 1)].recip();
        let r1c3 = self.matrix[(1, 2)] / self.matrix[(1, 1)];

        let r3c2 = self.matrix[(2, 3)].recip();
        let r3c3 = self.matrix[(2, 2)] / self.matrix[(2, 3)];

        #[rustfmt::skip]
        let matrix = Matrix4::new(
            r0c0,      S::zero(), S::zero(), r0c3,
            S::zero(), r1c1,      S::zero(), r1c3,
            S::zero(), S::zero(), S::zero(), -S::one(),
            S::zero(), S::zero(), r3c2,      r3c3,
        );
        matrix
    }
}

/// A frustum is defined in eye coordinates, where x points right, y points up,
/// and z points against the viewing direction. This is not how e.g. OpenCV
/// defines a camera coordinate system. To get from OpenCV camera coordinates
/// to eye coordinates, you need to rotate 180 deg around the x axis before
/// creating the perspective projection, see also the frustum unit test below.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Frustum<S: RealField> {
    world_from_clip: Matrix4<S>,
    clip_from_world: Matrix4<S>,
}

/// TODO(nnmm): Remove
pub struct CachedAxesFrustum<S: RealField> {
    frustum: Frustum<S>,
    separating_axes: CachedAxesIntersector<S>,
}

impl<S: RealField + Bounded> CachedAxesFrustum<S> {
    pub fn new(frustum: Frustum<S>) -> Self {
        let unit_axes = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
        let separating_axes = frustum
            .intersector()
            .cache_separating_axes(&unit_axes, &unit_axes);
        Self {
            frustum,
            separating_axes,
        }
    }
}

impl<S: RealField> Frustum<S> {
    pub fn new(world_from_eye: Isometry3<S>, clip_from_eye: Perspective<S>) -> Self {
        let clip_from_world = clip_from_eye.as_matrix() * world_from_eye.inverse().to_homogeneous();
        let world_from_clip = world_from_eye.to_homogeneous() * clip_from_eye.inverse();
        Frustum {
            world_from_clip,
            clip_from_world,
        }
    }

    /// Fails if the matrix is not invertible.
    pub fn from_matrix4(clip_from_world: Matrix4<S>) -> Option<Self> {
        let world_from_clip = clip_from_world.try_inverse()?;
        Some(Self {
            world_from_clip,
            clip_from_world,
        })
    }
}

impl<S> PointCulling<S> for CachedAxesFrustum<S>
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool {
        let p_clip = self.frustum.clip_from_world.transform_point(point);
        p_clip.coords.min() > nalgebra::convert(-1.0)
            && p_clip.coords.max() < nalgebra::convert(1.0)
    }

    fn intersects_aabb(&self, aabb: &Aabb<S>) -> bool {
        self.separating_axes.intersect(&aabb.compute_corners()) != Relation::Out
    }
}

impl<S> ConvexPolyhedron<S> for Frustum<S>
where
    S: RealField,
{
    #[rustfmt::skip]
    fn compute_corners(&self) -> [Point3<S>; 8] {
        let corner_from = |x, y, z| self.world_from_clip.transform_point(&Point3::new(x, y, z));
        [
            corner_from(-S::one(), -S::one(), -S::one()),
            corner_from(-S::one(), -S::one(),  S::one()),
            corner_from(-S::one(),  S::one(), -S::one()),
            corner_from(-S::one(),  S::one(),  S::one()),
            corner_from( S::one(), -S::one(), -S::one()),
            corner_from( S::one(), -S::one(),  S::one()),
            corner_from( S::one(),  S::one(), -S::one()),
            corner_from( S::one(),  S::one(),  S::one()),
        ]
    }

    fn intersector(&self) -> Intersector<S> {
        let corners = self.compute_corners();

        let edges = ArrayVec::from([
            Unit::new_normalize(corners[4] - corners[0]), // x
            Unit::new_normalize(corners[2] - corners[0]), // y
            Unit::new_normalize(corners[1] - corners[0]), // z lower left
            Unit::new_normalize(corners[3] - corners[2]), // z upper left
            Unit::new_normalize(corners[5] - corners[4]), // z lower right
            Unit::new_normalize(corners[7] - corners[6]), // z upper right
        ]);

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
