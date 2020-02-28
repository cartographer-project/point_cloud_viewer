//! Intersection checking by means of the separating axis theorem (SAT).
//!
//! Often, you want to do multiple intersection tests of the same object against
//! many other objects. To not recompute the same things – namely corners, edges and face normals –
//! each time, the [`Intersector`](struct.Intersector.html) struct can be reused between intersection tests
//! in these cases. If the edges and face normals do not change between objects, create a
//! [`CachedAxesIntersector`](struct.CachedAxesIntersector.html) and reuse it between intersection tests.
//!
//! ```
//! // For a generic intersection:
//! let frustum_intersector = frustum.intersector();
//! for obb in many_obbs {
//!   let relation = frustum_intersector.intersects(obb));
//! }
//! // For intersection with many AABBs, which can per definition only differ in
//! // their corners, not their edges and face normals (which are both the axis vecs):
//! let unit_vecs = [Vector3::x(), Vector3::y(), Vector3::z()];
//! let frustum_cached_intersector = frustum.intersector().cache_separating_axes(&unit_vecs, &unit_vecs);
//! for aabb in many_aabbs {
//!   let relation = frustum_cached_intersector.intersect(&aabb.compute_corners());
//! }
//! ```

use super::base::Relation;
use arrayvec::ArrayVec;
use nalgebra::{Point3, RealField, Unit, Vector3};
use num_traits::Bounded;

/// A trait for convex polyhedra that can perform intersection tests against other convex polyhedra.
///
/// The possible separating axes between two convex polyhedra are:
/// * The face normals of polyhedron A
/// * The face normals of polyhedron B
/// * The cross product between all edge combinations of A and B
/// Together with the corners, these are the sufficient statistics for the SAT test.
/// Hence, corners, edges and face normals must be provided by implementors of this trait.
pub trait ConvexPolyhedron<S: RealField> {
    /// For now, this is hardcoded to 8 corners and up to 6 edges/face normals.
    /// Using arrays should be cheaper than allocating a vector.
    /// We could also parametrize this trait by type-level numbers like nalgebra.
    fn compute_corners(&self) -> [Point3<S>; 8];
    /// Compute the minimum number of edges. There should be no parallel/antiparallel
    /// edges in the result.
    fn compute_edges(&self) -> ArrayVec<[Unit<Vector3<S>>; 6]>;
    /// Like edges, only unique (up to sign) face normals are needed.
    fn compute_face_normals(&self) -> ArrayVec<[Unit<Vector3<S>>; 6]>;
    /// Basically, collects the corners, edges and face normals.
    fn intersector(&self) -> Intersector<S> {
        let corners = self.compute_corners();
        let edges = self.compute_edges();
        let face_normals = self.compute_face_normals();
        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}

/// When you have one object that is intersection tested against many others,
/// compute this once (with the [`intersector`](trait.ConvexPolyhedron.html#method.intersector) method) and reuse it.
pub struct Intersector<S: RealField> {
    pub corners: [Point3<S>; 8],
    pub edges: ArrayVec<[Unit<Vector3<S>>; 6]>,
    pub face_normals: ArrayVec<[Unit<Vector3<S>>; 6]>,
}

impl<S: RealField + Bounded> Intersector<S> {
    fn separating_axes_iter<'a>(
        &'a self,
        other_edges: &'a [Unit<Vector3<S>>],
        other_face_normals: &'a [Unit<Vector3<S>>],
    ) -> impl Iterator<Item = Unit<Vector3<S>>> + 'a {
        let self_edges = self.edges.iter();
        let nested_iter = move |e1| other_face_normals.iter().map(move |e2| (e1, e2));
        let cross_products =
            self.face_normals
                .iter()
                .flat_map(nested_iter)
                .filter_map(|(e1, e2)| {
                    let cross = Unit::new_normalize(e1.cross(e2));
                    if cross[0].is_finite() && cross[1].is_finite() && cross[2].is_finite() {
                        Some(cross)
                    } else {
                        None
                    }
                });
        self_edges.chain(other_edges).cloned().chain(cross_products)
    }

    /// If you know that the edges and normals of the other object do not change, precompute
    /// the separating axes with this function. It's essentially partially applying
    /// [`intersect`](#method.intersect), leaving only the corners to be supplied.
    pub fn cache_separating_axes(
        self,
        other_edges: &[Unit<Vector3<S>>],
        other_face_normals: &[Unit<Vector3<S>>],
    ) -> CachedAxesIntersector<S> {
        let axes: Vec<_> = self
            .separating_axes_iter(other_edges, other_face_normals)
            .collect();
        // TODO(nnmm): De-duplicate axes
        CachedAxesIntersector {
            axes,
            corners: self.corners,
        }
    }

    /// Perform an intersection test. The resulting Relation expresses how the other object is spatially
    /// related to the self object – e.g. if `Relation::In` is returned, the other object is completely
    /// inside the self object.
    pub fn intersect(&self, other: &Self) -> Relation {
        sat(
            self.separating_axes_iter(&other.edges, &other.face_normals),
            &self.corners,
            &other.corners,
        )
    }
}

/// Stores pre-computed separating axes for intersection tests.
pub struct CachedAxesIntersector<S: RealField> {
    pub axes: Vec<Unit<Vector3<S>>>,
    pub corners: [Point3<S>; 8],
}

impl<S: RealField + Bounded> CachedAxesIntersector<S> {
    /// Perform an intersection test using the cached axes and the specified corner points. The resulting
    /// Relation expresses how the other object is spatially related to the self object – e.g. if
    /// `Relation::In` is returned, the other object is completely inside the self object.
    pub fn intersect(&self, corners: &[Point3<S>]) -> Relation {
        sat(self.axes.iter().cloned(), &self.corners, corners)
    }
}

/// See https://www.gamedev.net/forums/topic/694911-separating-axis-theorem-3d-polygons/ for more detail
fn sat<S, I>(separating_axes: I, corners_a: &[Point3<S>], corners_b: &[Point3<S>]) -> Relation
where
    S: RealField + Bounded,
    I: IntoIterator<Item = Unit<Vector3<S>>>,
{
    for sep_axis in separating_axes {
        // Project corners of A onto that axis
        let mut a_min_proj: S = Bounded::max_value();
        let mut a_max_proj: S = Bounded::min_value();
        for corner in corners_a {
            let corner_proj = corner.coords.dot(&sep_axis);
            a_min_proj = a_min_proj.min(corner_proj);
            a_max_proj = a_max_proj.max(corner_proj);
        }
        // Project corners of B onto that axis
        let mut b_min_proj: S = Bounded::max_value();
        let mut b_max_proj: S = Bounded::min_value();
        for corner in corners_b {
            let corner_proj = corner.coords.dot(&sep_axis);
            b_min_proj = b_min_proj.min(corner_proj);
            b_max_proj = b_max_proj.max(corner_proj);
        }
        if b_min_proj > a_max_proj || b_max_proj < a_min_proj {
            return Relation::Out;
        }
    }
    Relation::Cross
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::math::base::Relation;
    use arrayvec::ArrayVec;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_cube_with_cube() {
        let unit_vectors: ArrayVec<_> =
            ArrayVec::from([Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()])
                .into_iter()
                .collect();
        #[cfg_attr(rustfmt, rustfmt_skip)]
        let cube_isec_1 = Intersector {
            corners: [
                Point3::new(-1.0, -1.0, -1.0),
                Point3::new(-1.0, -1.0,  1.0),
                Point3::new(-1.0,  1.0, -1.0),
                Point3::new(-1.0,  1.0,  1.0),
                Point3::new( 1.0, -1.0, -1.0),
                Point3::new( 1.0, -1.0,  1.0),
                Point3::new( 1.0,  1.0, -1.0),
                Point3::new( 1.0,  1.0,  1.0),
            ],
            edges: unit_vectors.clone(),
            face_normals: unit_vectors.clone(),
        };
        #[cfg_attr(rustfmt, rustfmt_skip)]
        let cube_isec_2 = Intersector {
            corners: [
                Point3::new(-0.5, -0.5, -0.5),
                Point3::new(-0.5, -0.5,  1.5),
                Point3::new(-0.5,  1.5, -0.5),
                Point3::new(-0.5,  1.5,  1.5),
                Point3::new( 1.5, -0.5, -0.5),
                Point3::new( 1.5, -0.5,  1.5),
                Point3::new( 1.5,  1.5, -0.5),
                Point3::new( 1.5,  1.5,  1.5),
            ],
            edges: unit_vectors.clone(),
            face_normals: unit_vectors.clone(),
        };
        #[cfg_attr(rustfmt, rustfmt_skip)]
        let cube_isec_3 = Intersector {
            corners: [
                Point3::new(-0.9, -0.9, -0.9),
                Point3::new(-0.9, -0.9, -0.7),
                Point3::new(-0.9, -0.7, -0.9),
                Point3::new(-0.9, -0.7, -0.7),
                Point3::new(-0.7, -0.9, -0.9),
                Point3::new(-0.7, -0.9, -0.7),
                Point3::new(-0.7, -0.7, -0.9),
                Point3::new(-0.7, -0.7, -0.7),
            ],
            edges: unit_vectors.clone(),
            face_normals: unit_vectors.clone(),
        };

        assert_eq!(cube_isec_1.intersect(&cube_isec_2), Relation::Cross);
        assert_eq!(cube_isec_2.intersect(&cube_isec_3), Relation::Out);
        assert_eq!(cube_isec_1.intersect(&cube_isec_3), Relation::In);
        assert_eq!(cube_isec_3.intersect(&cube_isec_1), Relation::Cross);
    }
}
