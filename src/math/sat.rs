//! Intersection checking by means of the separating axis theorem (SAT).
//!
//! Often, you want to do multiple intersection tests of the same object against
//! many other objects. To not recompute the same things – namely corners, edges and face normals –
//! each time, the [`Intersector`](struct.Intersector.html) struct can be reused between intersection tests
//! in these cases. If the edges and face normals do not change between objects, create a
//! [`CachedAxesIntersector`](struct.CachedAxesIntersector.html) and reuse it between intersection tests.
//!
//! ```no_run
//! use nalgebra::Vector3;
//! use point_viewer::math::sat::ConvexPolyhedron;
//! use point_viewer::geometry::{Aabb, Obb, Frustum};
//! // Use your imagination here
//! let many_obbs: Vec<Obb<f64>> = unimplemented!();
//! let many_aabbs: Vec<Aabb<f64>> = unimplemented!();
//! let frustum: Frustum<f64> = unimplemented!();
//! // For a generic intersection, you can reuse the intersector:
//! let frustum_intersector = frustum.intersector();
//! for obb in many_obbs {
//!   let relation = frustum_intersector.intersect(&obb.intersector());
//! }
//! // For intersection with many Aabbs, which can per definition only differ in
//! // their corners, not their edges and face normals (which are both the axis vecs),
//! // you can reuse not only the intersector, but also the separating axes:
//! let unit_vecs = [Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()];
//! let frustum_cached_intersector = frustum.intersector().cache_separating_axes(&unit_vecs, &unit_vecs);
//! for aabb in many_aabbs {
//!   let relation = frustum_cached_intersector.intersect(&aabb.compute_corners());
//! }
//! ```

use arrayvec::ArrayVec;
use nalgebra::{Point3, RealField, Unit, Vector3};
use num_traits::Bounded;

/// Spatial relation between two objects.
/// Modeled after the collision crate.
#[derive(Copy, Clone, Debug, Eq, Hash, Ord, PartialOrd, PartialEq)]
pub enum Relation {
    /// Completely inside.
    In,
    /// Crosses the boundary.
    Cross,
    /// Completely outside.
    Out,
}

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
    /// An intersector contains corners, edges and face normals. Edges and face normals should be
    /// unique to get the best performance (antiparallel vectors are the same for this purpose).
    fn intersector(&self) -> Intersector<S>;
}

/// When you have one object that is intersection tested against many others,
/// compute this once (with the [`intersector`](trait.ConvexPolyhedron.html#method.intersector) method) and reuse it.
pub struct Intersector<S: RealField> {
    /// The corners of the polyhedron.
    pub corners: [Point3<S>; 8],
    /// The unique edges of the polyhedron.
    /// This is hardcoded to 6 for now because we don't need more. Increase as needed.
    pub edges: ArrayVec<[Unit<Vector3<S>>; 6]>,
    /// The unique face normals of the polyhedron.
    pub face_normals: ArrayVec<[Unit<Vector3<S>>; 6]>,
}

impl<S: RealField + Bounded> Intersector<S> {
    /// An iterator over separating axes – if we're only going to use the separating axes once,
    /// we don't want to require an allocation.
    fn separating_axes_iter<'a>(
        &'a self,
        other_edges: &'a [Unit<Vector3<S>>],
        other_face_normals: &'a [Unit<Vector3<S>>],
    ) -> impl Iterator<Item = Unit<Vector3<S>>> + 'a {
        let self_face_normals = self.face_normals.iter();
        let nested_iter = move |e1| other_edges.iter().map(move |e2| (e1, e2));
        let edge_cross_products = self
            .edges
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
        self_face_normals
            .chain(other_face_normals)
            .cloned()
            .chain(edge_cross_products)
    }

    /// If you know that the edges and normals of the other object do not change, precompute
    /// the separating axes with this function. It's essentially partially applying
    /// [`intersect`](#method.intersect), leaving only the corners to be supplied.
    ///
    /// It applies deduplication of separating axes to speed up queries made using those axes.
    /// That deduplication is currently O(n^2).
    pub fn cache_separating_axes(
        self,
        other_edges: &[Unit<Vector3<S>>],
        other_face_normals: &[Unit<Vector3<S>>],
    ) -> CachedAxesIntersector<S> {
        let all_axes: Vec<_> = self
            .separating_axes_iter(other_edges, other_face_normals)
            .collect();
        let mut dedup_axes = Vec::new();
        for ax1 in all_axes {
            let is_dupe = dedup_axes.iter().any(|ax2: &Unit<Vector3<S>>| {
                let d1 = (ax1.as_ref() - ax2.as_ref()).norm_squared();
                let d2 = (ax1.as_ref() + ax2.as_ref()).norm_squared();
                d1.min(d2) < S::default_epsilon()
            });
            if !is_dupe {
                dedup_axes.push(ax1);
            }
        }
        CachedAxesIntersector {
            axes: dedup_axes,
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
/// Return `Relation::In` if B is contained in A
pub fn sat<S, I>(separating_axes: I, corners_a: &[Point3<S>], corners_b: &[Point3<S>]) -> Relation
where
    S: RealField + Bounded,
    I: IntoIterator<Item = Unit<Vector3<S>>>,
{
    let mut rel = Relation::In;
    for sep_axis in separating_axes {
        // Project corners of A onto that axis
        let (a_min_proj, a_max_proj) = project_on_axis(corners_a, sep_axis);
        // Project corners of B onto that axis
        let (b_min_proj, b_max_proj) = project_on_axis(corners_b, sep_axis);

        if b_min_proj > a_max_proj || b_max_proj < a_min_proj {
            return Relation::Out;
        }
        // If B is not inside A wrt that axis, the only choice is between Cross and Out.
        if a_min_proj > b_min_proj || b_max_proj > a_max_proj {
            rel = Relation::Cross;
        }
    }
    rel
}

fn project_on_axis<S>(corners: &[Point3<S>], sep_axis: Unit<Vector3<S>>) -> (S, S)
where
    S: RealField + Bounded,
{
    let mut min_proj: S = Bounded::max_value();
    let mut max_proj: S = Bounded::min_value();
    for corner in corners {
        let corner_proj = corner.coords.dot(&sep_axis);
        min_proj = min_proj.min(corner_proj);
        max_proj = max_proj.max(corner_proj);
    }
    (min_proj, max_proj)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrayvec::ArrayVec;
    use nalgebra::{Point3, Vector3};

    #[test]
    fn test_cube_with_cube() {
        let unit_vectors: ArrayVec<_> =
            ArrayVec::from([Vector3::x_axis(), Vector3::y_axis(), Vector3::z_axis()])
                .into_iter()
                .collect();
        #[rustfmt::skip]
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
        #[rustfmt::skip]
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
        #[rustfmt::skip]
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
