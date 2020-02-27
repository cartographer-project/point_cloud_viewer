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

use super::aabb::AABB;
use super::base::Relation;
use arrayvec::ArrayVec;
use nalgebra::{Point3, RealField, Scalar, Vector3};
use num_traits::{Bounded, Zero};

/// A trait for convex polyhedra that can perform intersection tests against other convex polyhedra.
///
/// The possible separating axes between two convex polyhedra are:
/// * The face normals of polyhedron A
/// * The face normals of polyhedron B
/// * The cross product between all edge combinations of A and B
/// Together with the corners, these are the sufficient statistics for the SAT test.
/// Hence, corners, edges and face normals must be provided by implementors of this trait.
pub trait ConvexPolyhedron<S: Scalar> {
    /// For now, this is hardcoded to 8 corners and up to 6 edges/face normals.
    /// Using arrays should be cheaper than allocating a vector.
    /// We could also parametrize this trait by type-level numbers like nalgebra.
    fn compute_corners(&self) -> [Point3<S>; 8];
    /// Compute the minimum number of edges. There should be no parallel/antiparallel
    /// edges in the result.
    fn compute_edges(&self) -> [Option<Vector3<S>>; 6];
    /// Like edges, only unique (up to sign) face normals are needed.
    fn compute_face_normals(&self) -> [Option<Vector3<S>>; 6];
    fn intersector(&self) -> Intersector<S> {
        let corners = self.compute_corners();
        let maybe_edges = self.compute_edges();
        let maybe_face_normals = self.compute_face_normals();
        let edges = ArrayVec::from(maybe_edges)
            .into_iter()
            .filter_map(|x| x)
            .collect();
        let face_normals = ArrayVec::from(maybe_face_normals)
            .into_iter()
            .filter_map(|x| x)
            .collect();
        Intersector {
            corners,
            edges,
            face_normals,
        }
    }
}

/// When you have one object that is intersection tested against many others,
/// compute this once (with the `From` instance) and reuse it.
pub struct Intersector<S: Scalar> {
    corners: [Point3<S>; 8],
    edges: ArrayVec<[Vector3<S>; 6]>,
    face_normals: ArrayVec<[Vector3<S>; 6]>,
}

impl<S: Scalar + RealField + Bounded> Intersector<S> {
    fn separating_axes_iter<'a>(
        &'a self,
        other_edges: &'a [Vector3<S>],
        other_face_normals: &'a [Vector3<S>],
    ) -> impl Iterator<Item = Vector3<S>> + 'a {
        // TODO: change to reference
        let self_edges = self.edges.iter();
        let nested_iter = move |e1| other_face_normals.iter().map(move |e2| (e1, e2));
        let cross_products = self
            .face_normals
            .iter()
            .flat_map(nested_iter)
            .map(|(e1, e2): (&Vector3<S>, &Vector3<S>)| e1.cross(e2).normalize());
        self_edges.chain(other_edges).cloned().chain(cross_products)
    }

    /// If you know that the edges and normals of the other object do not change, precompute
    /// the separating axes with this function. It's essentially partially applying
    /// [`intersect`](#method.intersect), leaving only the corners to be supplied.
    pub fn cache_separating_axes(
        self,
        other_edges: &[Vector3<S>],
        other_face_normals: &[Vector3<S>],
    ) -> CachedAxesIntersector<S> {
        let axes: Vec<_> = self
            .separating_axes_iter(other_edges, other_face_normals)
            .collect();
        CachedAxesIntersector {
            axes,
            corners: self.corners,
        }
    }

    pub fn intersect(&self, other: Self) -> Relation {
        sat(
            self.separating_axes_iter(&other.edges, &other.face_normals),
            &self.corners,
            &other.corners,
        )
    }
}

/// The possible separating axes between two convex polyhedra are:
/// * The face normals of polyhedron A
/// * The face normals of polyhedron B
/// * The cross product between all edge combinations of A and B
pub struct CachedAxesIntersector<S: Scalar> {
    axes: Vec<Vector3<S>>,
    corners: [Point3<S>; 8],
}

impl<S: Scalar + RealField + Bounded> CachedAxesIntersector<S> {
    pub fn intersect(&self, corners: &[Point3<S>]) -> Relation {
        sat(self.axes.iter().cloned(), &self.corners, corners)
    }
}

impl<'a, S> IntoIterator for &'a CachedAxesIntersector<S>
where
    S: Scalar,
{
    type Item = Vector3<S>;
    type IntoIter = std::iter::Copied<std::slice::Iter<'a, Vector3<S>>>;

    fn into_iter(self) -> Self::IntoIter {
        self.axes.iter().copied()
    }
}

fn sat<S, I>(separating_axes: I, corners_a: &[Point3<S>], corners_b: &[Point3<S>]) -> Relation
where
    S: Scalar + RealField + Bounded,
    I: IntoIterator<Item = Vector3<S>>,
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

/// See https://www.gamedev.net/forums/topic/694911-separating-axis-theorem-3d-polygons/ for more detail
pub fn intersects_aabb<S: RealField + Bounded>(
    corners: &[Point3<S>],
    separating_axes: &[Vector3<S>],
    aabb: &AABB<S>,
) -> Relation {
    // SAT algorithm
    // https://gamedev.stackexchange.com/questions/44500/how-many-and-which-axes-to-use-for-3d-obb-collision-with-sat
    for sep_axis in separating_axes.iter() {
        // Project the cube and the box onto that axis
        let mut cube_min_proj: S = Bounded::max_value();
        let mut cube_max_proj: S = Bounded::min_value();
        for corner in aabb.corners().iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            cube_min_proj = cube_min_proj.min(corner_proj);
            cube_max_proj = cube_max_proj.max(corner_proj);
        }
        // Project corners of the box onto that axis
        let mut box_min_proj: S = Bounded::max_value();
        let mut box_max_proj: S = Bounded::min_value();
        for corner in corners.iter() {
            let corner_proj = corner.coords.dot(sep_axis);
            box_min_proj = box_min_proj.min(corner_proj);
            box_max_proj = box_max_proj.max(corner_proj);
        }
        if box_min_proj > cube_max_proj || box_max_proj < cube_min_proj {
            return Relation::Out;
        }
    }
    Relation::Cross
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Dummy {}

    impl ConvexPolyhedron<f64> for Dummy {
        fn compute_corners(&self) -> [Point3<f64>; 8] {
            [Point3::origin(); 8]
        }
        fn compute_edges(&self) -> [Option<Vector3<f64>>; 6] {
            [Vector3::x(), Vector3::y(), Vector3::z(), None, None, None]
        }
    }

    #[test]
    fn test_corners_and_edges() {
        let cae = CornersAndEdges::from(Dummy {});
        assert_eq!(cae.num_edges, 3);
        assert_eq!(cae.edges[0], Vector3::x());
        assert_eq!(cae.edges[1], Vector3::y());
        assert_eq!(cae.edges[2], Vector3::z());
    }
}
