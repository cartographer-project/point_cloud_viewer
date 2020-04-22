use crate::geometry::Aabb;
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Relation};
use nalgebra::Point3;

pub trait PointCulling {
    fn contains(&self, point: &Point3<f64>) -> bool;
}

/// Something that can perform an intersection test with an AABB.
pub trait IntersectAabb {
    // TODO(nnmm): return Relation
    fn intersect_aabb(&self, aabb: &Aabb) -> bool;
}

/// We use this trait to allow an indirection: The geometry itself does not need to be able to
/// efficiently do intersection tests with AABBs, because the geometry (e.g. an OBB) should not need
/// to store data to support intersection tests (like separating axes).
/// The trait has a lifetime to support the intersector borrowing from self, or being identical to
/// &Self.
///
/// Having `impl<'a, T: IntersectAabb, S> HasAabbIntersector<'a, S> for T` and
/// `impl<'a, S, T: ConvexPolyhedron> HasAabbIntersector<'a, S> for T` would be nice, but results
/// in conflicts.
pub trait HasAabbIntersector<'a> {
    type Intersector: IntersectAabb + 'a;
    fn aabb_intersector(&'a self) -> Self::Intersector;
}

impl IntersectAabb for CachedAxesIntersector<f64> {
    fn intersect_aabb(&self, aabb: &Aabb) -> bool {
        self.intersect(&aabb.compute_corners()) != Relation::Out
    }
}

impl<'a, T: ConvexPolyhedron<f64>> HasAabbIntersector<'a> for T {
    type Intersector = CachedAxesIntersector<f64>;
    fn aabb_intersector(&'a self) -> Self::Intersector {
        self.intersector().cache_separating_axes_for_aabb()
    }
}

// /// Use this macro as a crutch for the missing
// /// `impl<'a, S, T: ConvexPolyhedron> HasAabbIntersector<'a, S> for T`.
// macro_rules! has_aabb_intersector_for_convex_polyhedron {
//     ($type:ty) => {
//         impl<'a> HasAabbIntersector<'a, f64> for $type {
//             type Intersector = CachedAxesIntersector<f64>;
//             fn aabb_intersector(&'a self) -> Self::Intersector {
//                 self.intersector().cache_separating_axes_for_aabb()
//             }
//         }
//     };
// }
