use crate::geometry::Aabb;
use crate::math::sat::{CachedAxesIntersector, ConvexPolyhedron, Relation};
use nalgebra::{Point3, RealField};
use num_traits::Bounded;

pub trait PointCulling<S>
where
    S: RealField,
{
    fn contains(&self, point: &Point3<S>) -> bool;
}

/// Something that can perform an intersection test with an AABB.
pub trait IntersectAabb<S: RealField> {
    // TODO(nnmm): return Relation
    fn intersect_aabb(&self, aabb: &Aabb<S>) -> bool;
}

/// We use this trait to allow an indirection: The geometry itself does not need to be able to
/// efficiently do intersection tests with AABBs, because the geometry (e.g. an OBB) should not need
/// to store data to support intersection tests (like separating axes).
/// The trait has a lifetime to support the intersector borrowing from self, or being identical to
/// &Self.
///
/// Having `impl<'a, T: IntersectAabb<S>, S> HasAabbIntersector<'a, S> for T` and
/// `impl<'a, S, T: ConvexPolyhedron<S>> HasAabbIntersector<'a, S> for T` would be nice, but results
/// in conflicts.
pub trait HasAabbIntersector<'a, S: RealField> {
    type Intersector: IntersectAabb<S> + 'a;
    fn aabb_intersector(&'a self) -> Self::Intersector;
}

impl<S: RealField + Bounded> IntersectAabb<S> for CachedAxesIntersector<S> {
    fn intersect_aabb(&self, aabb: &Aabb<S>) -> bool {
        self.intersect(&aabb.compute_corners()) != Relation::Out
    }
}
