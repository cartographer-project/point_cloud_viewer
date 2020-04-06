//! A cell union, re-exported from the s2 crate.
pub use s2::cellunion::CellUnion;

use crate::geometry::Aabb;
use crate::math::base::PointCulling;
use crate::math::sat::ConvexPolyhedron;
use crate::math::FromPoint3;
use nalgebra::{Point3, RealField};
use s2::{cell::Cell, cellid::CellID, region::Region};

impl<S> PointCulling<S> for CellUnion
where
    S: RealField,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from_point(p))
    }

    fn intersects_aabb(&self, aabb: &Aabb<S>) -> bool {
        let aabb_corner_cells = aabb
            .compute_corners()
            .iter()
            .map(|p| CellID::from_point(p))
            .collect();
        let mut aabb_cell_union = CellUnion(aabb_corner_cells);
        aabb_cell_union.normalize();
        let rect = aabb_cell_union.rect_bound();
        self.0
            .iter()
            .any(|cell_id| rect.intersects_cell(&Cell::from(cell_id)))
    }
}
