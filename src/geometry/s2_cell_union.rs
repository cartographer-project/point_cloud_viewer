//! A cell union, re-exported from the s2 crate.
pub use s2::cellunion::CellUnion;

use crate::geometry::Aabb;
use crate::math::base::PointCulling;
use crate::math::sat::ConvexPolyhedron;
use crate::math::FromPoint3;
use nalgebra::{Point3, RealField};
use s2::{cell::Cell, cellid::CellID, region::Region};

/// Checks for an intersection between a cell_union and a polyhedron.
///
/// This is done by checking whether an cell in the cell union intersects
/// a covering of the polyhedron with S2 cells.
pub fn cell_union_intersects_polyhedron<S>(
    cell_union: &CellUnion,
    polyhedron: &impl ConvexPolyhedron<S>,
) -> bool
where
    S: RealField,
    f64: From<S>,
{
    let polyhedron_corner_cells = polyhedron
        .compute_corners()
        .iter()
        .map(|p| CellID::from_point(p))
        .collect();
    let mut polyhedron_cell_union = CellUnion(polyhedron_corner_cells);
    polyhedron_cell_union.normalize();
    let rect = polyhedron_cell_union.rect_bound();
    cell_union
        .0
        .iter()
        .any(|cell_id| rect.intersects_cell(&Cell::from(cell_id)))
}

impl<S> PointCulling<S> for CellUnion
where
    S: RealField,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from_point(p))
    }

    fn intersects_aabb(&self, aabb: &Aabb<S>) -> bool {
        cell_union_intersects_polyhedron(self, aabb)
    }
}
