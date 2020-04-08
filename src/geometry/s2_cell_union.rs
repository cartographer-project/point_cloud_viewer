//! A cell union, re-exported from the s2 crate.
pub use s2::cellunion::CellUnion;

use crate::geometry::Aabb;
use crate::math::base::{HasAabbIntersector, IntersectAabb, PointCulling};
use crate::math::sat::ConvexPolyhedron;
use crate::math::FromPoint3;
use nalgebra::{Point3, RealField};
use s2::{cell::Cell, cellid::CellID, region::Region};

/// Checks for an intersection between a list of cells and a polyhedron.
///
/// This is done by checking whether an cell in the cell union intersects
/// a covering of the polyhedron with S2 cells.
pub fn cells_intersecting_polyhedron<S>(
    cells: &[Cell],
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
    cells.iter().any(|cell| rect.intersects_cell(cell))
}

impl<S> PointCulling<S> for CellUnion
where
    S: RealField,
    f64: From<S>,
{
    fn contains(&self, p: &Point3<S>) -> bool {
        self.contains_cellid(&CellID::from_point(p))
    }
}

impl<S> IntersectAabb<S> for Vec<Cell>
where
    f64: From<S>,
    S: RealField,
{
    fn intersect_aabb(&self, aabb: &Aabb<S>) -> bool {
        cells_intersecting_polyhedron(self, aabb)
    }
}

impl<'a, S: RealField> HasAabbIntersector<'a, S> for CellUnion
where
    f64: From<S>,
{
    type Intersector = Vec<Cell>;
    fn aabb_intersector(&'a self) -> Self::Intersector {
        self.0.iter().map(Cell::from).collect()
    }
}
