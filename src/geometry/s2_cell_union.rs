//! A cell union, re-exported from the s2 crate.
pub use s2::cellunion::CellUnion;

use crate::geometry::Aabb;
use crate::math::base::{HasAabbIntersector, IntersectAabb, PointCulling};
use crate::math::sat::ConvexPolyhedron;
use crate::math::FromPoint3;
use nalgebra::Point3;
use s2::{cell::Cell, cellid::CellID, region::Region};

/// Checks for an intersection between a list of cells and a polyhedron.
///
/// This is done by checking whether any cell in the list intersects
/// a covering of the polyhedron with S2 cells.
pub fn cells_intersecting_polyhedron(cells: &[Cell], polyhedron: &impl ConvexPolyhedron) -> bool {
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

impl PointCulling for CellUnion {
    fn contains(&self, p: &Point3<f64>) -> bool {
        self.contains_cellid(&CellID::from_point(p))
    }
}

impl IntersectAabb for Vec<Cell> {
    fn intersect_aabb(&self, aabb: &Aabb) -> bool {
        cells_intersecting_polyhedron(self, aabb)
    }
}

impl<'a> HasAabbIntersector<'a> for CellUnion {
    type Intersector = Vec<Cell>;
    fn aabb_intersector(&'a self) -> Self::Intersector {
        self.0.iter().map(Cell::from).collect()
    }
}
