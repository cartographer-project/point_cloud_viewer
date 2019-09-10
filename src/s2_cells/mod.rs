use crate::batch_iterator::{PointCloud, PointLocation, PointQuery};
pub use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::math::{Isometry3, Obb};
use crate::octree::FilteredPointsIterator;
use crate::read_write::{Encoding, NodeIterator};
use crate::AttributeDataType;
use cgmath::{Point3, Transform, Vector4};
use fnv::FnvHashMap;
use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::point::Point as S2Point;
use s2::region::Region;
use std::borrow::Cow;
use std::collections::HashMap;

pub struct S2Cells {
    data_provider: Box<dyn DataProvider>,
    cells: FnvHashMap<CellID, Cell>,
    meta: S2Meta,
}

pub struct S2Meta {
    pub nodes: FnvHashMap<CellID, S2CellMeta>,
    pub attributes: HashMap<String, AttributeDataType>,
}

#[derive(Copy, Clone)]
pub struct S2CellMeta {
    pub num_points: u64,
}

/// Just a wrapper that implements Display
/// I already implemented that in open source
#[derive(Copy, Clone)]
pub struct S2CellId(pub CellID);

impl std::fmt::Display for S2CellId {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}", self.0.to_token())
    }
}

impl PointCloud for S2Cells {
    type Id = S2CellId;
    type PointsIter = FilteredPointsIterator;

    fn nodes_in_location(&self, query: &PointQuery) -> Vec<Self::Id> {
        match &query.location {
            PointLocation::AllPoints() => self.cells.keys().cloned().map(S2CellId).collect(),
            PointLocation::Aabb(aabb) => cells_in_obb(
                self.cells.values(),
                &Obb::from(aabb),
                query.global_from_local.as_ref(),
            ),
            PointLocation::Obb(obb) => {
                cells_in_obb(self.cells.values(), &obb, query.global_from_local.as_ref())
            }
            PointLocation::Frustum(mat) => {
                let world_from_clip = mat.inverse_transform().unwrap();
                let points = CLIP_CUBE_CORNERS
                    .iter()
                    .map(|p| Point3::from_homogeneous(world_from_clip * p));
                cells_in_convex_hull(self.cells.values(), points)
            }
            PointLocation::OrientedBeam(_) => unimplemented!(),
        }
    }

    fn encoding_for_node(&self, _: Self::Id) -> Encoding {
        Encoding::Plain
    }

    fn points_in_node<'a>(
        &'a self,
        query: &PointQuery,
        node_id: Self::Id,
    ) -> Result<Self::PointsIter> {
        let culling = query.get_point_culling();
        let num_points = self.meta.nodes[&node_id.0].num_points as usize;
        let node_iterator = NodeIterator::from_data_provider(
            &*self.data_provider,
            self.encoding_for_node(node_id),
            &node_id,
            num_points,
        )?;
        Ok(FilteredPointsIterator {
            culling,
            node_iterator,
        })
    }
}

/// Wrapper arround cells_in_convex_hull for Obbs
fn cells_in_obb<'a, IterCell>(
    cells: IterCell,
    obb: &Obb<f64>,
    global_from_local: Option<&Isometry3<f64>>,
) -> Vec<S2CellId>
where
    IterCell: Iterator<Item = &'a Cell>,
{
    let obb = match global_from_local {
        Some(isometry) => Cow::Owned(Obb::new(
            isometry * &obb.isometry_inv.inverse(),
            obb.half_extent,
        )),
        None => Cow::Borrowed(obb),
    };
    let points = obb.corners;
    cells_in_convex_hull(cells, points.iter().cloned())
}

/// Returns all cells that intersect the convex hull of the given points
fn cells_in_convex_hull<'a, IterCell, IterPoint>(
    cells: IterCell,
    points: IterPoint,
) -> Vec<S2CellId>
where
    IterCell: Iterator<Item = &'a Cell>,
    IterPoint: IntoIterator<Item = Point3<f64>>,
{
    // We could choose either a covering rect or a covering cap as a convex hull
    let point_cells = points
        .into_iter()
        .map(|p| CellID::from(S2Point::from_coords(p.x, p.y, p.z)))
        .collect();
    let mut cell_union = CellUnion(point_cells);
    cell_union.normalize();
    let convex_hull = cell_union.rect_bound();
    cells
        .filter(|cell| convex_hull.intersects_cell(cell))
        .map(|cell| S2CellId(cell.id))
        .collect()
}

/// This is projected back with the inverse frustum matrix
/// to find the corners of the frustum
const CLIP_CUBE_CORNERS: [Vector4<f64>; 8] = [
    Vector4 {
        x: -1.0,
        y: -1.0,
        z: -1.0,
        w: 1.0,
    },
    Vector4 {
        x: -1.0,
        y: -1.0,
        z: 1.0,
        w: 1.0,
    },
    Vector4 {
        x: -1.0,
        y: 1.0,
        z: -1.0,
        w: 1.0,
    },
    Vector4 {
        x: -1.0,
        y: 1.0,
        z: 1.0,
        w: 1.0,
    },
    Vector4 {
        x: 1.0,
        y: -1.0,
        z: -1.0,
        w: 1.0,
    },
    Vector4 {
        x: 1.0,
        y: -1.0,
        z: 1.0,
        w: 1.0,
    },
    Vector4 {
        x: 1.0,
        y: 1.0,
        z: -1.0,
        w: 1.0,
    },
    Vector4 {
        x: 1.0,
        y: 1.0,
        z: 1.0,
        w: 1.0,
    },
];
