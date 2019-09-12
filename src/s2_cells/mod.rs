use crate::batch_iterator::{PointCloud, PointLocation, PointQuery};
use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::math::{Isometry3, Obb};
use crate::octree::FilteredPointsIterator;
use crate::proto;
use crate::read_write::{Encoding, NodeIterator};
use crate::{AttributeDataType, CURRENT_VERSION};
use cgmath::{Point3, Transform, Vector4};
use fnv::FnvHashMap;
use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::point::Point as S2Point;
use s2::region::Region;
use std::borrow::Cow;
use std::collections::HashMap;
use std::convert::TryInto;

pub struct S2Cells {
    data_provider: Box<dyn DataProvider>,
    cells: FnvHashMap<CellID, Cell>,
    meta: S2Meta,
}

#[derive(Copy, Clone)]
pub struct S2CellMeta {
    pub num_points: u64,
}

pub struct S2Meta {
    pub cells: FnvHashMap<CellID, S2CellMeta>,
    pub attributes: HashMap<String, AttributeDataType>,
}

impl S2Meta {
    pub fn to_proto(&self) -> proto::Meta {
        let cell_protos = self
            .cells
            .iter()
            .map(|(cell_id, cell_meta)| {
                let mut meta = proto::S2Cell::new();
                meta.set_id(cell_id.0);
                meta.set_num_points(cell_meta.num_points.try_into().unwrap());
                meta
            })
            .collect();
        let mut meta = proto::Meta::new();
        meta.set_version(CURRENT_VERSION);
        let mut s2_meta = proto::S2Meta::new();
        s2_meta.set_cells(::protobuf::RepeatedField::<proto::S2Cell>::from_vec(
            cell_protos,
        ));
        let attributes_meta = self
            .attributes
            .iter()
            .map(|(name, attribute)| {
                let mut attr_meta = proto::Attribute::new();
                attr_meta.set_name(name.to_string());
                attr_meta.set_data_type(attribute.to_proto());
                attr_meta
            })
            .collect();
        s2_meta.set_attributes(::protobuf::RepeatedField::<proto::Attribute>::from_vec(
            attributes_meta,
        ));
        meta.set_s2(s2_meta);
        meta
    }
}

/// Just a wrapper that implements Display
/// TODO(nnmm): Remove as soon as version 0.10 of s2 is released
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
            PointLocation::Aabb(aabb) => {
                self.cells_in_obb(&Obb::from(aabb), query.global_from_local.as_ref())
            }
            PointLocation::Obb(obb) => self.cells_in_obb(&obb, query.global_from_local.as_ref()),
            PointLocation::Frustum(mat) => {
                let world_from_clip = mat.inverse_transform().unwrap();
                let points = CUBE_CORNERS
                    .iter()
                    .map(|p| Point3::from_homogeneous(world_from_clip * p));
                self.cells_in_convex_hull(points)
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
        let num_points = self.meta.cells[&node_id.0].num_points as usize;
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

impl S2Cells {
    /// Wrapper arround cells_in_convex_hull for Obbs
    fn cells_in_obb(
        &self,
        obb: &Obb<f64>,
        global_from_local: Option<&Isometry3<f64>>,
    ) -> Vec<S2CellId> {
        let obb = match global_from_local {
            Some(isometry) => Cow::Owned(Obb::new(
                isometry * &obb.isometry_inv.inverse(),
                obb.half_extent,
            )),
            None => Cow::Borrowed(obb),
        };
        let points = obb.corners;
        self.cells_in_convex_hull(points.iter().cloned())
    }

    /// Returns all cells that intersect the convex hull of the given points
    fn cells_in_convex_hull<IterPoint>(&self, points: IterPoint) -> Vec<S2CellId>
    where
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
        self.cells
            .values()
            .filter(|cell| convex_hull.intersects_cell(cell))
            .map(|cell| S2CellId(cell.id))
            .collect()
    }
}

/// This is projected back with the inverse frustum matrix
/// to find the corners of the frustum
const CUBE_CORNERS: [Vector4<f64>; 8] = [
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
