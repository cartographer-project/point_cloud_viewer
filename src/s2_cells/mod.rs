use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::geometry::Aabb;
use crate::iterator::{PointCloud, PointLocation};
use crate::math::{ConvexPolyhedron, FromPoint3};
use crate::proto;
use crate::read_write::{Encoding, NodeIterator};
use crate::{AttributeDataType, PointCloudMeta, CURRENT_VERSION};
use fnv::FnvHashMap;
use s2::cell::Cell;
use s2::cellid::CellID;
use s2::cellunion::CellUnion;
use s2::region::Region;
use std::collections::HashMap;
use std::iter;

pub struct S2Cells {
    data_provider: Box<dyn DataProvider>,
    cells: FnvHashMap<CellID, Cell>,
    meta: S2Meta,
}

#[derive(Copy, Clone)]
pub struct S2CellMeta {
    pub num_points: u64,
}

impl S2CellMeta {
    pub fn to_proto(self, cell_id: u64) -> proto::S2Cell {
        let mut meta = proto::S2Cell::new();
        meta.set_id(cell_id);
        meta.set_num_points(self.num_points);
        meta
    }
}

pub struct S2Meta {
    cells: FnvHashMap<CellID, S2CellMeta>,
    attribute_data_types: HashMap<String, AttributeDataType>,
    bounding_box: Aabb,
}

impl PointCloudMeta for S2Meta {
    fn attribute_data_types(&self) -> &HashMap<String, AttributeDataType> {
        &self.attribute_data_types
    }
}

impl S2Meta {
    pub fn new(
        cells: FnvHashMap<CellID, S2CellMeta>,
        attribute_data_types: HashMap<String, AttributeDataType>,
        bounding_box: Aabb,
    ) -> Self {
        S2Meta {
            cells,
            attribute_data_types,
            bounding_box,
        }
    }

    pub fn iter_attr_with_xyz(&self) -> impl Iterator<Item = (&str, AttributeDataType)> {
        self.attribute_data_types
            .iter()
            .map(|(name, d_type)| (name.as_str(), *d_type))
            .chain(iter::once(("xyz", AttributeDataType::F64Vec3)))
    }

    pub fn get_cells(&self) -> &FnvHashMap<CellID, S2CellMeta> {
        &self.cells
    }

    pub fn bounding_box(&self) -> &Aabb {
        &self.bounding_box
    }

    pub fn to_proto(&self) -> proto::Meta {
        let cell_protos = self
            .cells
            .iter()
            .map(|(cell_id, cell_meta)| cell_meta.to_proto(cell_id.0))
            .collect();
        let mut meta = proto::Meta::new();
        meta.set_version(CURRENT_VERSION);
        meta.set_bounding_box(proto::AxisAlignedCuboid::from(&self.bounding_box));
        let mut s2_meta = proto::S2Meta::new();
        s2_meta.set_cells(::protobuf::RepeatedField::<proto::S2Cell>::from_vec(
            cell_protos,
        ));
        let attributes_meta = self
            .attribute_data_types
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

    pub fn from_proto(meta_proto: proto::Meta) -> Result<Self> {
        // check if the meta is meant to be for S2 point cloud
        if meta_proto.version < 12 {
            // from version 12
            return Err(ErrorKind::InvalidInput(format!(
                "No S2 point cloud supported with version {}",
                meta_proto.version
            ))
            .into());
        }
        if !(meta_proto.version >= 12 && meta_proto.has_s2()) {
            return Err(ErrorKind::InvalidInput(
                "This meta does not describe S2 point clouds".to_string(),
            )
            .into());
        }

        let bounding_box = Aabb::from(meta_proto.get_bounding_box());
        let s2_meta_proto = meta_proto.get_s2();
        // cells, num_points
        let mut cells = FnvHashMap::default();
        s2_meta_proto.get_cells().iter().for_each(|cell| {
            let cell_id = CellID(cell.id as u64);
            cells.insert(
                cell_id,
                S2CellMeta {
                    num_points: cell.num_points,
                },
            );
        });

        let mut attribute_data_types = HashMap::default();
        for attr in s2_meta_proto.attributes.iter() {
            let attr_type: AttributeDataType = AttributeDataType::from_proto(attr.get_data_type())?;
            attribute_data_types.insert(attr.name.to_owned(), attr_type);
        }

        Ok(S2Meta {
            cells,
            attribute_data_types,
            bounding_box,
        })
    }

    pub fn from_data_provider(data_provider: &dyn DataProvider) -> Result<Self> {
        let meta = data_provider.meta_proto()?;
        S2Meta::from_proto(meta)
    }
}

impl PointCloud for S2Cells {
    type Id = CellID;

    fn nodes_in_location(&self, location: &PointLocation) -> Vec<Self::Id> {
        match location {
            PointLocation::AllPoints => self.cells.keys().cloned().collect(),
            PointLocation::Aabb(aabb) => self.cells_in_convex_polyhedron(aabb),
            PointLocation::Obb(obb) => self.cells_in_convex_polyhedron(obb),
            PointLocation::Frustum(frustum) => self.cells_in_convex_polyhedron(frustum),
            PointLocation::S2Cells(cell_union) => self.cells_intersecting_region(cell_union),
            PointLocation::WebMercatorRect(wmr) => self.cells_in_convex_polyhedron(wmr),
        }
    }

    fn encoding_for_node(&self, _: Self::Id) -> Encoding {
        Encoding::Plain
    }

    fn points_in_node(
        &self,
        attributes: &[&str],
        node_id: Self::Id,
        batch_size: usize,
    ) -> Result<NodeIterator> {
        let num_points = self.meta.cells[&node_id].num_points as usize;
        let node_iterator = NodeIterator::from_data_provider(
            &*self.data_provider,
            &self.meta.attribute_data_types_for(&attributes)?,
            self.encoding_for_node(node_id),
            &node_id,
            num_points,
            batch_size,
        )?;
        Ok(node_iterator)
    }

    fn bounding_box(&self) -> &Aabb {
        &self.meta.bounding_box
    }
}

impl S2Cells {
    pub fn from_data_provider(data_provider: Box<dyn DataProvider>) -> Result<Self> {
        let meta_proto = data_provider.meta_proto()?;
        let meta = S2Meta::from_proto(meta_proto)?;
        let cells: FnvHashMap<_, _> = meta
            .get_cells()
            .keys()
            .map(|id| (*id, Cell::from(id)))
            .collect();
        Ok(S2Cells {
            data_provider,
            cells,
            meta,
        })
    }

    pub fn to_meta_proto(&self) -> proto::Meta {
        self.meta.to_proto()
    }

    /// Returns all cells that intersect this convex polyhedron
    fn cells_in_convex_polyhedron<T>(&self, poly: &T) -> Vec<CellID>
    where
        T: ConvexPolyhedron,
    {
        // We could choose either a covering rect or a covering cap as a convex hull
        let point_cells = poly
            .compute_corners()
            .iter()
            .map(|p| CellID::from_point(&p))
            .collect();
        let mut cell_union = CellUnion(point_cells);
        cell_union.normalize();
        let rect = cell_union.rect_bound();
        self.cells_intersecting_region(&rect)
    }

    fn cells_intersecting_region(&self, region: &impl Region) -> Vec<CellID> {
        self.cells
            .values()
            .filter(|cell| region.intersects_cell(cell))
            .map(|cell| cell.id)
            .collect()
    }
}
