use crate::proto;
use crate::{AttributeDataType, CURRENT_VERSION};
use fnv::FnvHashMap;
use s2::cellid::CellID;
use std::collections::HashMap;
use std::convert::TryInto;

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
