use crate::proto;
use crate::read_write::{Encoding, NodeWriter, OpenMode};
use crate::{AttributeData, AttributeDataType, PointsBatch, CURRENT_VERSION};
use cgmath::InnerSpace;
use fnv::FnvHashMap;
use lru::LruCache;
use protobuf::Message;
use s2::cellid::CellID;
use s2::point::Point;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs::File;
use std::io::{BufWriter, Error, ErrorKind, Result};
use std::iter::Iterator;
use std::path::PathBuf;
use crate::s2_cells::{S2CellMeta, S2Meta};

/// The actual number of underlying writers is MAX_NUM_NODE_WRITERS * num_attributes.
const MAX_NUM_NODE_WRITERS: usize = 25;
/// Corresponds to cells of up to about 10m x 10m.
const S2_SPLIT_LEVEL: u64 = 20;
/// Lower bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
const EARTH_RADIUS_MIN_M: f64 = 6_352_800.0;
/// Upper bound for distance from earth's center.
/// See https://en.wikipedia.org/wiki/Earth_radius#Geophysical_extremes
const EARTH_RADIUS_MAX_M: f64 = 6_384_400.0;

pub struct S2Splitter<W> {
    writers: LruCache<CellID, W>,
    already_opened_writers: HashSet<CellID>,
    cell_stats: FnvHashMap<CellID, S2CellMeta>,
    attributes_seen: BTreeMap<String, AttributeDataType>,
    encoding: Encoding,
    open_mode: OpenMode,
    stem: PathBuf,
}

impl<W> NodeWriter<PointsBatch> for S2Splitter<W>
where
    W: NodeWriter<PointsBatch>,
{
    fn new(path: impl Into<PathBuf>, encoding: Encoding, open_mode: OpenMode) -> Self {
        let writers = LruCache::new(MAX_NUM_NODE_WRITERS);
        let already_opened_writers = HashSet::new();
        let stem = path.into();
        let cell_stats = FnvHashMap::default();
        let attributes_seen = BTreeMap::new();
        S2Splitter {
            writers,
            already_opened_writers,
            cell_stats,
            attributes_seen,
            encoding,
            open_mode,
            stem,
        }
    }

    fn write(&mut self, points_batch: &PointsBatch) -> Result<()> {
        self.check_attributes(points_batch)?;
        let mut batches_by_s2_cell = HashMap::new();
        for (i, pos) in points_batch.position.iter().enumerate() {
            let radius = pos.magnitude();
            if radius > EARTH_RADIUS_MAX_M || radius < EARTH_RADIUS_MIN_M {
                let msg = format!(
                    "Point ({}, {}, {}) is not a valid ECEF point",
                    pos.x, pos.y, pos.z
                );
                return Err(Error::new(ErrorKind::InvalidInput, msg));
            }
            let s2_point = Point::from_coords(pos.x, pos.y, pos.z);
            let s2_cell_id = CellID::from(s2_point).parent(S2_SPLIT_LEVEL);
            self.cell_stats.entry(s2_cell_id).or_insert(S2CellMeta { num_points: 0 }).num_points += 1;
            let s2_cell_batch = batches_by_s2_cell.entry(s2_cell_id).or_insert(PointsBatch {
                position: Vec::new(),
                attributes: BTreeMap::new(),
            });
            s2_cell_batch.position.push(*pos);
            for (in_key, in_data) in &points_batch.attributes {
                use AttributeData::*;
                let key = in_key.to_string();
                s2_cell_batch
                    .attributes
                    .entry(key)
                    .and_modify(|out_data| match (in_data, out_data) {
                        (U8(in_vec), U8(out_vec)) => out_vec.push(in_vec[i]),
                        (I64(in_vec), I64(out_vec)) => out_vec.push(in_vec[i]),
                        (U64(in_vec), U64(out_vec)) => out_vec.push(in_vec[i]),
                        (F32(in_vec), F32(out_vec)) => out_vec.push(in_vec[i]),
                        (F64(in_vec), F64(out_vec)) => out_vec.push(in_vec[i]),
                        (U8Vec3(in_vec), U8Vec3(out_vec)) => out_vec.push(in_vec[i]),
                        (F64Vec3(in_vec), F64Vec3(out_vec)) => out_vec.push(in_vec[i]),
                        _ => panic!("Input data type unequal output data type."),
                    })
                    .or_insert(match in_data {
                        U8(in_vec) => U8(vec![in_vec[i]]),
                        I64(in_vec) => I64(vec![in_vec[i]]),
                        U64(in_vec) => U64(vec![in_vec[i]]),
                        F32(in_vec) => F32(vec![in_vec[i]]),
                        F64(in_vec) => F64(vec![in_vec[i]]),
                        U8Vec3(in_vec) => U8Vec3(vec![in_vec[i]]),
                        F64Vec3(in_vec) => F64Vec3(vec![in_vec[i]]),
                    });
            }
        }

        for (cell_id, batch) in &batches_by_s2_cell {
            self.writer(cell_id).write(batch)?;
        }
        Ok(())
    }
}

impl<W> S2Splitter<W>
where
    W: NodeWriter<PointsBatch>,
{
    fn writer(&mut self, cell_id: &CellID) -> &mut W {
        let path = self.stem.join(cell_id.to_token());
        if !self.writers.contains(cell_id) {
            let open_mode = if self.open_mode == OpenMode::Append
                || self.already_opened_writers.contains(cell_id)
            {
                OpenMode::Append
            } else {
                self.already_opened_writers.insert(*cell_id);
                OpenMode::Truncate
            };
            self.writers
                .put(*cell_id, W::new(path, self.encoding.clone(), open_mode));
        }
        self.writers.get_mut(cell_id).unwrap()
    }

    fn check_attributes(&mut self, batch: &PointsBatch) -> Result<()> {
        let mut attr_iter = batch
            .attributes
            .iter()
            .map(|(name, data)| (name, data.data_type()));
        if self.attributes_seen.len() == 0 {
            self.attributes_seen
                .extend(attr_iter.map(|(key, val)| (key.to_owned(), val)));
            Ok(())
        } else {
            attr_iter.try_for_each(|(name, dtype)| {
                self.attributes_seen
                    .get(name)
                    .filter(|seen_dtype| **seen_dtype == dtype)
                    .ok_or_else(|| {
                        let msg = format!(
                            "S2Splitter received incompatible data types for attribute {}",
                            name
                        );
                        Error::new(ErrorKind::InvalidInput, msg)
                    })?;
                Ok(())
            })
        }
    }

    fn get_meta(self) -> S2Meta {
        S2Meta {
            attributes: self.attributes_seen.into_iter().collect(),
            nodes: self.cell_stats,
        }
    }
}

pub fn s2_cloud_to_meta_proto(
    cells: Vec<proto::S2Cell>,
    attributes: &BTreeMap<String, AttributeDataType>,
) -> proto::Meta {
    let mut meta = proto::Meta::new();
    meta.set_version(CURRENT_VERSION);
    let mut s2_meta = proto::S2Meta::new();
    s2_meta.set_cells(::protobuf::RepeatedField::<proto::S2Cell>::from_vec(cells));
    let attributes_meta = attributes
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

pub fn s2_cell_to_proto(cell_id: u64, num_points: i64) -> proto::S2Cell {
    let mut meta = proto::S2Cell::new();
    meta.set_id(cell_id);
    meta.set_num_points(num_points);
    meta
}
