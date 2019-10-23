use crate::math::{EARTH_RADIUS_MAX_M, EARTH_RADIUS_MIN_M};
use crate::read_write::{Encoding, NodeWriter, OpenMode};
use crate::s2_cells::{S2CellMeta, S2Meta};
use crate::{AttributeData, AttributeDataType, PointsBatch};
use cgmath::{EuclideanSpace, InnerSpace, Point3};
use collision::{Aabb, Aabb3};
use fnv::FnvHashMap;
use lru::LruCache;
use s2::cellid::CellID;
use s2::point::Point;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::{Error, ErrorKind, Result};
use std::iter::Iterator;
use std::path::PathBuf;

/// The actual number of underlying writers is MAX_NUM_NODE_WRITERS * num_attributes.
const MAX_NUM_NODE_WRITERS: usize = 25;
/// Corresponds to cells of up to about 160m x 160m.
const S2_SPLIT_LEVEL: u64 = 16;

pub struct S2Splitter<W> {
    writers: LruCache<CellID, W>,
    already_opened_writers: HashSet<CellID>,
    cell_stats: FnvHashMap<CellID, S2CellMeta>,
    bounding_box: Option<Aabb3<f64>>,
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
        let bounding_box = None;
        let attributes_seen = BTreeMap::new();
        S2Splitter {
            writers,
            already_opened_writers,
            cell_stats,
            bounding_box,
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
            let p3 = Point3::from_vec(*pos);
            let b = self.bounding_box.get_or_insert(Aabb3::new(p3, p3));
            *b = b.grow(p3);
            let s2_point = Point::from_coords(pos.x, pos.y, pos.z);
            let s2_cell_id = CellID::from(s2_point).parent(S2_SPLIT_LEVEL);
            self.cell_stats
                .entry(s2_cell_id)
                .or_insert(S2CellMeta { num_points: 0 })
                .num_points += 1;
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

    /// Records the list of attributes seen in the first batch, and checks
    /// that the following batches contain the same attributes.
    fn check_attributes(&mut self, batch: &PointsBatch) -> Result<()> {
        let mut attr_iter = batch
            .attributes
            .iter()
            .map(|(name, data)| (name, data.data_type()));
        if self.attributes_seen.is_empty() {
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

    pub fn get_meta(self) -> S2Meta {
        S2Meta::new(
            self.cell_stats,
            self.attributes_seen.into_iter().collect(),
            self.bounding_box.unwrap_or_else(Aabb3::zero),
        )
    }
}
