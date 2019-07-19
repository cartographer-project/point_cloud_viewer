use crate::read_write::{Encoding, NodeWriter, OpenMode};
use crate::s2_geo::{cell_id, ECEFExt};
use crate::{AttributeData, PointsBatch};
use lru::LruCache;
use s2::cellid::CellID;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::io::Result;
use std::path::PathBuf;

// The actual number of underlying writers is MAX_NUM_NODE_WRITERS * num_attributes.
const MAX_NUM_NODE_WRITERS: usize = 25;
// Corresponds to cells of up to about 10m x 10m.
const S2_SPLIT_LEVEL: u8 = 20;

pub struct S2Splitter<W> {
    writers: LruCache<CellID, W>,
    already_opened_writers: HashSet<CellID>,
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
        S2Splitter {
            writers,
            already_opened_writers,
            encoding,
            open_mode,
            stem,
        }
    }

    fn write(&mut self, points_batch: &PointsBatch) -> Result<()> {
        let mut batches_by_s2_cell = HashMap::new();
        for (i, pos) in points_batch.position.iter().enumerate() {
            let s2_cell_batch = batches_by_s2_cell
                .entry(cell_id(ECEFExt::from(*pos), S2_SPLIT_LEVEL))
                .or_insert(PointsBatch {
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
                        (I64(in_vec), I64(out_vec)) => out_vec.push(in_vec[i]),
                        (U64(in_vec), U64(out_vec)) => out_vec.push(in_vec[i]),
                        (F32(in_vec), F32(out_vec)) => out_vec.push(in_vec[i]),
                        (F64(in_vec), F64(out_vec)) => out_vec.push(in_vec[i]),
                        (U8Vec3(in_vec), U8Vec3(out_vec)) => out_vec.push(in_vec[i]),
                        (F64Vec3(in_vec), F64Vec3(out_vec)) => out_vec.push(in_vec[i]),
                        _ => panic!("Input data type unequal output data type."),
                    })
                    .or_insert(match in_data {
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
}
