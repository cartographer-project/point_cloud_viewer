use crate::read_write::{Encoding, NodeWriter};
use crate::s2_geo::{cell_id, ECEFExt};
use crate::{AttributeData, PointsBatch};
use std::collections::{BTreeMap, HashMap};
use std::io::Result;
use std::path::PathBuf;

pub struct S2Splitter<W> {
    writers: HashMap<String, W>,
    stem: PathBuf,
    split_level: u8,
}

impl<W> S2Splitter<W>
where
    W: NodeWriter<PointsBatch>,
{
    pub fn new(path: impl Into<PathBuf>, split_level: u8) -> Self {
        let writers = HashMap::new();
        let stem = path.into();
        S2Splitter {
            writers,
            stem,
            split_level,
        }
    }

    pub fn write(&mut self, points_batch: &PointsBatch) -> Result<()> {
        let mut out = HashMap::new();
        for (i, pos) in points_batch.position.iter().enumerate() {
            let out_batch = out
                .entry(cell_id(ECEFExt::from(*pos), self.split_level).to_token())
                .or_insert(PointsBatch {
                    position: Vec::new(),
                    attributes: BTreeMap::new(),
                });
            out_batch.position.push(*pos);
            for (in_key, in_data) in &points_batch.attributes {
                use AttributeData::*;
                let key = in_key.to_string();
                out_batch
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

        for (key, batch) in &out {
            self.writer(key).write(batch)?;
        }
        Ok(())
    }

    fn writer(&mut self, key: &str) -> &mut W {
        let path = self.stem.join(key.to_string());
        self.writers
            .entry(key.to_string())
            .or_insert_with(|| W::from(path, Encoding::Plain))
    }
}
