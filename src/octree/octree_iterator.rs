use crate::errors::*;
use crate::octree::{ChildIndex, DataProvider, NodeId, Octree};
use crate::read_write::{AttributeReader, Encoding, NodeIterator, RawNodeReader};
use crate::AttributeDataType;
use std::collections::{HashMap, VecDeque};
use std::io::BufReader;

impl NodeIterator {
    pub fn from_data_provider<Id: ToString>(
        data_provider: &dyn DataProvider,
        encoding: Encoding,
        id: &Id,
        num_points: usize,
        batch_size: usize,
    ) -> Result<Self> {
        if num_points == 0 {
            return Ok(NodeIterator::default());
        }

        let mut attributes = HashMap::new();

        let mut position_color_reads =
            data_provider.data(&id.to_string(), &["position", "color"])?;
        let position_read = position_color_reads
            .remove("position")
            .ok_or_else(|| -> Error { "No position reader available.".into() })?;
        match position_color_reads.remove("color") {
            Some(color_data) => {
                let color_reader = AttributeReader {
                    data_type: AttributeDataType::U8Vec3,
                    reader: BufReader::new(color_data),
                };
                attributes.insert("color".to_string(), color_reader);
            }
            None => return Err("No color reader available.".into()),
        }

        if let Ok(mut data_map) = data_provider.data(&id.to_string(), &["intensity"]) {
            match data_map.remove("intensity") {
                Some(intensity_data) => {
                    let intensity_reader = AttributeReader {
                        data_type: AttributeDataType::F32,
                        reader: BufReader::new(intensity_data),
                    };
                    attributes.insert("intensity".to_string(), intensity_reader);
                }
                None => return Err("No intensity reader available.".into()),
            }
        };

        Ok(Self::new(
            RawNodeReader::new(position_read, attributes, encoding)?,
            num_points,
            batch_size,
        ))
    }
}

pub struct NodeIdsIterator<'a, F> {
    octree: &'a Octree,
    filter_func: F,
    node_ids: VecDeque<NodeId>,
}

impl<'a, F> NodeIdsIterator<'a, F>
where
    F: Fn(&NodeId, &Octree) -> bool,
{
    pub fn new(octree: &'a Octree, filter_func: F) -> NodeIdsIterator<'a, F> {
        NodeIdsIterator {
            octree,
            node_ids: vec![NodeId::from_level_index(0, 0)].into(),
            filter_func,
        }
    }
}

impl<'a, F> Iterator for NodeIdsIterator<'a, F>
where
    F: Fn(&NodeId, &'a Octree) -> bool,
{
    type Item = NodeId;

    fn next(&mut self) -> Option<NodeId> {
        while let Some(current) = self.node_ids.pop_front() {
            if (self.filter_func)(&current, &self.octree) {
                for child_index in 0..8 {
                    let child_id = current.get_child_id(ChildIndex::from_u8(child_index));
                    if self.octree.nodes.contains_key(&child_id) {
                        self.node_ids.push_back(child_id);
                    }
                }
                return Some(current);
            }
        }
        None
    }
}
