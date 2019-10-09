// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use crate::data_provider::DataProvider;
use crate::errors::*;
use crate::read_write::{AttributeReader, Encoding, RawNodeReader};
use crate::{AttributeDataType, NumberOfPoints, PointsBatch};
use num_integer::div_ceil;
use std::collections::HashMap;
use std::io::BufReader;

/// Streams points from our data provider representation.
pub struct NodeIterator {
    reader: Option<RawNodeReader>,
    num_points: usize,
    point_count: usize,
    batch_size: usize,
}

impl Default for NodeIterator {
    fn default() -> Self {
        NodeIterator {
            reader: None,
            num_points: 0,
            point_count: 0,
            batch_size: 0,
        }
    }
}

impl NodeIterator {
    pub fn new(reader: RawNodeReader, num_points: usize, batch_size: usize) -> Self {
        if num_points == 0 {
            return NodeIterator::default();
        }

        NodeIterator {
            reader: Some(reader),
            num_points,
            point_count: 0,
            batch_size,
        }
    }

    pub fn from_data_provider<Id: ToString>(
        data_provider: &dyn DataProvider,
        attributes: &[&str],
        encoding: Encoding,
        id: &Id,
        num_points: usize,
        batch_size: usize,
    ) -> Result<Self> {
        if num_points == 0 {
            return Ok(NodeIterator::default());
        }

        let attribute_data_types = match data_provider.meta_proto() {
            Ok(ref meta) if meta.has_s2() => meta
                .get_s2()
                .get_attributes()
                .iter()
                .map(|a| {
                    AttributeDataType::from_proto(a.data_type)
                        .map(|data_type| (a.name.clone(), data_type))
                })
                .collect::<Result<HashMap<String, AttributeDataType>>>()?,
            _ => vec![
                ("color".to_string(), AttributeDataType::U8Vec3),
                ("intensity".to_string(), AttributeDataType::F32),
            ]
            .into_iter()
            .collect(),
        };

        let mut all_reads =
            data_provider.data(&id.to_string(), &[&["position"], attributes].concat())?;
        // Unwrapping all following removals is safe,
        // as the data provider would already have errored on unavailability.
        let position_reader = all_reads.remove("position").unwrap();

        let mut attribute_readers = HashMap::new();
        for attribute in attributes {
            let data_type = *attribute_data_types.get(*attribute).ok_or_else(|| {
                format!(
                    "Attribute data type for {} not found in meta data.",
                    attribute
                )
            })?;
            let reader = BufReader::new(all_reads.remove(*attribute).unwrap());
            let attribute_reader = AttributeReader { data_type, reader };
            attribute_readers.insert(attribute.to_string(), attribute_reader);
        }

        Ok(Self::new(
            RawNodeReader::new(position_reader, attribute_readers, encoding)?,
            num_points,
            batch_size,
        ))
    }
}

impl NumberOfPoints for NodeIterator {
    fn num_points(&self) -> usize {
        self.num_points
    }
}

impl Iterator for NodeIterator {
    type Item = PointsBatch;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_batches = div_ceil(self.num_points, self.batch_size);
        (num_batches, Some(num_batches))
    }
    fn next(&mut self) -> Option<PointsBatch> {
        if let Some(reader) = &mut self.reader {
            if self.point_count < self.num_points {
                let num_points_to_read =
                    std::cmp::min(self.batch_size, self.num_points - self.point_count);
                let res = reader
                    .read_batch(num_points_to_read)
                    .expect("Couldn't read from node.");
                self.point_count += num_points_to_read;
                return Some(res);
            }
        }
        None
    }
}
