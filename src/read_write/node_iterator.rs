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

use crate::read_write::RawNodeReader;
use crate::{NumberOfPoints, PointsBatch};
use num_integer::div_ceil;

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
