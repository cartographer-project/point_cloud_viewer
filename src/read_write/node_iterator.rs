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
use crate::Point;

/// Streams points from our data provider representation.
pub struct PointIterator {
    reader: Option<RawNodeReader>,
    num_points: usize,
    point_count: usize,
}

impl Default for PointIterator {
    fn default() -> Self {
        PointIterator {
            reader: None,
            num_points: 0,
            point_count: 0,
        }
    }
}

impl PointIterator {
    pub fn new(reader: RawNodeReader, num_points: usize) -> Self {
        if num_points == 0 {
            return PointIterator::default();
        }

        PointIterator {
            reader: Some(reader),
            num_points,
            point_count: 0,
        }
    }
}

impl Iterator for PointIterator {
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.num_points, Some(self.num_points))
    }
    fn next(&mut self) -> Option<Point> {
        if let Some(reader) = &mut self.reader {
            if self.point_count < self.num_points {
                let res = reader.read().expect("Couldn't read from node.");
                self.point_count += 1;
                return Some(res);
            }
        }
        None
    }
}
