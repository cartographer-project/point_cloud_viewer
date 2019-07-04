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

use crate::Point;
use std::io::Result;

pub trait NodeReader {
    fn read(&mut self) -> Result<Point>;
    fn num_points(&self) -> usize;
}

/// Streams points from our data provider representation.
pub enum NodeIterator<R> {
    WithData(R),
    Empty,
}

impl<R> NodeIterator<R>
where
    R: NodeReader,
{
    pub fn new(reader: R) -> Self {
        if reader.num_points() == 0 {
            return NodeIterator::Empty;
        }

        NodeIterator::WithData(reader)
    }
}

impl<R> Iterator for NodeIterator<R>
where
    R: NodeReader,
{
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        let num_points = match self {
            NodeIterator::WithData(ref reader) => reader.num_points(),
            NodeIterator::Empty => 0,
        };
        (num_points, Some(num_points))
    }
    fn next(&mut self) -> Option<Point> {
        let reader = match self {
            NodeIterator::WithData(reader) => reader,
            NodeIterator::Empty => return None,
        };
        reader.read().ok()
    }
}
