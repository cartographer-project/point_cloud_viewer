// Copyright 2016 The Cartographer Authors
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

use std::io::BufReader;
use std::fs::{self, File};
use std::path::Path;
use Point;
use ply;
use errors::*;

/// Abstraction to read binary blobs into points.
pub struct PointStream {
    data: BufReader<File>,
    num_points_read: i64,
    pub num_total_points: i64,
}

impl Iterator for PointStream {
    type Item = Point;

    fn next(&mut self) -> Option<Self::Item> {
        if self.num_points_read >= self.num_total_points {
            return None;
        }
        let point = ply::read_point(&mut self.data);
        self.num_points_read += 1;
        Some(point)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.num_total_points as usize, Some(self.num_total_points as usize))
    }
}

impl PointStream {
    pub fn from_ply(ply_file: &Path) -> Self {
        let (file, num_total_points) = ply::open(ply_file);
        Self::from_reader_and_count(BufReader::new(file), num_total_points)
    }

    pub fn from_blob(blob_path: &Path) -> Result<Self> {
        let num_total_points = fs::metadata(blob_path)?.len() as i64 / 15;
        let file = File::open(blob_path)?;
        Ok(Self::from_reader_and_count(BufReader::new(file), num_total_points))
    }

    fn from_reader_and_count(data: BufReader<File>, num_total_points: i64) -> Self {
        PointStream {
            data: data,
            num_total_points: num_total_points,
            num_points_read: 0,
        }
    }
}
