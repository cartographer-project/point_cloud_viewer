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

use {InternalIterator, Point};
use math::Vector3f;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

#[derive(Debug)]
pub struct PtsIterator {
    data: BufReader<File>,
}

impl PtsIterator {
    pub fn new(filename: &Path) -> Self {
        let file = File::open(filename).unwrap();
        PtsIterator { data: BufReader::new(file) }
    }
}

impl InternalIterator for PtsIterator {
    fn size_hint(&self) -> Option<usize> {
        None
    }

    fn for_each<F: FnMut(&Point)>(mut self, mut f: F) {
        let mut line = String::new();
        loop {
            line.clear();
            self.data.read_line(&mut line).unwrap();
            if line.is_empty() {
                break;
            }

            let parts: Vec<&str> = line.trim().split(|c| c == ' ' || c == ',').collect();
            if parts.len() != 7 {
                continue;
            }
            let p = Point {
                position: Vector3f::new(
                    parts[0].parse::<f32>().unwrap(),
                    parts[1].parse::<f32>().unwrap(),
                    parts[2].parse::<f32>().unwrap(),
                ),
                r: parts[4].parse::<u8>().unwrap(),
                g: parts[5].parse::<u8>().unwrap(),
                b: parts[6].parse::<u8>().unwrap(),
            };
            f(&p);
        }
    }
}
