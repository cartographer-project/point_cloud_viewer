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

use crate::read_write::{DataWriter, NodeWriter, WriteLE};
use crate::{NodeLayer, Point};
use std::path::PathBuf;

pub struct RawNodeWriter {
    xyz_writer: DataWriter,
    layer_writers: Vec<DataWriter>,
    stem: PathBuf,
}

impl NodeWriter for RawNodeWriter {
    fn write(&mut self, p: &Point) {
        p.position.write_le(&mut self.xyz_writer);
        p.color.write_le(&mut self.layer_writers[0]);
        if let Some(i) = p.intensity {
            if self.layer_writers.len() < 2 {
                self.layer_writers.push(
                    DataWriter::new(&self.stem.with_extension(NodeLayer::Intensity.extension()))
                        .unwrap(),
                );
            }
            i.write_le(&mut self.layer_writers[1]);
        };
    }
}

impl RawNodeWriter {
    pub fn new(path: impl Into<PathBuf>) -> Self {
        let stem: PathBuf = path.into();
        let xyz_writer =
            DataWriter::new(&stem.with_extension(NodeLayer::Position.extension())).unwrap();
        let layer_writers =
            vec![DataWriter::new(&stem.with_extension(NodeLayer::Color.extension())).unwrap()];
        Self {
            xyz_writer,
            layer_writers,
            stem,
        }
    }
}
