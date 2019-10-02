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

use crate::read_write::PlyIterator;
use crate::Point;
use pbr::ProgressBar;
use std::io::Stdout;
use std::path::PathBuf;

#[derive(Debug)]
pub enum InputFile {
    Ply(PathBuf),
}

pub enum InputFileIterator {
    Ply(PlyIterator),
}

impl Iterator for InputFileIterator {
    type Item = Point;

    fn size_hint(&self) -> (usize, Option<usize>) {
        match *self {
            InputFileIterator::Ply(ref p) => p.size_hint(),
        }
    }

    fn next(&mut self) -> Option<Point> {
        match self {
            InputFileIterator::Ply(p) => p.next(),
        }
    }
}

pub fn make_stream(input: &InputFile) -> (InputFileIterator, Option<ProgressBar<Stdout>>) {
    let stream = match *input {
        InputFile::Ply(ref filename) => {
            InputFileIterator::Ply(PlyIterator::from_file(filename).unwrap())
        }
    };

    let progress_bar = match stream.size_hint() {
        (_, Some(size)) => Some(ProgressBar::new(size as u64)),
        (_, None) => None,
    };
    (stream, progress_bar)
}
