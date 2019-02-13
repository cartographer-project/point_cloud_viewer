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

use point_viewer::generation::build_octree_from_file;
use std::path::PathBuf;

fn main() {
    let matches = clap::App::new("build_octree")
        .args(&[
            clap::Arg::with_name("output_directory")
                .help("Output directory to write the octree into.")
                .long("output_directory")
                .required(true)
                .takes_value(true),
            clap::Arg::with_name("resolution")
                .help(
                    "Minimal precision that this point cloud should have. This decides \
                     on the number of bits used to encode each node.",
                )
                .long("resolution")
                .default_value("0.001"),
            clap::Arg::with_name("input")
                .help("PLY/PTS file to parse for the points.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let output_directory = &PathBuf::from(matches.value_of("output_directory").unwrap());
    let resolution = matches
        .value_of("resolution")
        .unwrap()
        .parse::<f64>()
        .expect("resolution could not be parsed as float.");

    let filename = PathBuf::from(matches.value_of("input").unwrap());

    let pool = scoped_pool::Pool::new(10);
    build_octree_from_file(&pool, output_directory, resolution, filename);
}
