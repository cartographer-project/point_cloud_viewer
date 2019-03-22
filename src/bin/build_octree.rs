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

use point_viewer::generation::{build_octree_from_file, RootBbox};
use std::path::PathBuf;
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(name = "build_octree")]
struct CommandlineArguments {
    /// PLY/PTS file to parse for the points.
    #[structopt(parse(from_os_str))]
    input: PathBuf,
    /// Output directory to write the octree into.
    #[structopt(long = "output_directory", parse(from_os_str))]
    output_directory: PathBuf,
    /// Minimal precision that this point cloud should have.
    /// This decides on the number of bits used to encode each node.
    #[structopt(long = "resolution", default_value = "0.001")]
    resolution: f64,
    // Flag to skip bbox calculation and use a fixed bbox.
    // The fixed bbox is extends 6400000 length units in each direction.
    #[structopt(long = "bbox_type", raw(possible_values = "&RootBbox::variants()", case_insensitive = "true"), default_value = "FromData")]
    bbox_type: RootBbox,
}

fn main() {
    let args = CommandlineArguments::from_args();
    let pool = scoped_pool::Pool::new(10);
    build_octree_from_file(&pool, args.output_directory, args.resolution, args.input, args.bbox_type);
}
