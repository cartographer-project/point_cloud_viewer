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

use point_viewer::octree::build_octree_from_file;
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

    /// The number of threads used to shard octree building. Set this as high as possible for SSDs.
    #[structopt(long = "num_threads", default_value = "10")]
    num_threads: usize,
}

fn main() {
    let args = CommandlineArguments::from_args();
    let pool = scoped_pool::Pool::new(args.num_threads);
    build_octree_from_file(&pool, args.output_directory, args.resolution, args.input);
}
