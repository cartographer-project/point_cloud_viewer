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

use clap::Clap;
use point_viewer::octree::build_octree_from_file;
use rayon::ThreadPoolBuilder;
use std::path::PathBuf;

#[derive(Clap, Debug)]
#[clap(name = "build_octree")]
struct CommandlineArguments {
    /// PLY/PTS file to parse for the points.
    #[clap(parse(from_os_str))]
    input: PathBuf,

    /// Output directory to write the octree into.
    #[clap(long, parse(from_os_str))]
    output_directory: PathBuf,

    /// Minimal precision that this point cloud should have.
    /// This decides on the number of bits used to encode each node.
    #[clap(long, default_value = "0.001")]
    resolution: f64,

    /// The number of threads used to shard octree building. Set this as high as possible for SSDs.
    #[clap(long, default_value = "10")]
    num_threads: usize,
}

fn main() {
    let args = CommandlineArguments::parse();
    ThreadPoolBuilder::new()
        .num_threads(args.num_threads)
        .build_global()
        .expect("Could not create thread pool.");
    build_octree_from_file(
        args.output_directory,
        args.resolution,
        args.input,
        &["color", "intensity"],
    );
}
