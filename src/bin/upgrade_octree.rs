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

use point_viewer::octree::{self, NodeId, OctreeDataProvider, OnDiskOctreeDataProvider};
use point_viewer::proto;
use protobuf::Message;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};

fn upgrade_version9(directory: &Path, mut meta: proto::Meta) {
    println!("Upgrading version 9 => 10.");
    for node_proto in &mut meta.nodes.iter_mut() {
        let mut id = node_proto.id.as_mut().unwrap();
        let node_id = NodeId::from_proto(id);
        id.deprecated_level = 0;
        id.deprecated_index = 0;
        *id = node_id.to_proto();
    }
    meta.version = 10;
    let mut buf_writer = BufWriter::new(File::create(&directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

fn main() {
    let matches = clap::App::new("upgrade_octree")
        .args(&[clap::Arg::with_name("directory")
            .help("Directory of octree to upgrade.")
            .required(true)
            .takes_value(true)])
        .get_matches();

    let directory = PathBuf::from(matches.value_of("directory").unwrap());
    let data_provider = OnDiskOctreeDataProvider {
        directory: directory.clone(),
    };

    loop {
        let meta = data_provider
            .meta_proto()
            .expect("Could not read meta proto.");
        match meta.version {
            9 => upgrade_version9(&directory, meta),
            other if other == octree::CURRENT_VERSION => {
                println!("Octree at current version {}", octree::CURRENT_VERSION);
                break;
            }
            other => {
                println!("Do not know how to upgrade version {}", other);
                std::process::exit(1);
            }
        }
    }
}
