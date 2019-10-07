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

use point_viewer::octree::{DataProvider, NodeId, OnDiskDataProvider};
use point_viewer::proto;
use protobuf::Message;
use std::fs::File;
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use structopt::StructOpt;

#[derive(StructOpt, Debug)]
#[structopt(name = "upgrade_octree")]
struct CommandlineArguments {
    /// Directory of octree to upgrade.
    #[structopt(parse(from_os_str))]
    directory: PathBuf,
}

fn upgrade_version9(directory: &Path, mut meta: proto::Meta) {
    println!("Upgrading version 9 => 10.");
    for node_proto in &mut meta.deprecated_nodes.iter_mut() {
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

fn upgrade_version10(directory: &Path, mut meta: proto::Meta) {
    println!("Upgrading version 10 => 11.");
    let bbox = meta.bounding_box.as_mut().unwrap();
    let deprecated_min = bbox.take_deprecated_min();
    bbox.set_min(point_viewer::proto::Vector3d::from(deprecated_min));
    let deprecated_max = bbox.take_deprecated_max();
    bbox.set_max(point_viewer::proto::Vector3d::from(deprecated_max));

    meta.version = 11;
    let mut buf_writer = BufWriter::new(File::create(&directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

fn upgrade_version11(directory: &Path, mut meta: proto::Meta) {
    println!("Upgrading version 11 => 12.");
    let mut octree = proto::OctreeMeta::new();

    octree.set_resolution(meta.deprecated_resolution);
    meta.deprecated_resolution = 0.0;

    octree.set_nodes(meta.take_deprecated_nodes());

    meta.set_octree(octree);
    meta.version = 12;
    let mut buf_writer = BufWriter::new(File::create(&directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

fn upgrade_version12(directory: &Path, mut meta: proto::Meta) {
    println!("Upgrading version 12 => 13.");
    if meta.has_octree() {
        let bounding_box = meta.mut_octree().take_deprecated_bounding_box();
        meta.set_bounding_box(bounding_box);
    }
    meta.version = 13;
    let mut buf_writer = BufWriter::new(File::create(&directory.join("meta.pb")).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

fn main() {
    let args = CommandlineArguments::from_args();
    let data_provider = OnDiskDataProvider {
        directory: args.directory.clone(),
    };

    loop {
        let meta = data_provider
            .meta_proto()
            .expect("Could not read meta proto.");
        match meta.version {
            9 => upgrade_version9(&args.directory, meta),
            10 => upgrade_version10(&args.directory, meta),
            11 => upgrade_version11(&args.directory, meta),
            12 => upgrade_version12(&args.directory, meta),
            other if other == point_viewer::CURRENT_VERSION => {
                println!(
                    "Octree at current version {}",
                    point_viewer::CURRENT_VERSION
                );
                break;
            }
            other => {
                println!("Do not know how to upgrade version {}", other);
                std::process::exit(1);
            }
        }
    }
}
