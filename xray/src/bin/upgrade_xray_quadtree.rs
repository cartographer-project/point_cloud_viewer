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
use protobuf::Message;
use std::fs::File;
use std::io::{BufWriter, Cursor};
use std::path::{Path, PathBuf};
use xray::META_FILENAME;
use xray_proto_rust::proto;

#[derive(Clap, Debug)]
#[clap(name = "upgrade_xray_quadtree")]
struct CommandlineArguments {
    /// Directory of xray quadtree to upgrade.
    #[clap(parse(from_os_str))]
    directory: PathBuf,
}

fn upgrade_version2(filename: &Path, mut meta: proto::Meta) {
    eprintln!("Upgrading version 2 => 3.");
    let bounding_rect = meta.mut_bounding_rect();
    let deprecated_min = bounding_rect.get_deprecated_min();
    let mut min = proto::Vector2d::new();
    min.set_x(f64::from(deprecated_min.x));
    min.set_y(f64::from(deprecated_min.y));
    bounding_rect.set_min(min);
    bounding_rect.deprecated_min.clear();
    bounding_rect.set_edge_length(f64::from(bounding_rect.get_deprecated_edge_length()));

    meta.version = 3;
    let mut buf_writer = BufWriter::new(File::create(filename).unwrap());
    meta.write_to_writer(&mut buf_writer).unwrap();
}

fn main() {
    let args = CommandlineArguments::parse();
    let filename = args.directory.join(META_FILENAME);

    loop {
        let meta = {
            let data = std::fs::read(&filename).unwrap();
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
                .expect("Could not read meta proto")
        };
        match meta.version {
            2 => upgrade_version2(&filename, meta),
            other if other == xray::CURRENT_VERSION => {
                eprintln!("Xray quadtree at current version {}", xray::CURRENT_VERSION);
                break;
            }
            other => {
                eprintln!("Do not know how to upgrade version {}", other);
                std::process::exit(1);
            }
        }
    }
}
