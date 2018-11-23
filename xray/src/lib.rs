extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
extern crate fnv;
extern crate image;
#[macro_use]
extern crate iron;
extern crate point_viewer;
extern crate protobuf;
extern crate quadtree;
extern crate router;
extern crate scoped_pool;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;
extern crate urlencoded;
extern crate xray_proto_rust;
extern crate stats;

use cgmath::Point2;
use fnv::FnvHashSet;
use quadtree::{NodeId, Rect};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;

pub use xray_proto_rust::proto;

pub const CURRENT_VERSION: i32 = 2;

pub fn read_meta_proto_from_file<P: AsRef<Path>>(filename: P) -> proto::Meta {
    let f = File::open(&filename).expect("Could not open meta.pb");
    let meta = protobuf::parse_from_reader::<proto::Meta>(&mut BufReader::new(f))
        .expect("Could not parse meta.pb");

    assert!(meta.version == CURRENT_VERSION,
            "Invalid meta.pb version. We only support {}, but found {}.",
            CURRENT_VERSION,
            meta.version);

    meta
}

#[derive(Clone, Debug)]
pub struct Meta {
    pub nodes: FnvHashSet<NodeId>,
    pub bounding_rect: Rect,
    pub tile_size: u32,
    pub deepest_level: u8,
}

impl Meta {
    // Reads the meta file from disk. Panics on error
    pub fn from_disk<P: AsRef<Path>>(filename: P) -> Self {
        let meta = read_meta_proto_from_file(filename);

        Meta {
            nodes: meta.nodes
                .iter()
                .map(|n| NodeId::new(n.level as u8, n.index))
                .collect(),
            bounding_rect: Rect::new(
                Point2::new(
                    meta.get_bounding_rect().get_min().x,
                    meta.get_bounding_rect().get_min().y,
                ),
                meta.get_bounding_rect().get_edge_length(),
            ),
            tile_size: meta.tile_size,
            deepest_level: meta.deepest_level as u8,
        }
    }
}

pub mod backend;
pub mod generation;
