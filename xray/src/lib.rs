extern crate cgmath;
extern crate collision;
extern crate fnv;
#[macro_use]
extern crate iron;
extern crate protobuf;
extern crate quadtree;
extern crate router;
extern crate serde;
#[macro_use]
extern crate serde_derive;
extern crate serde_json;
extern crate urlencoded;
extern crate xray_proto_rust;
#[macro_use]
extern crate clap;
extern crate image;
extern crate point_viewer;


use cgmath::Point2;
use fnv::FnvHashSet;
use quadtree::{NodeId, Rect};
use std::fs::File;
use std::io::{Cursor, Read};
use std::path::Path;

pub const CURRENT_VERSION: i32 = 1;

#[derive(Debug)]
pub struct Meta {
    pub nodes: FnvHashSet<NodeId>,
    pub bounding_rect: Rect,
    pub tile_size: u32,
    pub deepest_level: u8,
}

impl Meta {
    // Reads the meta file from disk. Panics on error
    pub fn from_disk<P: AsRef<Path>>(filename: P) -> Self {
        let meta = {
            let mut data = Vec::new();
            File::open(filename)
                .expect("Could not proto file.")
                .read_to_end(&mut data)
                .unwrap();
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
                .expect("Could not parse meta.pb")
        };

        if meta.version != CURRENT_VERSION {
            panic!(
                "Invalid version. We only support {}, but found {}.",
                CURRENT_VERSION, meta.version
            );
        }

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

pub use xray_proto_rust::proto;
