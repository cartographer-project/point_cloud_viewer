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
use std::io::{self, Cursor};
use std::path::Path;

pub const CURRENT_VERSION: i32 = 2;

#[derive(Debug,Clone)]
pub struct Meta {
    pub nodes: FnvHashSet<NodeId>,
    pub bounding_rect: Rect,
    pub tile_size: u32,
    pub deepest_level: u8,
}

// TODO(sirver): This should all return errors.
impl Meta {
    pub fn from_disk<P: AsRef<Path>>(filename: P) -> io::Result<Self> {
        let proto = {
            let data = std::fs::read(filename)?;
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data))
                .map_err(|_| io::Error::new(io::ErrorKind::Other, "Could not parse meta.pb"))?
        };
        Ok(Self::from_proto(&proto))
    }

    // Reads the meta from the provided encoded protobuf.
    pub fn from_proto(proto: &proto::Meta) -> Self {
        if proto.version != CURRENT_VERSION {
            panic!(
                "Invalid version. We only support {}, but found {}.",
                CURRENT_VERSION, proto.version
            );
        }

        Meta {
            nodes: proto.nodes
                .iter()
                .map(|n| NodeId::new(n.level as u8, n.index))
                .collect(),
            bounding_rect: Rect::new(
                Point2::new(
                    proto.get_bounding_rect().get_min().x,
                    proto.get_bounding_rect().get_min().y,
                ),
                proto.get_bounding_rect().get_edge_length(),
            ),
            tile_size: proto.tile_size,
            deepest_level: proto.deepest_level as u8,
        }
    }
}

pub mod backend;
pub mod generation;

pub use xray_proto_rust::proto;
