use cgmath::Point2;
use cgmath::{Matrix4, Point3};
use collision::{Aabb3, Frustum, Relation};
use fnv::FnvHashSet;
use quadtree::{ChildIndex, Node};
use quadtree::{NodeId, Rect};
use serde_derive::Serialize;
use std::io::{self, Cursor};
use std::path::Path;

pub const CURRENT_VERSION: i32 = 2;

#[derive(Debug, Clone)]
pub struct Meta {
    pub nodes: FnvHashSet<NodeId>,
    pub bounding_rect: Rect,
    pub tile_size: u32,
    pub deepest_level: u8,
}

#[derive(Serialize, Debug)]
pub struct NodeMeta {
    pub id: String,
    pub bounding_rect: BoundingRect,
}

#[derive(Clone, Serialize, Debug)]
pub struct BoundingRect {
    pub min_x: f32,
    pub min_y: f32,
    pub edge_length: f32,
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
            nodes: proto
                .nodes
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

    pub fn get_nodes_for_level(
        &self,
        level: u8,
        matrix_entries: Vec<f32>,
    ) -> Result<Vec<NodeMeta>, String> {
        // TODO(sirver): This function could actually work much faster by not traversing the
        // levels, but just finding the covering of the rectangle of the current bounding box.
        //
        // Also it should probably not take a frustum but the view bounding box we are interested in.
        if matrix_entries.len() != 4 * 4 {
            return Err(format!(
                "Expected {} entries in matrix, got {}",
                4 * 4,
                matrix_entries.len()
            ));
        }

        let matrix = {
            let e = &matrix_entries;
            Matrix4::new(
                e[0], e[1], e[2], e[3], e[4], e[5], e[6], e[7], e[8], e[9], e[10], e[11], e[12],
                e[13], e[14], e[15],
            )
        };
        let frustum =
            Frustum::from_matrix4(matrix).ok_or("Unable to create frustum from matrix")?;
        let mut result = Vec::new();
        let mut open = vec![Node::root_with_bounding_rect(self.bounding_rect.clone())];
        while !open.is_empty() {
            let node = open.pop().unwrap();
            let aabb = Aabb3::new(
                Point3::new(node.bounding_rect.min().x, node.bounding_rect.min().y, -0.1),
                Point3::new(node.bounding_rect.max().x, node.bounding_rect.max().y, 0.1),
            );

            if frustum.contains(&aabb) == Relation::Out || !self.nodes.contains(&node.id) {
                continue;
            }

            if node.level() == level {
                result.push(NodeMeta {
                    id: node.id.to_string(),
                    bounding_rect: BoundingRect {
                        min_x: node.bounding_rect.min().x,
                        min_y: node.bounding_rect.min().y,
                        edge_length: node.bounding_rect.edge_length(),
                    },
                });
            } else {
                for i in 0..4 {
                    open.push(node.get_child(ChildIndex::from_u8(i)));
                }
            }
        }
        Ok(result)
    }
}

pub mod backend;
pub mod generation;

pub use xray_proto_rust::proto;
