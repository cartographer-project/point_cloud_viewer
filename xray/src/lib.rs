use fnv::FnvHashSet;
use nalgebra::{Matrix4, Point2, Point3};
use point_viewer::geometry::{Aabb, Frustum};
use point_viewer::math::sat::{ConvexPolyhedron, Relation};
use protobuf::Message;
use quadtree::{ChildIndex, Node};
use quadtree::{NodeId, Rect};
use serde_derive::Serialize;
use std::fs::File;
use std::io::{self, BufWriter, Cursor};
use std::path::Path;

// Version 2 -> 3: Change in Rect proto from Vector2f to Vector2d min and float to double edge_length.
// We are able to convert the proto on read, so the tools can still read version 2.
pub const CURRENT_VERSION: i32 = 3;
pub const META_FILENAME: &str = "meta.pb";
pub const IMAGE_FILE_EXTENSION: &str = "png";

lazy_static::lazy_static! {
    pub static ref META_PREFIX: &'static str = Path::new(META_FILENAME)
        .file_stem()
        .unwrap()
        .to_str()
        .unwrap();
    pub static ref META_EXTENSION: &'static str = Path::new(META_FILENAME)
        .extension()
        .unwrap()
        .to_str()
        .unwrap();
}

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
    pub min_x: f64,
    pub min_y: f64,
    pub edge_length: f64,
}

// TODO(sirver): This should all return errors.
impl Meta {
    pub fn from_disk<P: AsRef<Path>>(filename: P) -> io::Result<Self> {
        let proto = {
            let data = std::fs::read(filename)?;
            protobuf::parse_from_reader::<proto::Meta>(&mut Cursor::new(data)).map_err(|_| {
                io::Error::new(
                    io::ErrorKind::Other,
                    format!("Could not parse {}", META_FILENAME),
                )
            })?
        };
        Ok(Self::from_proto(&proto))
    }

    pub fn to_disk<P: AsRef<Path>>(&self, filename: P) -> io::Result<()> {
        let mut buf_writer = BufWriter::new(File::create(filename.as_ref())?);
        self.to_proto()
            .write_to_writer(&mut buf_writer)
            .map_err(|_| {
                io::Error::new(
                    io::ErrorKind::Other,
                    format!("Couldn't write meta to {:?}.", filename.as_ref()),
                )
            })
    }

    // Reads the meta from the provided encoded protobuf.
    pub fn from_proto(proto: &proto::Meta) -> Self {
        match proto.version {
            2 => eprintln!(
                "Data is an older xray quadtree version: {}, current would be {}. \
                 If feasible, try upgrading this xray quadtree using `upgrade_xray_quadtree`.",
                proto.version, CURRENT_VERSION
            ),
            CURRENT_VERSION => (),
            _ => panic!(
                "Invalid version. We only support {}, but found {}.",
                CURRENT_VERSION, proto.version
            ),
        }

        let bounding_rect = proto.get_bounding_rect();
        let (min, edge_length) = bounding_rect.min.clone().into_option().map_or_else(
            || {
                let deprecated_min = bounding_rect.get_deprecated_min();
                (
                    Point2::new(f64::from(deprecated_min.x), f64::from(deprecated_min.y)),
                    f64::from(bounding_rect.get_deprecated_edge_length()),
                )
            },
            |v| (Point2::new(v.x, v.y), bounding_rect.get_edge_length()),
        );
        Meta {
            nodes: proto
                .nodes
                .iter()
                .map(|n| NodeId::new(n.level as u8, n.index))
                .collect(),
            bounding_rect: Rect::new(min, edge_length),
            tile_size: proto.tile_size,
            deepest_level: proto.deepest_level as u8,
        }
    }

    pub fn to_proto(&self) -> proto::Meta {
        let mut meta = proto::Meta::new();
        let bounding_rect = meta.mut_bounding_rect();
        bounding_rect.set_edge_length(self.bounding_rect.edge_length());
        let min = bounding_rect.mut_min();
        min.set_x(self.bounding_rect.min().x);
        min.set_y(self.bounding_rect.min().y);
        meta.set_deepest_level(u32::from(self.deepest_level));
        meta.set_tile_size(self.tile_size);
        meta.set_version(CURRENT_VERSION);

        for node_id in &self.nodes {
            let mut proto = proto::NodeId::new();
            proto.set_index(node_id.index());
            proto.set_level(u32::from(node_id.level()));
            meta.mut_nodes().push(proto);
        }

        meta
    }

    pub fn get_root_node(&self) -> Option<Node> {
        self.nodes
            .iter()
            .min_by_key(|node_id| node_id.level())
            .map(|node_id| Node {
                id: *node_id,
                bounding_rect: self.bounding_rect.clone(),
            })
    }

    pub fn iter_level<'a>(&'a self, level: u8) -> impl Iterator<Item = NodeId> + 'a {
        self.nodes
            .iter()
            .copied()
            .filter(move |node_id| node_id.level() == level)
    }

    pub fn get_nodes_for_level(
        &self,
        level: u8,
        matrix_entries: &[f32],
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
            #[rustfmt::skip]
            let mat = Matrix4::new(
                e[0], e[4],  e[8], e[12],
                e[1], e[5],  e[9], e[13],
                e[2], e[6], e[10], e[14],
                e[3], e[7], e[11], e[15],
            );
            nalgebra::convert(mat)
        };
        let frustum =
            Frustum::from_matrix4(matrix).ok_or("Unable to create frustum from matrix")?;
        let frustum_isec = frustum.intersector().cache_separating_axes_for_aabb();
        let mut result = Vec::new();
        let mut open = vec![Node::from_node_id_and_root_bounding_rect(
            NodeId::root(),
            self.bounding_rect.clone(),
        )];
        while let Some(node) = open.pop() {
            let aabb = Aabb::new(
                Point3::new(node.bounding_rect.min().x, node.bounding_rect.min().y, -0.1),
                Point3::new(node.bounding_rect.max().x, node.bounding_rect.max().y, 0.1),
            );

            if frustum_isec.intersect(&aabb.compute_corners()) == Relation::Out
                || !self.nodes.contains(&node.id)
            {
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
                    open.push(node.get_child(&ChildIndex::from_u8(i)));
                }
            }
        }
        Ok(result)
    }
}

pub mod backend;
pub mod build_quadtree;
pub mod colormap;
pub mod generation;
pub mod inpaint;
pub mod utils;

pub use xray_proto_rust::proto;
