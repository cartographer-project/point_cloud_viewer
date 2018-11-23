use Meta;
use cgmath::{Matrix4, Point3};
use collision::{Aabb3, Frustum, Relation};
use iron;
use iron::mime::Mime;
use iron::prelude::*;
use quadtree::{ChildIndex, Node};
use router::Router;
use std::fs;
use std::io;
use std::path::PathBuf;
use std::sync::Arc;
use urlencoded::UrlEncodedQuery;

#[derive(Serialize, Debug)]
struct BoundingRect {
    min_x: f32,
    min_y: f32,
    edge_length: f32,
}

#[derive(Serialize, Debug)]
struct NodeMeta {
    id: String,
    bounding_rect: BoundingRect,
}

#[derive(Serialize, Debug)]
struct MetaReply {
    bounding_rect: BoundingRect,
    tile_size: u32,
    deepest_level: u8,
}

pub trait XRay: Sync {
    /// Returns the meta for the X-Ray.
    fn get_meta(&self) -> io::Result<Meta>;

    /// Returns the PNG blob of the node image for this 'image_id' or an Error.
    fn get_node_image(&self, node_id: &str) -> io::Result<Vec<u8>>;
}

pub struct OnDiskXRay {
    directory: PathBuf,
}

impl OnDiskXRay {
    pub fn new(directory: PathBuf) -> io::Result<Self> {
        let me = Self { directory };
        // See if we can find a meta directory.
        let _ = me.get_meta()?;
        Ok(me)
    }
}

impl XRay for OnDiskXRay {
    fn get_meta(&self) -> io::Result<Meta> {
        let meta = Meta::from_disk(self.directory.join("meta.pb"))?;
        Ok(meta)
    }

    fn get_node_image(&self, node_id: &str) -> io::Result<Vec<u8>> {
        let mut filename = self.directory.join(node_id);
        filename.set_extension("png");
        let data = fs::read(&filename)?;
        Ok(data)
    }
}

struct HandleNodeImage<T: XRay> {
    xray_provider: T, 
}

impl<T: XRay + Send + 'static> iron::Handler for HandleNodeImage<T> {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let id = req.extensions.get::<Router>().unwrap().find("id");
        if id.is_none() {
            return Ok(Response::with(iron::status::NotFound));
        }
        let id = id.unwrap();
        let reply = itry!(self.xray_provider.get_node_image(&id), iron::status::NotFound);
        let content_type = "image/png".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

struct HandleMeta {
    meta: Arc<Meta>,
}

impl iron::Handler for HandleMeta {
    fn handle(&self, _: &mut Request) -> IronResult<Response> {
        let result = MetaReply {
            bounding_rect: BoundingRect {
                min_x: self.meta.bounding_rect.min().x,
                min_y: self.meta.bounding_rect.min().y,
                edge_length: self.meta.bounding_rect.edge_length(),
            },
            tile_size: self.meta.tile_size,
            deepest_level: self.meta.deepest_level,
        };
        let reply = ::serde_json::to_string_pretty(&result).unwrap();
        let content_type = "application/json".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

struct HandleNodesForLevel {
    meta: Arc<Meta>,
}

impl iron::Handler for HandleNodesForLevel {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let query = req.get_ref::<UrlEncodedQuery>().unwrap();
        let level_s = &query.get("level").unwrap()[0];
        let level = itry!(level_s.parse::<u8>(), iron::status::NotFound);

        let matrix = {
            // Entries are column major.
            let e: Vec<f32> = query.get("matrix").unwrap()[0]
                .split(',')
                .map(|s| s.parse::<f32>().unwrap())
                .collect();
            Matrix4::new(
                e[0],
                e[1],
                e[2],
                e[3],
                e[4],
                e[5],
                e[6],
                e[7],
                e[8],
                e[9],
                e[10],
                e[11],
                e[12],
                e[13],
                e[14],
                e[15],
            )
        };
        let frustum = Frustum::from_matrix4(matrix).unwrap();

        // TODO(sirver): This function could actually work much faster by not traversing the
        // levels, but just finding the covering of the rectangle of the current bounding box.
        //
        // Also it should probably not take a frustum but the view bounding box we are interested
        // in.
        let mut result = Vec::new();
        let mut open = vec![
            Node::root_with_bounding_rect(self.meta.bounding_rect.clone()),
        ];
        while !open.is_empty() {
            let node = open.pop().unwrap();
            let aabb = Aabb3::new(
                Point3::new(node.bounding_rect.min().x, node.bounding_rect.min().y, -0.1),
                Point3::new(node.bounding_rect.max().x, node.bounding_rect.max().y, 0.1),
            );

            if frustum.contains(&aabb) == Relation::Out || !self.meta.nodes.contains(&node.id) {
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

        let reply = ::serde_json::to_string_pretty(&result).unwrap();
        let content_type = "application/json".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

pub fn serve(prefix: &str, router: &mut Router, xray_provider: impl XRay + Send + 'static) -> io::Result<()> {
    let meta = Arc::new(xray_provider.get_meta()?);
    router.get(
        format!("{}/meta", prefix),
        HandleMeta {
            meta: Arc::clone(&meta),
        },
    );
    router.get(
        format!("{}/nodes_for_level", prefix),
        HandleNodesForLevel {
            meta: Arc::clone(&meta),
        },
    );
    router.get(
        format!("{}/node_image/:id", prefix),
        HandleNodeImage {
            xray_provider: xray_provider,
        },
    );
    Ok(())
}
