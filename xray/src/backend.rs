use iron;
use iron::mime::Mime;
use iron::prelude::*;
use router::Router;
use std::fs;
use std::io;
use std::path::PathBuf;
use std::sync::Arc;
use urlencoded::UrlEncodedQuery;
use {BoundingRect, Meta};

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

pub struct HandleNodeImage<T: XRay> {
    pub xray_provider: T,
}

impl<T: XRay + Send + 'static> iron::Handler for HandleNodeImage<T> {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let id = req.extensions.get::<Router>().unwrap().find("id");
        if id.is_none() {
            return Ok(Response::with(iron::status::NotFound));
        }
        let id = id.unwrap();
        let reply = itry!(
            self.xray_provider.get_node_image(&id),
            iron::status::NotFound
        );
        let content_type = "image/png".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

pub struct HandleMeta {
    pub meta: Arc<Meta>,
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

pub struct HandleNodesForLevel {
    pub meta: Arc<Meta>,
}

impl iron::Handler for HandleNodesForLevel {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let query = req.get_ref::<UrlEncodedQuery>().unwrap();
        let level_s = &query.get("level").unwrap()[0];
        let level = itry!(level_s.parse::<u8>(), iron::status::BadRequest);
        // Entries are column major.
        let matrix_entries: Vec<f32> = query.get("matrix").unwrap()[0]
            .split(',')
            .map(|s| s.parse::<f32>().unwrap())
            .collect();
        match self.meta.get_nodes_for_level(level, matrix_entries) {
            Ok(result) => {
                let reply = ::serde_json::to_string_pretty(&result).unwrap();
                let content_type = "application/json".parse::<Mime>().unwrap();
                Ok(Response::with((content_type, iron::status::Ok, reply)))
            }
            Err(s) => Ok(Response::with((iron::status::BadRequest, s))),
        }
    }
}

pub fn serve(
    prefix: &str,
    router: &mut Router,
    xray_provider: impl XRay + Send + 'static,
) -> io::Result<()> {
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
