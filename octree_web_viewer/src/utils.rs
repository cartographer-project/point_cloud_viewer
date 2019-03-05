use crate::backend::{NodesData, VisibleNodes};
use crate::backend_error::PointsViewerError;

use actix_web::http::Method;
use actix_web::{server, HttpRequest, HttpResponse};
use point_viewer::octree;
use std::collections::HashMap;
use std::sync::{Arc, RwLock};

const INDEX_HTML: &str = include_str!("../client/index.html");
const APP_BUNDLE: &str = include_str!("../../target/app_bundle.js");
const APP_BUNDLE_MAP: &str = include_str!("../../target/app_bundle.js.map");

pub fn index(_req: &HttpRequest) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(INDEX_HTML)
}

pub fn app_bundle(_req: &HttpRequest) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE)
}

pub fn app_bundle_source_map(_req: &HttpRequest) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE_MAP)
}

#[derive(Clone)]
pub struct OctreeKeyParams {
    /// Location prefix, including
    prefix: String,
    /// Tree ID
    suffix: String,
}

impl OctreeKeyParams {
    pub fn get_octree_address(&self, octree_key: String) -> Result<String, PointsViewerError> {
        Ok(format!("{}/{}/{}", self.prefix, octree_key, self.suffix))
    }
}

#[derive(Clone)]
pub struct AppState {
    /// LRU Cache for Octrees
    pub octree_map: Arc<RwLock<HashMap<String, Arc<octree::Octree>>>>,
    //pub octree_factory: octree::OctreeFactory,
    pub key_params: OctreeKeyParams,
}

impl AppState {
    pub fn new(map_size: usize, prefix: impl Into<String>, suffix: impl Into<String>) -> Self {
        AppState {
            octree_map: Arc::new(RwLock::new(HashMap::with_capacity(map_size))),
            //octree_factory: octree::OctreeFactory::new(),
            key_params: OctreeKeyParams {
                prefix: prefix.into(),
                suffix: suffix.into(),
            },
        }
    }

    pub fn load_octree(
        &self,
        uuid: impl AsRef<String>,
    ) -> Result<Arc<octree::Octree>, PointsViewerError> {
        //exists
        let octree_id = uuid.as_ref();
        {
            let map = self.octree_map.read().unwrap();
            let octree = map.get(octree_id);

            if let Some(tree) = octree {
                return Ok(Arc::clone(&tree));
            }
        }
        return self.insert_octree(octree_id.to_string()); //todo ownwership
    }

    pub fn insert_octree(&self, uuid: String) -> Result<Arc<octree::Octree>, PointsViewerError> {
        let uuid_key = uuid.clone();
        let addr = &self.key_params.get_octree_address(uuid)?;
        let octree: Arc<octree::Octree> = Arc::from(octree::octree_from_directory(&addr)?);
        {
            let mut wmap = self.octree_map.write().unwrap(); //todo try?
            wmap.insert(uuid_key, Arc::clone(&octree));
        }
        Ok(octree)
    }
}

/// octree server function
pub fn start_octree_server(
    app_state: AppState,
    ip_port: &str,
    uuid: impl AsRef<String>,
) -> Result<(), PointsViewerError> {
    let octree = app_state.load_octree(uuid).unwrap();
    server::new(move || {
        let octree_cloned_visible_nodes = Arc::clone(&octree); //test
        let octree_cloned_nodes_data = Arc::clone(&octree);
        actix_web::App::new()
            //actix_web::App::with_state(app_state)
            .resource("/", |r| r.method(Method::GET).f(index))
            .resource("/app_bundle.js", |r| r.method(Method::GET).f(app_bundle))
            .resource("/app_bundle.js.map", |r| {
                r.method(Method::GET).f(app_bundle_source_map)
            })
            .resource("/visible_nodes/{uuid}/", |r| {
                r.h(VisibleNodes::new(octree_cloned_visible_nodes)) //todo
            })
            .resource("/nodes_data/{uuid}/", |r| {
                r.h(NodesData::new(octree_cloned_nodes_data)) //todo
            })
    })
    .bind(&ip_port)
    .unwrap_or_else(|_| panic!("Can not bind to {}", &ip_port))
    .start();
    Ok(())
}
