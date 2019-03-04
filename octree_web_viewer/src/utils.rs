use crate::backend::{NodesData, VisibleNodes};
use crate::backend_error::PointsViewerError;

use actix_web::http::Method;
use actix_web::{server, HttpRequest, HttpResponse};
use lru::LruCache; // alternative use multicache::MultiCache; //size limited cache v 0.5.0
use point_viewer::octree;
use std::sync::Arc;

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

struct OctreeKeyParams {
    /// Location prefix, including
    prefix: String,
    /// Tree ID
    suffix: String,
}

impl OctreeKeyParams {
    pub fn get_octree_address(
        &self,
        octree_key: impl Into<String>,
    ) -> Result<String, PointsViewerError> {
        Ok(format!(
            "{}/{}/{}",
            self.prefix,
            octree_key.into(),
            self.suffix
        ))
    }
}

struct AppState {
    /// LRU Cache for Octrees
    // todo pub octree_cache : Cell<LruCache<String, Arc<Octree>>>,
    pub octree_cache: Arc<Mutex<Lru<String, octree::Octree>>>,
    //pub octree_factory: octree::OctreeFactory,
    pub key_params: OctreeKeyParams,
}

impl AppState {
    pub fn new(
        cache: Arc<Mutex<LruCache<String, octree::Octree>>>,
        prefix: String,
        suffix: String,
    ) -> Self {
        AppState {
            //todo: does it make sense to have item number or cache with byte size?
            //octree_cache : Cell::new(LruCache::new(max_cache_items)),//todo: mutable in docs examples
            octree_cache: cache,
            //octree_factory: octree::OctreeFactory::new(),
            key_params: OctreeKeyParams {
                prefix: prefix,
                suffix: suffix,
            },
        }
    }

    pub fn load_octree(&self, uuid: String) -> Result<Arc<octree::Octree>, PointsViewerError> {
        //exists
        if self.octree_cache.contains_key(&uuid) {
            Ok( self.octree_cache.get(&uuid)?)
        } else {
            let addr = &self.key_params.get_octree_address(uuid)?;
            //let octree :Arc<octree::Octree>> = octree_factory.generate(&addr)?;
            let octree: Arc<octree::Octree> =
                Arc::new(octree::octree_from_directory(&addr).map_err(|err| err.into())?); //from with
            self.octree_cache.put(uuid, octree);
            Ok(Arc::clone(octree))
        }
    }
}

/// octree server function
pub fn start_octree_server(app_state: AppState, ip_port: &str, uuid: Into<String>) -> Result<(), PointsViewerError> {
    server::new(move || {

        let octree_cloned_visible_nodes = app_state.load_octree(uuid);
        let octree_cloned_nodes_data = Arc::clone(&octree);
        actix_web::App::new()
        //actix_web::App::with_state(app_state)
            .resource("/", |r| r.method(Method::GET).f(index))
            .resource("/app_bundle.js", |r| r.method(Method::GET).f(app_bundle))
            .resource("/app_bundle.js.map", |r| {
                r.method(Method::GET).f(app_bundle_source_map)
            })
            .resource("/visible_nodes/{uuid}/", |r| {
                .h(VisibleNodes::new(octree_cloned_visible_nodes)) //todo
            })
            .resource("/nodes_data/{uuid}/", |r| {
                .h(NodesData::new(octree_cloned_nodes_data)) //todo
            })
    })
    .bind(&ip_port)
    .unwrap_or_else(|_| panic!("Can not bind to {}", &ip_port))
    .start();
    Ok(())
}
