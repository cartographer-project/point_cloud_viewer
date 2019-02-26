use actix_web::http::Method;
use actix_web::{server, HttpRequest, HttpResponse};

use crate::backend::{NodesData, VisibleNodes};
use crate::backend_error::PointsViewerError;
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

/// octree server function
pub fn start_octree_server(
    octree: Arc<octree::Octree>,
    ip_port: &str,
) -> Result<(), PointsViewerError> {
    server::new(move || {
        let octree_cloned_visible_nodes = Arc::clone(&octree);
        let octree_cloned_nodes_data = Arc::clone(&octree);
        actix_web::App::new()
            .resource("/", |r| r.method(Method::GET).f(index))
            .resource("/app_bundle.js", |r| r.method(Method::GET).f(app_bundle))
            .resource("/app_bundle.js.map", |r| {
                r.method(Method::GET).f(app_bundle_source_map)
            })
            .resource("/visible_nodes", |r| {
                r.method(Method::GET)
                    .h(VisibleNodes::new(octree_cloned_visible_nodes))
            })
            .resource("/nodes_data", |r| {
                r.method(Method::POST)
                    .h(NodesData::new(octree_cloned_nodes_data))
            })
    })
    .bind(&ip_port)
    .unwrap_or_else(|_| panic!("Can not bind to {}", &ip_port))
    .start();
    Ok(())
}
