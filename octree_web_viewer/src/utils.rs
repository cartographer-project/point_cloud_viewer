use crate::backend::{get_nodes_data, get_visible_nodes};
use crate::backend_error::PointsViewerError;
use crate::state::AppState;
use actix_web::http::Method;
use actix_web::{server, HttpRequest, HttpResponse};
use std::sync::Arc;

const INDEX_HTML: &str = include_str!("../client/index.html");
const APP_BUNDLE: &str = include_str!("../../target/app_bundle.js");
const APP_BUNDLE_MAP: &str = include_str!("../../target/app_bundle.js.map");

pub fn index(_req: &HttpRequest<Arc<AppState>>) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(INDEX_HTML)
}

pub fn app_bundle(_req: &HttpRequest<Arc<AppState>>) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE)
}

pub fn app_bundle_source_map(_req: &HttpRequest<Arc<AppState>>) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE_MAP)
}

/// octree server function
pub fn start_octree_server(
    app_state: Arc<AppState>,
    ip_port: &str,
) -> Result<(), PointsViewerError> {
    
    
    server::new(move || {
        actix_web::App::with_state(Arc::clone(&app_state))
            .resource("/", |r| r.method(Method::GET).f(index))
            .resource("/app_bundle.js", |r| r.method(Method::GET).f(app_bundle))
            .resource("/app_bundle.js.map", |r| {
                r.method(Method::GET).f(app_bundle_source_map)
            })
            .resource("/visible_nodes/{uuid}/", |r| {
                r.f(get_visible_nodes) 
            })
            .resource("/nodes_data/{uuid}/", |r| {
                r.with_async(get_nodes_data) 
            })
    })
    .bind(&ip_port)
    .unwrap_or_else(|_| panic!("Can not bind to {}", &ip_port))
    .start();
    Ok(())
}
