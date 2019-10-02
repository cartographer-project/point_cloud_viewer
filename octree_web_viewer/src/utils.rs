use crate::backend::{get_nodes_data, get_visible_nodes};
use crate::backend_error::PointsViewerError;
use crate::state::AppState;
use actix_web::{web, HttpResponse, HttpServer};
use std::sync::Arc;

const INDEX_HTML: &str = include_str!("../client/index.html");
const APP_BUNDLE: &str = include_str!("../../target/app_bundle.js");
const APP_BUNDLE_MAP: &str = include_str!("../../target/app_bundle.js.map");

pub fn index() -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(INDEX_HTML)
}

pub fn app_bundle() -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE)
}

pub fn app_bundle_source_map() -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/html")
        .body(APP_BUNDLE_MAP)
}

pub fn get_init_tree(state: web::Data<Arc<AppState>>) -> HttpResponse {
    HttpResponse::Ok()
        .content_type("text/plain")
        .body(state.get_init_id())
}

/// octree server function
pub fn start_octree_server(
    app_state: Arc<AppState>,
    ip_port: &str,
) -> Result<(), PointsViewerError> {
    HttpServer::new(move || {
        actix_web::App::new()
            .data(Arc::clone(&app_state))
            .service(web::resource("/").route(web::get().to(index)))
            .service(web::resource("/app_bundle.js").route(web::get().to(app_bundle)))
            .service(
                web::resource("/app_bundle.js.map").route(web::get().to(app_bundle_source_map)),
            )
            .service(web::resource("/init_tree").to(get_init_tree))
            .service(web::resource("/visible_nodes/{octree_id}/").to(get_visible_nodes))
            .service(web::resource("/nodes_data/{octree_id}/").to_async(get_nodes_data))
    })
    .bind(&ip_port)
    .unwrap_or_else(|_| panic!("Can not bind to {}", &ip_port))
    .start();
    Ok(())
}
