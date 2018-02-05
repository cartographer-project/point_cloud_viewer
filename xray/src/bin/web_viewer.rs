// Copyright 2016 Google Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

extern crate cgmath;
#[macro_use]
extern crate clap;
extern crate collision;
#[macro_use]
extern crate iron;
extern crate protobuf;
extern crate quadtree;
extern crate router;
extern crate serde;
extern crate serde_json;
extern crate urlencoded;
extern crate xray;

#[macro_use]
extern crate serde_derive;

use cgmath::{Matrix4, Point3};
use collision::{Aabb3, Frustum, Relation};
use iron::mime::Mime;
use iron::prelude::*;
use quadtree::{ChildIndex, Node};
use router::Router;
use std::fs;
use std::io::Read;
use std::path::PathBuf;
use std::sync::Arc;
use urlencoded::UrlEncodedQuery;
use xray::Meta;

const INDEX_HTML: &'static str = include_str!("../../client/index.html");
const APP_BUNDLE: &'static str = include_str!("../../../target/xray_app_bundle.js");
const APP_BUNDLE_MAP: &'static str = include_str!("../../../target/xray_app_bundle.js.map");

fn index(_: &mut Request) -> IronResult<Response> {
    let content_type = "text/html".parse::<Mime>().unwrap();
    Ok(Response::with((content_type, iron::status::Ok, INDEX_HTML)))
}

fn app_bundle(_: &mut Request) -> IronResult<Response> {
    let content_type = "text/html".parse::<Mime>().unwrap();
    Ok(Response::with((content_type, iron::status::Ok, APP_BUNDLE)))
}

fn app_bundle_source_map(_: &mut Request) -> IronResult<Response> {
    let content_type = "text/html".parse::<Mime>().unwrap();
    Ok(Response::with((
        content_type,
        iron::status::Ok,
        APP_BUNDLE_MAP,
    )))
}

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

struct HandleNodeImage {
    directory: PathBuf,
}

impl iron::Handler for HandleNodeImage {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let id = req.extensions.get::<Router>().unwrap().find("id");
        if id.is_none() {
            return Ok(Response::with(iron::status::NotFound));
        }
        let id = id.unwrap();

        let mut filename = self.directory.join(id);
        filename.set_extension("png");
        let mut file = itry!(fs::File::open(filename), iron::status::NotFound);

        let mut reply = Vec::new();
        itry!(file.read_to_end(&mut reply));
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

        let mut result = Vec::new();
        let mut open = vec![
            Node::root_with_bounding_rect(self.meta.bounding_rect.clone()),
        ];
        while !open.is_empty() {
            let node = open.pop().unwrap();
            if node.level() == level {
                let aabb = Aabb3::new(
                    Point3::new(node.bounding_rect.min().x, node.bounding_rect.min().y, -0.1),
                    Point3::new(node.bounding_rect.max().x, node.bounding_rect.max().y, 0.1),
                );
                if frustum.contains(&aabb) == Relation::Out || !self.meta.nodes.contains(&node.id) {
                    continue;
                }
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

fn main() {
    let matches = clap::App::new("web_viewer")
        .args(&[
            clap::Arg::with_name("port")
                .help("Port to listen on for connections.")
                .long("port")
                .takes_value(true),
            clap::Arg::with_name("quadtree_directory")
                .help("Input directory of the quadtree directory to serve.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let port = value_t!(matches, "port", u16).unwrap_or(5434);
    let quadtree_directory = PathBuf::from(matches.value_of("quadtree_directory").unwrap());
    let meta = Arc::new(Meta::from_disk(quadtree_directory.join("meta.pb")));

    let mut router = Router::new();
    router.get("/", index);
    router.get("/app_bundle.js", app_bundle);
    router.get("/app_bundle.js.map", app_bundle_source_map);
    router.get(
        "/meta",
        HandleMeta {
            meta: Arc::clone(&meta),
        },
    );
    router.get(
        "/nodes_for_level",
        HandleNodesForLevel {
            meta: Arc::clone(&meta),
        },
    );
    router.get(
        "/node_image/:id",
        HandleNodeImage {
            directory: quadtree_directory.clone(),
        },
    );

    println!("Listening on port {}.", port);
    Iron::new(router).http(("0.0.0.0", port)).unwrap();
}
