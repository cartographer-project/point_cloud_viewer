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

extern crate byteorder;
extern crate point_viewer;
extern crate router;
extern crate time;
extern crate urlencoded;
#[macro_use]
extern crate clap;
extern crate iron;
extern crate json;

use byteorder::{LittleEndian, WriteBytesExt};
use iron::mime::Mime;
use iron::prelude::*;
use point_viewer::math::{CuboidLike, Matrix4f};
use point_viewer::octree;
use router::Router;
use std::io::Read;
use std::path::PathBuf;
use std::sync::{Arc, RwLock};
use urlencoded::UrlEncodedQuery;

const INDEX_HTML: &'static str = include_str!("../client/index.html");
const APP_BUNDLE: &'static str = include_str!("../target/app_bundle.js");
const APP_BUNDLE_MAP: &'static str = include_str!("../target/app_bundle.js.map");


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
    Ok(Response::with((content_type, iron::status::Ok, APP_BUNDLE_MAP)))
}

struct VisibleNodes {
    octree: Arc<RwLock<octree::Octree>>,
}

impl iron::Handler for VisibleNodes {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        // TODO(hrapp): This should not crash on error, but return a valid http response.
        let query = req.get_ref::<UrlEncodedQuery>().unwrap();
        let width: i32 = query.get("width").unwrap()[0].parse().unwrap();
        let height: i32 = query.get("height").unwrap()[0].parse().unwrap();
        let matrix = {
            // Entries are column major.
            let e: Vec<f32> = query.get("matrix").unwrap()[0]
                .split(',')
                .map(|s| s.parse::<f32>().unwrap())
                .collect();
            Matrix4f::new(
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
        let use_lod = {
            let lod: i32 = query.get("use_lod").unwrap()[0].parse().unwrap();
            if lod == 1 {
                octree::UseLod::Yes
            } else {
                octree::UseLod::No
            }
        };

        let visible_nodes = {
            let octree = self.octree.read().unwrap();
            octree.get_visible_nodes(&matrix, width, height, use_lod)
        };
        let mut reply = String::from("[");
        let visible_nodes_string = visible_nodes
            .iter()
            .map(|n| format!("[\"{}\", {}]", n.id, n.level_of_detail))
            .collect::<Vec<_>>()
            .join(",");
        reply.push_str(&visible_nodes_string);
        reply.push(']');
        let content_type = "application/json".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply)))
    }
}

// Javascript requires its arrays to be padded to 4 bytes.
fn pad(input: &mut Vec<u8>) {
    let pad = input.len() % 4;
    if pad == 0 {
        return;
    }
    for _ in 0..(4 - pad) {
        input.push(0);
    }
}

#[derive(Debug)]
struct NodeToLoad {
    id: octree::NodeId,
    level_of_detail: i32,
}

struct NodesData {
    octree: Arc<RwLock<octree::Octree>>,
}

impl iron::Handler for NodesData {
    fn handle(&self, req: &mut Request) -> IronResult<Response> {
        let start = time::precise_time_ns();
        let mut content = String::new();
        // TODO(hrapp): This should not crash on error, but return a valid http response.
        req.body.read_to_string(&mut content).unwrap();
        let data = json::parse(&content).unwrap();
        let nodes_to_load = data.members()
            .map(
                |e| {
                    NodeToLoad {
                        id: octree::NodeId::from_string(e[0].as_str().unwrap().to_string()),
                        level_of_detail: e[1].as_i32().unwrap(),
                    }
                }
            );

        // So this is godawful: We need to get data to the GPU without JavaScript herp-derping with
        // it - because that will stall interaction. The straight forward approach would be to ship
        // json with base64 encoded values - unfortunately base64 decoding in JavaScript yields a
        // string which cannot be used as a buffer. So we would need to manually convert this into
        // an Array with is very slow.
        // The alternative is to binary encode the whole request and parse it on the client side,
        // which requires careful constructing on the server and parsing on the client.
        let mut reply_blob = Vec::<u8>::new();

        let mut num_nodes_fetched = 0;
        let mut num_points = 0;
        let octree = self.octree.read().unwrap();
        for node in nodes_to_load {
            let mut node_data = octree
                .get_node_data(&node.id, node.level_of_detail)
                .unwrap();

            // Write the bounding box information.
            let min = node_data.meta.bounding_cube.min();
            reply_blob.write_f32::<LittleEndian>(min.x).unwrap();
            reply_blob.write_f32::<LittleEndian>(min.y).unwrap();
            reply_blob.write_f32::<LittleEndian>(min.z).unwrap();
            reply_blob
                .write_f32::<LittleEndian>(node_data.meta.bounding_cube.edge_length())
                .unwrap();

            // Number of points.
            reply_blob
                .write_u32::<LittleEndian>(node_data.meta.num_points as u32)
                .unwrap();

            // Position encoding.
            let bytes_per_coordinate = node_data.meta.position_encoding.bytes_per_coordinate();
            reply_blob.write_u8(bytes_per_coordinate as u8).unwrap();
            assert!(
                bytes_per_coordinate * node_data.meta.num_points as usize * 3 ==
                node_data.position.len()
            );
            assert!(node_data.meta.num_points as usize * 3 == node_data.color.len());
            pad(&mut reply_blob);


            reply_blob.append(&mut node_data.position);
            pad(&mut reply_blob);

            reply_blob.append(&mut node_data.color);
            pad(&mut reply_blob);

            num_nodes_fetched += 1;
            num_points += node_data.meta.num_points;
        }

        let duration_ms = (time::precise_time_ns() - start) as f32 / 1000000.;
        println!(
            "Got {} nodes with {} points ({}ms).",
            num_nodes_fetched,
            num_points,
            duration_ms
        );

        let content_type = "application/octet-stream".parse::<Mime>().unwrap();
        Ok(Response::with((content_type, iron::status::Ok, reply_blob)))
    }
}

fn main() {
    let matches = clap::App::new("server")
        .args(
            &[
                clap::Arg::with_name("port")
                    .help("Port to listen on for connections.")
                    .long("port")
                    .takes_value(true),
                clap::Arg::with_name("octree_directory")
                    .help("Input directory of the octree directory to serve.")
                    .index(1)
                    .required(true),
            ]
        )
        .get_matches();

    let port = value_t!(matches, "port", u16).unwrap_or(5433);
    let octree_directory = PathBuf::from(matches.value_of("octree_directory").unwrap());

    let otree = {
        let otree = match octree::Octree::new(octree_directory) {
            Ok(otree) => otree,
            Err(err) => panic!("Could not load octree: {}", err),
        };
        Arc::new(RwLock::new(otree))
    };

    let mut router = Router::new();
    router.get("/", index);
    router.get("/app_bundle.js", app_bundle);
    router.get("/app_bundle.js.map", app_bundle_source_map);
    router.get("/visible_nodes", VisibleNodes { octree: otree.clone() });
    router.post("/nodes_data", NodesData { octree: otree.clone() });

    println!("Listening on port {}.", port);
    Iron::new(router).http(("0.0.0.0", port)).unwrap();
}
