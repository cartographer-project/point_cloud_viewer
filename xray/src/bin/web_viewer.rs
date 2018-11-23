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

#[macro_use]
extern crate clap;
extern crate iron;
extern crate router;
extern crate xray;

use iron::mime::Mime;
use iron::prelude::*;
use router::Router;
use std::path::PathBuf;

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

    let mut router = Router::new();
    router.get("/", index);
    router.get("/app_bundle.js", app_bundle);
    router.get("/app_bundle.js.map", app_bundle_source_map);
    xray::backend::serve("", &mut router, xray::backend::OnDiskXRayProvider::new(quadtree_directory).expect("Could not serve from directory. Not a xray directory?")).unwrap();

    println!("Listening on port {}.", port);
    Iron::new(router).http(("0.0.0.0", port)).unwrap();
}
