// Copyright 2018 Google Inc.
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

use clap::value_t;
use futures::future::Future;
use futures::Stream;
use grpcio::{ChannelBuilder, Environment};
use std::path::{Path, PathBuf};
use std::str::FromStr;
use std::sync::Arc;

use point_viewer::octree::{octree_from_directory, OctreeFactory};
use point_viewer::Point;
use point_viewer_grpc::proto_grpc::OctreeClient;
use point_viewer_grpc::service::start_grpc_server;
use point_viewer_grpc_proto_rust::proto;

fn main() {
    let matches = clap::App::new("octree_benchmark")
        .args(&[
            clap::Arg::with_name("port")
                .help("Port for the server to listen on for connections. [50051]")
                .long("port")
                .takes_value(true),
            clap::Arg::with_name("no-client")
                .help("Do not actually send points, only read them on the server.")
                .long("no-client")
                .takes_value(false),
            clap::Arg::with_name("num-points")
                .help("Number of points to stream. [50000000]")
                .long("num-points")
                .takes_value(true),
            clap::Arg::with_name("octree_directory")
                .help("Input directory of the octree directory to serve.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let octree_directory = PathBuf::from(
        matches
            .value_of("octree_directory")
            .expect("octree_directory not given"),
    );
    let num_points = u64::from_str(matches.value_of("num-points").unwrap_or("50000000"))
        .expect("num-points needs to be a number");
    if matches.is_present("no-client") {
        server_benchmark(&octree_directory, num_points)
    } else {
        let port = value_t!(matches, "port", u16).unwrap_or(50051);
        full_benchmark(&octree_directory, num_points, port)
    }
}

fn server_benchmark(octree_directory: &Path, num_points: u64) {
    let octree = octree_from_directory(octree_directory).unwrap_or_else(|_| {
        panic!(
            "Could not create octree from '{}'",
            octree_directory.display()
        )
    });
    let mut counter: u64 = 0;
    let mut num_prints: u64 = 1;
    octree.all_points().for_each(|p: Vec<Point>| {
        counter += p.len() as u64;
        if counter >= num_prints * 1_000_000 {
            println!("Streamed {}M points", counter / 1_000_000);
            num_prints += 1;
        }
        if counter >= num_points {
            std::process::exit(0)
        }
    });
}

fn full_benchmark(octree_directory: &Path, num_points: u64, port: u16) {
    let octree_factory = OctreeFactory::new();
    let mut server = start_grpc_server("0.0.0.0", port, octree_directory, octree_factory);
    server.start();

    let env = Arc::new(Environment::new(1));
    let ch = ChannelBuilder::new(env).connect(&format!("localhost:{}", port));
    let client = OctreeClient::new(ch);

    let req = proto::GetAllPointsRequest::new();
    let receiver = client.get_all_points(&req).unwrap();

    let mut counter: u64 = 0;

    'outer: for rep in receiver.wait() {
        for _pos in rep.expect("Stream error").get_positions().iter() {
            if counter % 1_000_000 == 0 {
                println!("Streamed {}M points", counter / 1_000_000);
            }
            counter += 1;
            if counter == num_points {
                break 'outer;
            }
        }
    }

    let _ = server.shutdown().wait();
}
