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

#[macro_use]
extern crate clap;
extern crate cgmath;
extern crate floating_duration;
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate point_viewer_grpc_proto_rust;
extern crate stats;

use cgmath::Vector3;
use floating_duration::TimeFormat;
use futures::future::Future;
use futures::Stream;
use grpcio::{ChannelBuilder, Environment};
use point_viewer::octree::OnDiskOctree;
use point_viewer::{InternalIterator, Point};
use point_viewer_grpc::proto_grpc::OctreeClient;
use point_viewer_grpc::service::start_grpc_server;
use point_viewer_grpc_proto_rust::proto;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::Arc;
use stats::OnlineStats;

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
        server_benchmark(octree_directory, num_points)
    } else {
        let port = value_t!(matches, "port", u16).unwrap_or(50051);
        full_benchmark(octree_directory, num_points, port)
    }
}

#[derive(Debug, Default)]
struct RunningStats {
    count: u64,
    x_stats: OnlineStats,
    y_stats: OnlineStats,
    z_stats: OnlineStats,
}

impl RunningStats {
    pub fn update(&mut self, p: &Vector3<f32>) {
        self.count += 1;
        self.x_stats.add(p.x);
        self.y_stats.add(p.y);
        self.z_stats.add(p.z);
    }
}

fn server_benchmark(octree_directory: PathBuf, num_points: u64) {
    let octree = OnDiskOctree::new(&octree_directory).expect(&format!(
        "Could not create octree from '{}'",
        octree_directory.display()
    ));
    let now = ::std::time::Instant::now();
    let mut stats = RunningStats::default();
    octree.all_points().for_each(|p: &Point| {
        stats.update(&p.position);
        if stats.count % 1000000 == 0 {
            println!("Streamed {}M points", stats.count / 1_000_000);
        }
        if stats.count == num_points {
            println!(
                "Streamed {} points in {}.",
                stats.count,
                TimeFormat(now.elapsed())
            );
            println!("Running Stats:\n{:#?}", stats);
            std::process::exit(0)
        }
    });

}

fn full_benchmark(octree_directory: PathBuf, num_points: u64, port: u16) {
    let mut server = start_grpc_server(octree_directory, "0.0.0.0", port);
    server.start();

    let now = ::std::time::Instant::now();
    let mut stats = RunningStats::default();
    {
        let env = Arc::new(Environment::new(1));
        let ch = ChannelBuilder::new(env).connect(&format!("localhost:{}", port));
        let client = OctreeClient::new(ch);
        let req = proto::GetAllPointsRequest::new();
        let receiver = client.get_all_points(&req).unwrap();

        'outer: for rep in receiver.wait() {
            for pos in rep.expect("Stream error").get_positions().iter() {
                stats.update(&Vector3::new(pos.x, pos.y, pos.z));
                if stats.count % 1000000 == 0 {
                    println!("Streamed {}M points", stats.count / 1_000_000);
                }
                if stats.count == num_points {
                    break 'outer;
                }
            }
        }
    }

    println!(
        "Streamed {} points in {}.",
        stats.count,
        TimeFormat(now.elapsed())
    );
    println!("Running Stats:\n{:#?}", stats);

    let _ = server.shutdown().wait();
}
