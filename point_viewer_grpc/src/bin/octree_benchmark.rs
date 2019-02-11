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
extern crate futures;
extern crate grpcio;
extern crate point_viewer;
extern crate point_viewer_grpc;
extern crate point_viewer_grpc_proto_rust;

use point_viewer_grpc_proto_rust::proto;
use std::path::PathBuf;
use std::str::FromStr;
use std::sync::Arc;
use std::thread;

use futures::future::Future;
use futures::Stream;
use grpcio::{ChannelBuilder, Environment};
use point_viewer::octree::OnDiskOctree;
use point_viewer::{InternalIterator, Point};
use point_viewer_grpc::proto_grpc::OctreeClient;
use point_viewer_grpc::service::start_grpc_server;

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
            clap::Arg::with_name("num-threads")
                .help("The number of concurrent requests to split the query into.")
                .long("num-threads")
                .takes_value(true),
            clap::Arg::with_name("cq-count")
                .help("The number of concurrent requests to split the query into.")
                .long("cq-count")
                .takes_value(true),
            clap::Arg::with_name("rspcq")
                .help("The number of concurrent requests to split the query into.")
                .long("rspcq")
                .takes_value(true),
            clap::Arg::with_name("cq-count-client")
                .help("The number of concurrent requests to split the query into.")
                .long("cq-count-client")
                .takes_value(true),
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
    let num_threads = u64::from_str(matches.value_of("num-threads").unwrap_or("1"))
        .expect("num-threads needs to be a number");
    let cq_count = usize::from_str(matches.value_of("cq-count").unwrap_or("1"))
        .expect("cq-count needs to be a number");
    let rspcq = usize::from_str(matches.value_of("rspcq").unwrap_or("1"))
        .expect("rspcq needs to be a number");
    let cq_count_client = usize::from_str(matches.value_of("cq-count-client").unwrap_or("1"))
        .expect("cq-count-client needs to be a number");
    if matches.is_present("no-client") {
        server_benchmark(octree_directory, num_points, num_threads)
    } else {
        let port = value_t!(matches, "port", u16).unwrap_or(50051);
        full_benchmark(octree_directory, num_points, port, num_threads, cq_count, rspcq, cq_count_client)
    }
}


fn server_benchmark(octree_directory: PathBuf, num_points: u64, num_threads: u64) {
    let mut handles = Vec::with_capacity(num_threads as usize);
    for i in 0..num_threads {
        let octree = OnDiskOctree::new(&octree_directory).expect(&format!(
            "Could not create octree from '{}'",
            octree_directory.display()
        ));

        handles.push(thread::spawn(move || {
            let mut counter: u64 = 0;
            octree.all_points(i as i32, num_threads as i32).for_each(|_p: &Point| {
                if counter % 1000000 == 0 {
                    println!("Streamed {}M points", counter / 1000000);
                }
                counter += 1;
                if counter == num_points/num_threads {
                    std::process::exit(0)
                }
            });
        }));
    };
    for handle in handles {
        handle.join().unwrap();
    }
}


fn full_benchmark(octree_directory: PathBuf, num_points: u64, port: u16, num_threads: u64, cq_count: usize, rspcq: usize, cq_count_client: usize) {
    let mut server = start_grpc_server(octree_directory, "0.0.0.0", port, cq_count, rspcq);
    server.start();

    let env = Arc::new(Environment::new(cq_count_client));
    let ch = ChannelBuilder::new(env).connect(&format!("localhost:{}", port));

    let mut handles = Vec::with_capacity(num_threads as usize);
    let mut counter: u64 = 0;
    for i in 0..num_threads {
        let client = OctreeClient::new(ch.clone());
        handles.push(thread::Builder::new().name(format!("client {}", i)).spawn(move || {
            let mut req = proto::GetAllPointsParallelRequest::new();
            req.set_reqIndex(i as i32);
            req.set_total(num_threads as i32);
            let receiver = client.get_all_points_parallel(&req).unwrap();
            'outer: for rep in receiver.wait() {
                for _pos in rep.expect("Stream error").get_positions().iter() {
                    if counter % 1000000 == 0 {
                        println!("Streamed {}M points", counter / 1000000);
                    }
                    counter += 1;
                    if counter == num_points/num_threads {
                        break 'outer;
                    }
                }
            }
        }).unwrap());
    };
    for handle in handles {
        handle.join().unwrap();
    }

    let _ = server.shutdown().wait();
}
