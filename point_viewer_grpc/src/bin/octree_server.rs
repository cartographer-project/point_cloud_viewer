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
use futures::sync::oneshot;
use futures::Future;
use std::io::Read;
use std::path::PathBuf;
use std::{io, thread};

use point_viewer_grpc::service::start_grpc_server;

fn main() {
    let matches = clap::App::new("octree_server")
        .args(&[
            clap::Arg::with_name("port")
                .help("Port to listen on for connections. [50051]")
                .long("port")
                .takes_value(true),
            clap::Arg::with_name("octree_directory")
                .help("Input directory of the octree directory to serve.")
                .index(1)
                .required(true),
        ])
        .get_matches();

    let port = value_t!(matches, "port", u16).unwrap_or(50051);
    let octree_directory = PathBuf::from(matches.value_of("octree_directory").unwrap());
    let mut server = start_grpc_server(&octree_directory, "0.0.0.0", port);
    server.start();

    for &(ref host, port) in server.bind_addrs() {
        println!("listening on {}:{}", host, port);
    }
    let (tx, rx) = oneshot::channel();
    thread::spawn(move || {
        println!("Press ENTER to exit...");
        let _ = io::stdin().read(&mut [0]).unwrap();
        tx.send(())
    });
    let _ = rx.wait();
    let _ = server.shutdown().wait();
}
