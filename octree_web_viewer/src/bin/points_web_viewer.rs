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

use octree_web_viewer::utils;
use point_viewer::octree;
use std::path::PathBuf;
use std::sync::Arc;
use structopt::StructOpt;

/// HTTP web viewer for 3d points stored in OnDiskOctrees
#[derive(StructOpt, Debug)]
#[structopt(name = "points_web_viewer", about = "Visualizing points")]
struct CommandlineArguments {
    /// The octree directory to serve, including a trailing slash.
    octree_path: String,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "--port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "--ip")]
    ip: String,
}

fn main() {
    let args = CommandlineArguments::from_args();

    let ip_port = format!("{}:{}", args.ip, args.port);
    let octree_directory = PathBuf::from(&args.octree_path);
    // The actix-web framework handles requests asynchronously using actors. If we need multi-threaded
    // write access to the Octree, instead of using an RwLock we should use the actor system.

    let octree: Arc<octree::Octree> = {
        let octree = match octree::octree_from_directory(octree_directory) {
            Ok(octree) => octree,
            Err(err) => panic!("Could not load octree: {}", err),
        };
        Arc::from(octree)
    };
    let sys = actix::System::new("octree-server");

    let _ = utils::start_octree_server(octree, &ip_port);

    println!("Starting http server: {}", &ip_port);
    let _ = sys.run();
}
