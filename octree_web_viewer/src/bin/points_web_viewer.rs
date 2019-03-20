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

use octree_web_viewer::backend_error::PointsViewerError;
use octree_web_viewer::state::AppState;
use octree_web_viewer::utils::start_octree_server;
use point_viewer::octree::OctreeFactory;
use std::path::PathBuf;
use std::sync::Arc;
use structopt::StructOpt;

/// HTTP web viewer for 3d points stored in OnDiskOctrees
#[derive(StructOpt, Debug)]
#[structopt(name = "points_web_viewer", about = "Visualizing points")]
pub struct CommandLineArguments {
    /// The octree directory to serve, including a trailing slash.
    #[structopt(name = "DIR", parse(from_os_str))]
    octree_path: PathBuf,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "ip")]
    ip: String,
    #[structopt(default_value = "20", long = "cache_items")]
    cache_max: usize,
}

/// init app state with command arguments
/// backward compatibilty is ensured
pub fn state_from(args: CommandLineArguments) -> Result<AppState, PointsViewerError> {
    // initial implementation: suffix from args not yet supported
    let suffix = PathBuf::from("");
    let prefix_opt = args.octree_path.parent();
    let octree_factory = OctreeFactory::new();
    if let Some(prefix) = prefix_opt {
        let octree_id = args.octree_path.strip_prefix(&prefix)?;
        return Ok(AppState::new(
            args.cache_max,
            prefix,
            suffix,
            octree_id.to_str().unwrap(),
            octree_factory,
        ));
    } else {
        // octree directory is root
        return Ok(AppState::new(
            args.cache_max,
            PathBuf::new(),
            suffix,
            args.octree_path.to_string_lossy(),
            octree_factory,
        ));
    }
}

fn main() {
    let args = CommandLineArguments::from_args();

    let ip_port = format!("{}:{}", args.ip, args.port);

    // initialize app state
    let app_state: Arc<AppState> = Arc::new(state_from(args).unwrap());
    // The actix-web framework handles requests asynchronously using actors. If we need multi-threaded
    // write access to the Octree, instead of using an RwLock we should use the actor system.
    // put octree arc in cache

    let sys = actix::System::new("octree-server");
    let _ = start_octree_server(app_state, &ip_port);

    println!("Starting http server: {}", &ip_port);
    let _ = sys.run();
}
