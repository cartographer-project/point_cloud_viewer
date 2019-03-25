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
use std::path::{Path, PathBuf};
use std::sync::Arc;
use structopt::StructOpt;

/// HTTP web viewer for 3d points stored in OnDiskOctrees
///
/// Octrees that share a common path and (optional) identical subfolder structure can be served
/// just by giving a full directory path, <common_octree_path>/<octree_identifier>/<optional_subfolder_suffix> as <DIR>.
/// The option "--suffix_depth" allows to specify where the actual octree files reside in relation to the octree identifier.
#[derive(StructOpt, Debug)]
#[structopt(name = "points_web_viewer", about = "Visualizing points")]
pub struct CommandLineArguments {
    /// The octree directory to serve, including a trailing slash.
    /// this overrides <--prefix> and <--octree_id> options
    #[structopt(name = "DIR", parse(from_os_str))]
    octree_path: PathBuf,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "ip")]
    ip: String,
    /// optional: specify suffix depth (how many subfolders after uuid)
    #[structopt(default_value = "0", long = "suffix_depth")]
    suffix_depth: u8,
    /// maximum number of octrees stored in the map
    #[structopt(default_value = "20", long = "cache_items")]
    cache_max: usize,
}

/// init app state with command arguments
/// backward compatibilty is ensured
pub fn state_from(args: CommandLineArguments) -> Result<AppState, PointsViewerError> {
    let octree_factory = OctreeFactory::new();
    let mut suffix = Path::new("");
    let mut suffix_depth = args.suffix_depth;
    let mut octree_path: PathBuf;
    // parse path if suffix is specified
    if suffix_depth > 0 {
        let mut octree_directory = args.octree_path.parent().unwrap_or_else(|| Path::new(""));
        while suffix_depth > 1 {
            octree_directory = octree_directory.parent().unwrap_or_else(|| Path::new(""));
            suffix_depth -= 1;
        }
        suffix = args.octree_path.strip_prefix(&octree_directory)?;
        octree_path = PathBuf::from(octree_directory);
    } else {
        octree_path = args.octree_path;
    }

    let prefix = octree_path.parent().unwrap_or_else(|| Path::new(""));
    let octree_id = octree_path.strip_prefix(&prefix)?;
    Ok(AppState::new(
        args.cache_max,
        prefix,
        suffix,
        octree_id.to_str().unwrap(),
        octree_factory,
    ))
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
