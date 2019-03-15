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
use std::path::PathBuf;
use std::sync::Arc;
use structopt::StructOpt;

/// HTTP web viewer for 3d points stored in OnDiskOctrees
#[derive(StructOpt, Debug)]
#[structopt(name = "points_web_viewer", about = "Visualizing points")]
pub struct CommandLineArguments {
    /// The octree directory to serve, including a trailing slash.
    /// this overrides the path_* options
    #[structopt(name = "DIR", parse(from_os_str))]
    octree_path: Option<PathBuf>,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "ip")]
    ip: String,
    /// instead of DIR: specify path prefix for octree dir
    #[structopt(long = "prefix", parse(from_os_str), conflicts_with = "octree_path")]
    path_prefix: Option<PathBuf>,
    /// Optional: suffix for subfolder of octree dir
    #[structopt(default_value = "", long = "suffix", parse(from_os_str))]
    path_suffix: PathBuf,
    /// instead of DIR: specify path folder for octree dir
    #[structopt(long = "octree_id", parse(from_os_str), conflicts_with = "octree_path")]
    octree_id: Option<PathBuf>,
    /// Cache items
    #[structopt(default_value = "3", long = "cache_items")]
    cache_max: usize,
}

/// init app state with command arguments
/// backward compatibilty is ensured
pub fn state_from(args: CommandLineArguments) -> Result<AppState, PointsViewerError> {
    let suffix = args.path_suffix;

    let app_state = match args.octree_path {
        Some(octree_directory) => {
            let mut prefix = octree_directory.parent().unwrap();
            if octree_directory.ends_with(&suffix) {
                prefix = prefix.parent().unwrap();
            }
            let octree_id = octree_directory.strip_prefix(&prefix)?;
            AppState::new(args.cache_max, prefix, suffix, octree_id)
        }
        None => {
            let prefix = args
                .path_prefix
                .ok_or_else(|| {
                    PointsViewerError::NotFound(
                        "Input argoment Syntax is incorrect: check prefix".to_string(),
                    )
                })
                .unwrap();
            let octree_id = args
                .octree_id
                .ok_or_else(|| {
                    PointsViewerError::NotFound(
                        "Input argoment Syntax is incorrect: check octree_id".to_string(),
                    )
                })
                .unwrap();
            AppState::new(args.cache_max, prefix, suffix, octree_id)
        }
    };
    Ok(app_state)
}

fn main() {
    let args = CommandLineArguments::from_args();

    let ip_port = format!("{}:{}", args.ip, args.port);

    // initialize app state
    let app_state: Arc<AppState> = Arc::new(state_from(args).unwrap());
    // The actix-web framework handles requests asynchronously using actors. If we need multi-threaded
    // write access to the Octree, instead of using an RwLock we should use the actor system.
    //put octree arc in cache

    let sys = actix::System::new("octree-server");

    //let _ = start_octree_server(app_state, &ip_port, octree_id);
    let _ = start_octree_server(app_state, &ip_port);

    println!("Starting http server: {}", &ip_port);
    let _ = sys.run();
}
