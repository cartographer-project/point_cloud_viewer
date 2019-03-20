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
use std::path::{Path, PathBuf};
use std::sync::Arc;
use structopt::clap::ArgGroup;
use structopt::StructOpt;

fn group_inputs() -> ArgGroup<'static> {
    ArgGroup::with_name("path_stubs")
        .conflicts_with("octree_path")
        .requires_all(&["path_prefix", "octree_id"])
}
/// HTTP web viewer for 3d points stored in OnDiskOctrees
///
/// Octrees that share a common path can be served under the following possibilities.
/// #1 Octrees without subfolder structure can be passed just by giving a full directory path, <octreeroot>/<octreeid>, as <DIR>.
/// The tree identifier to the (last) folder can be changed from the client by GUI input.
/// #2 if Octrees reside under a root directory, with a common subfolder stucture <root>/OctreeID/<identicalSubPath>,
/// the program has to be launched specifying --root <root> and --subpath <octreeId>/<identicalSubPath>
/// the changes of the client will affect the <octreeId> only.
#[derive(StructOpt, Debug)]
#[structopt(
    name = "points_web_viewer",
    about = "Visualizing points",
    raw(group = "group_inputs()")
)]
pub struct CommandLineArguments {
    /// The octree directory to serve, including a trailing slash.
    /// this overrides <--prefix> and <--octree_id> options
    #[structopt(name = "DIR", parse(from_os_str))]
    octree_path: Option<PathBuf>,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "ip")]
    ip: String,
    /// instead of DIR: specify path prefix (root) for octree dir
    #[structopt(long = "root", parse(from_os_str), group = "path_stubs")]
    path_prefix: Option<PathBuf>,
    /// instead of DIR: specify path from root to the folder for octree dir
    #[structopt(long = "subpath", parse(from_os_str), group = "path_stubs")]
    octree_id: Option<PathBuf>,
    /// has suffix: the top most ID can be switched to navigate across different octrees
    /// Cache items
    #[structopt(default_value = "20", long = "cache_items")]
    cache_max: usize,
}

/// init app state with command arguments
/// backward compatibilty is ensured
pub fn state_from(args: CommandLineArguments) -> Result<AppState, PointsViewerError> {
    let mut suffix = PathBuf::from("");

    // if <DIR> define, obtain structure from <DIR> only
    let app_state = match args.octree_path {
        Some(octree_directory) => {
            let prefix_opt = octree_directory.parent();
            if suffix.to_string_lossy().is_empty() {
                if let Some(prefix) = prefix_opt {
                    let octree_id = octree_directory.strip_prefix(&prefix)?;
                    AppState::new(args.cache_max, prefix, suffix, octree_id.to_str().unwrap())
                } else {
                    // octree directory is root
                    AppState::new(
                        args.cache_max,
                        PathBuf::new(),
                        suffix,
                        octree_directory.to_string_lossy(),
                    )
                }
            } else {
                let mut components = octree_directory.components();
                let mut prefix: PathBuf = PathBuf::new();
                let mut tmp_octree_id = components.next();
                while !suffix.as_path().eq(components.as_path()) {
                    prefix.push(tmp_octree_id.unwrap());
                    tmp_octree_id = components.next();
                }

                AppState::new(
                    args.cache_max,
                    prefix,
                    suffix,
                    (tmp_octree_id.unwrap().as_ref() as &Path).to_string_lossy(),
                )
            }
        }
        //<DIR> is undefined: root and subpath requireds
        None => {
            let prefix = args
                .path_prefix
                .ok_or_else(|| {
                    PointsViewerError::NotFound(
                        "Input argument Syntax is incorrect: check octree root path".to_string(),
                    )
                })
                .unwrap();
            let mut octree_id = args
                .octree_id
                .ok_or_else(|| {
                    PointsViewerError::NotFound(
                        "Input argument Syntax is incorrect: check octree_id".to_string(),
                    )
                })
                .unwrap()
                .parent();
            match octree_id {
                Some(id) => {
                    // todo (cvitadello) rela
                    suffix = args.octree_id.strip_prefix(octree_id);
                    AppState::new(
                        args.cache_max,
                        prefix,
                        suffix.unwrap_or(|| PathBuf::from("")),
                        octree_id,
                    )
                }
            }
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
    // put octree arc in cache

    let sys = actix::System::new("octree-server");
    let _ = start_octree_server(app_state, &ip_port);

    println!("Starting http server: {}", &ip_port);
    let _ = sys.run();
}
