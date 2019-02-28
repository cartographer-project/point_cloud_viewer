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
struct CommandLineArguments {
    /// The octree directory to serve, including a trailing slash.
    #[structopt(name = "DIR", parse(from_os_str), required_unless ="use_custom")]
    octree_path: PathBuf,
    /// Port to listen on.
    #[structopt(default_value = "5433", long = "--port")]
    port: u16,
    /// IP string.
    #[structopt(default_value = "127.0.0.1", long = "--ip")]
    ip: String,
    /// use custom path prefix and suffix
    #[structopt(short = "c", long = "--custom")]
    use_custom: bool,
    /// Optional: prefix for path
    #[structopt(long = "--prefix", required_if("use_custom", true))]
    path_prefix: Option(String),
    /// Optional: suffix for path
    #[structopt(long = "--suffix", required_if("use_custom", true))]
    path_suffix: Option(String),
    /// Optional: octree id
    #[structopt(long = "--uuid", required_if("use_custom", true))] 
    octree_id: Option(String),
    /// Cache items
    #[structopt(default_value = "1", long = "--cache_items")]
    cache_max: usizeß,
    
}

pub fn state_from( args: CommandLineArguments) -> Result<AppState>{
     let mut octree_directory = args.octree_path;
     // todo remove let octree_directory = PathBuf::from(&args.octree_path);

     //resolve suffix: trailing backslash
     let suffix = match args.path_suffix{
         Some(path) => {if !path.ends_with('/'){ path.append("/");} path},
         None => "/"
     };

     //resolve prefix
     let prefix = match args.path_prefix {
         Some(path) => {if !path.ends_with('/'){ path.append("/");} path},
         None => octree_directory.parent()
     };
    //resolve current octree id
     let uuid = match args.octree_id{
         Some(uuid) => uuid,
         None => {let octree_id = octree_directory.strip_prefix(prefix)?;
                  let tmp_suffix = suffix;
                  if octree_id.ends_with(suffix){
                      octree_id = octree_id.parent();

                  }} //pop the last
     };
     
     //
     if args.use_custom && octree_directory.is_empty() {
        // if available create from input string
        octree_directory = Path::new(prefix).join(uuid).join(suffix);
     }
    // instantiate app_state
    let app_state =  AppState::new(args.cache_max, prefix, suffix);
    //if possible create first octree //todo factory
    let mut octree_bytesize = 0;
    let octree: Arc<octree::Octree> = {
        let octree = match octree::octree_from_directory(octree_directory) {
            Ok(octree) => octree,
            Err(err) => panic!("Could not load octree: {}", err),
        };
        Arc::new(octree)
    };
    //put octree arc in cache
    app_state.cache.put_arc(uuid, octree, );
}

fn main() {
    let args = CommandLineArguments::from_args();

    let ip_port = format!("{}:{}", args.ip, args.port);
    

    // initialize app state
    let mut app_state = state_from(args).unwrap();
    // The actix-web framework handles requests asynchronously using actors. If we need multi-threaded
    // write access to the Octree, instead of using an RwLock we should use the actor system.

    // TODO(catevita) octree factory


    let sys = actix::System::new("octree-server");

    let _ = utils::start_octree_server(octree, &ip_port);

    println!("Starting http server: {}", &ip_port);
    let _ = sys.run();
}
